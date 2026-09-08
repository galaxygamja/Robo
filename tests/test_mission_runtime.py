"""Measured-route integration fixtures; these are not hardware measurements."""
from __future__ import annotations

import copy
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import test_runtime_process as video_tests
from test_runtime_session import detection

from robo_control.control_loop import ClosedLoopController, MockActuatorBank
from robo_control.fleet import roles_from_scenario
from robo_control.mission_runtime import MOTION_PHASES, MissionExecutor
from robo_control.runtime_report import inspect_report
from robo_control.runtime_session import LiveControlSession

ROOT = Path(__file__).resolve().parents[1]


def fixture(task_ids=("move-D1",)):
    fleet = json.loads((ROOT / "config/qualifier_senior.json").read_text(encoding="utf-8"))
    roles = roles_from_scenario(fleet)
    plan = {"schema_version": 1, "coordinate_system": "bottom_left_x_right_y_up_mm",
        "radii_mm": {rid: 20. for rid in roles}, "cell_mm": 40., "obstacles_mm": [],
        "tasks": [{"task_id": task,
            "pickup": {"x_mm": 300., "y_mm": 500., "heading_rad": 0.},
            "drop": {"x_mm": 350., "y_mm": 500., "heading_rad": 0.},
            "retreat": {"x_mm": 300., "y_mm": 500., "heading_rad": math.pi}}
            for task in task_ids]}
    return fleet, roles, plan


class MissionHarness:
    def __init__(self, *, plan_change=None, task_ids=("move-D1",)):
        fleet, roles, plan = fixture(task_ids)
        if plan_change:
            plan_change(plan)
        self.session = LiveControlSession(roles=roles, field_size_mm=(1143., 1181.),
            source_name="webcam:test", session_id="mission-test", mission_plan=plan, fleet=fleet)
        self.seq = 0
        self.now = 10.
        self.positions = {"H1": [300., 500., 0.], "H2": [800., 800., 0.],
                          "B1": [800., 300., 0.], "B2": [300., 900., 0.]}
        self.last = None

    def step(self, *, feedback=None, change=None, no_record=False):
        self.seq += 1
        self.now = 10. + self.seq * .02
        row = detection(self.seq, self.now, ids=tuple(self.positions))
        row["robots"] = [{"robot_id": rid, "robot_center_mm": pose[:2], "heading_rad": pose[2]}
                         for rid, pose in self.positions.items()]
        if change:
            change(row)
        self.last = self.session.advance(None if no_record else row, self.now, feedback=feedback)
        return self.last

    def sensor(self, **changes):
        mission = self.session.mission
        active = mission.active
        values = {"session_id": "mission-test", "command_id": mission.command_id,
                  "robot_id": active.task.robot_id, "observed_at_s": self.now + .02,
                  "signals": {s: True for s in active.required_signals}, "synthetic": False}
        if active.phase == "confirm_clear":
            values.update(piece_id=active.piece.id, piece_x_mm=active.task.target_x_mm,
                          piece_y_mm=active.task.target_y_mm, piece_yaw_rad=0.)
        values.update(changes)
        return values

    def advance_to_sensor(self):
        # A test-only feedback plant. Production has no position integrator.
        for _ in range(1500):
            event = self.step()
            if event["closed_reason"]:
                raise AssertionError(event["closed_reason"])
            mission = self.session.mission
            if mission.active and mission.active.phase not in MOTION_PHASES:
                return event
            self.integrate(event)
        raise AssertionError("Did not reach sensor phase")

    def integrate(self, event):
        for command in event["actuator"]["robots"]:
            p = self.positions[command["robot_id"]]
            p[0] += command["velocity_world_mm_s"][0] * .02
            p[1] += command["velocity_world_mm_s"][1] * .02
            p[2] += command["angular_velocity_rad_s"] * .02


class DifferentialExecutionTests(unittest.TestCase):
    def command(self, x, y, heading=0., target_heading=0.):
        h = MissionHarness(plan_change=lambda p: p["tasks"][0]["pickup"].update(
            x_mm=x, y_mm=y, heading_rad=target_heading))
        h.positions["H1"][2] = heading
        for _ in range(5):
            event = h.step()
        return h, event

    def test_lateral_and_behind_goals_rotate_without_strafing(self):
        for x, y in ((300., 650.), (200., 500.)):
            h, event = self.command(x, y)
            command = next(c for c in event["actuator"]["robots"] if c["robot_id"] == "H1")
            self.assertEqual(command["velocity_world_mm_s"], [0., 0.])
            self.assertNotEqual(command["angular_velocity_rad_s"], 0.)
            self.assertEqual(command["wheel_velocity_rad_s"], [])
            self.assertFalse(event["device_io"])
            self.assertEqual(h.session.bank.drive_model, "differential_body")

    def test_heading_wrap_uses_short_turn_and_final_heading_is_required(self):
        controller = ClosedLoopController(roles={"H1": "hamster"}, drive_model="differential_body",
                                          goals={"H1": {"x_mm": 300., "y_mm": 500., "heading_rad": -math.pi+.1}})
        record = {"source_name": "test", "sequence": 1, "captured_at_s": 10.,
            "observation_usable": True, "stop_required": False, "tracks": [{
                "robot_id": "H1", "robot_center_mm": [300., 500.], "heading_rad": math.pi-.1,
                "observed_at_s": 10., "velocity_mm_s": [0., 0.], "angular_velocity_rad_s": 0.,
                "state": "observed", "valid_for_control": True}]}
        packet = controller.tick(record, 10.)
        self.assertGreater(packet["robots"][0]["angular_velocity_rad_s"], 0.)
        self.assertFalse(packet["robots"][0]["at_goal"])

    def test_bank_rejects_strafe_wheels_and_mixed_contract_and_expires(self):
        h, event = self.command(450., 500.)
        packet = event["command"]
        for mutate in (lambda p: p["robots"][0].update(velocity_world_mm_s=[0., 50.]),
                       lambda p: p["robots"][0].update(wheel_velocity_rad_s=[1., 1.]),
                       lambda p: p.update(drive_model="mecanum")):
            bank = MockActuatorBank(h.session.controller.ids, session_id="mission-test", drive_model="differential_body")
            changed = copy.deepcopy(packet)
            mutate(changed)
            self.assertNotEqual(bank.receive(changed, h.now)["reason"], "mock_active")
        bank = MockActuatorBank(h.session.controller.ids, session_id="mission-test", drive_model="differential_body")
        self.assertEqual(bank.receive(packet, h.now)["reason"], "mock_active")
        self.assertEqual(bank.tick(h.now + .3)["reason"], "command_watchdog")
        self.assertTrue(all(c["forward_velocity_mm_s"] == 0 for c in bank.snapshot()["robots"]))


class MissionExecutionTests(unittest.TestCase):
    def test_world_contract_keeps_unbound_piece_coordinates_unknown_and_expires(self):
        h = MissionHarness(plan_change=lambda p: p["tasks"][0]["pickup"].update(x_mm=450.))
        for _ in range(5):
            event = h.step()
        world = event["mission"]["world"]
        self.assertTrue(world["ready"])
        self.assertTrue(all(p["position_mm"] is None for p in world["pieces"]))
        self.assertEqual(world["session_id"], "mission-test")
        for _ in range(11):
            event = h.step(no_record=True)
        self.assertFalse(event["mission"]["world"]["ready"])
        self.assertTrue(all(c["forward_velocity_mm_s"] == 0 for c in event["actuator"]["robots"]))

    def test_match_clock_does_not_wait_for_initial_observation(self):
        h = MissionHarness()
        mission = h.session.mission
        mission.poll(10.)
        mission.poll(130.)
        self.assertEqual(mission.fault, "match_timeout")

    def test_waypoints_do_not_advance_from_time_or_commanded_velocity(self):
        h = MissionHarness(plan_change=lambda p: p["tasks"][0]["pickup"].update(x_mm=450.))
        for _ in range(100):
            event = h.step()
        self.assertEqual(event["mission"]["phase"], "approach")
        self.assertEqual(event["mission"]["completed_tasks"], [])
        self.assertEqual(h.positions["H1"], [300., 500., 0.])
        for command in event["actuator"]["robots"]:
            if command["robot_id"] != "H1":
                self.assertEqual(command["velocity_world_mm_s"], [0., 0.])

    def test_a_star_detours_static_obstacle_and_does_not_skip_unseen_waypoint(self):
        def alter(p):
            p["tasks"][0]["pickup"].update(x_mm=650.)
            p["obstacles_mm"] = [{"x_mm": 440., "y_mm": 440., "width_mm": 60., "height_mm": 120.}]
        h = MissionHarness(plan_change=alter)
        for _ in range(4):
            event = h.step()
        route = event["mission"]["route"]
        self.assertIsNotNone(route)
        self.assertTrue(any(abs(p["y_mm"]-500) > 60 for p in route))
        self.assertEqual(event["mission"]["waypoint_index"], 0)

    def test_reviewed_static_geometry_is_detached_from_the_callers_input(self):
        fleet, roles, plan = fixture()
        plan["obstacles_mm"] = [{"x_mm": 440., "y_mm": 700., "width_mm": 60., "height_mm": 100.}]
        mission = MissionExecutor(plan, fleet, roles=roles, field_size_mm=(1143., 1181.))
        plan["obstacles_mm"][0]["x_mm"] = 800.
        plan["obstacles_mm"].clear()
        self.assertFalse(mission._segment_clear((460., 650.), (460., 850.), "H1", {}))
        self.assertEqual(440., mission.plan["obstacles_mm"][0]["x_mm"])

    def test_blocked_route_latches_and_invalid_plan_fails_before_execution(self):
        def alter(p):
            p["tasks"][0]["pickup"].update(x_mm=650.)
            p["obstacles_mm"] = [{"x_mm": 450., "y_mm": 0., "width_mm": 100., "height_mm": 1181.}]
        h = MissionHarness(plan_change=alter)
        for _ in range(4):
            event = h.step()
        self.assertEqual(event["closed_reason"], "route_unavailable")
        self.assertTrue(all(c["forward_velocity_mm_s"] == 0 for c in event["actuator"]["robots"]))
        for mutate in (lambda p: p.update(coordinate_system="metres"),
                       lambda p: p["tasks"][0].update(task_id="unknown"),
                       lambda p: p["tasks"][0]["pickup"].update(x_mm=0),
                       lambda p: p.update(cell_mm=.01),
                       lambda p: p["tasks"].append(copy.deepcopy(p["tasks"][0])),
                       lambda p: p["radii_mm"].pop("H2")):
            fleet, roles, plan = fixture()
            mutate(plan)
            with self.assertRaises((ValueError, TypeError)):
                MissionExecutor(plan, fleet, roles=roles, field_size_mm=(1143., 1181.))

    def test_observation_loss_stops_and_requires_recovery_without_progress(self):
        h = MissionHarness(plan_change=lambda p: p["tasks"][0]["pickup"].update(x_mm=450.))
        for _ in range(5):
            h.step()
        event = h.step(change=lambda r: r.update(robots=[]))
        self.assertEqual(event["fresh_streak"], 0)
        self.assertIsNone(event["mission"]["route"])
        for i in range(3):
            event = h.step()
            if i < 2:
                self.assertTrue(all(c["forward_velocity_mm_s"] == 0 for c in event["actuator"]["robots"]))
        self.assertEqual(event["mission"]["phase"], "approach")

    def test_sensor_gate_rejects_missing_stale_synthetic_and_wrong_command_feedback(self):
        h = MissionHarness()
        event = h.advance_to_sensor()
        self.assertEqual(event["mission"]["phase"], "close_servo")
        for change in ({"synthetic": True}, {"observed_at_s": h.now-1},
                       {"command_id": "previous-task"}, {"session_id": "other"},
                       {"signals": {"servo_closed": "true"}}):
            event = h.step(feedback=h.sensor(**change))
            self.assertEqual(event["mission"]["phase"], "close_servo")
        event = h.step(feedback=h.sensor())
        self.assertEqual(event["mission"]["phase"], "confirm_grip")
        self.assertFalse(event["mission"]["manipulator_intent"]["dispatch_enabled"])

    def test_missing_sensor_times_out_and_loss_during_manipulation_never_reissues(self):
        for missing in (False, True):
            h = MissionHarness()
            h.advance_to_sensor()
            for _ in range(210):
                event = h.step(change=(lambda r: r.update(robots=[])) if missing else None)
                if event["closed_reason"]:
                    break
            self.assertIsNotNone(event["closed_reason"])
            self.assertEqual(event["mission"]["completed_tasks"], [])
            self.assertIsNone(event["mission"]["manipulator_intent"])

    def test_complete_two_tasks_with_explicit_sensor_fixtures_and_measured_replay(self):
        h = MissionHarness(task_ids=("move-D1", "move-D3"))
        phases = set()
        for _ in range(5000):
            mission = h.session.mission
            sensor = h.sensor() if mission.active and mission.active.phase not in MOTION_PHASES else None
            event = h.step(feedback=sensor)
            phases.add(event["mission"]["phase"])
            if event["closed_reason"]:
                break
            h.integrate(event)
        self.assertEqual(event["closed_reason"], "mission_completed")
        self.assertEqual(len(event["mission"]["completed_tasks"]), 2)
        self.assertIsNone(event["mission"]["score"])
        self.assertTrue({"approach", "carry", "confirm_clear", "retreat"} <= phases)
        self.assertTrue(all(c["forward_velocity_mm_s"] == 0 for c in event["actuator"]["robots"]))

    def test_release_requires_correct_measured_piece_and_destination(self):
        h = MissionHarness()
        for _ in range(2500):
            mission = h.session.mission
            if mission.active and mission.active.phase == "confirm_clear":
                break
            sensor = h.sensor() if mission.active and mission.active.phase not in MOTION_PHASES else None
            event = h.step(feedback=sensor)
            self.assertIsNone(event["closed_reason"])
            h.integrate(event)
        self.assertEqual(h.session.mission.active.phase, "confirm_clear")
        for changes in ({"piece_id": "wrong"}, {"piece_x_mm": float("nan")},
                        {"piece_x_mm": 10.}, {"signals": {"optical_clear": True}}):
            event = h.step(feedback=h.sensor(**changes))
            self.assertEqual(event["mission"]["phase"], "confirm_clear")

    def test_report_roundtrip_differential_final_zero_and_configuration(self):
        h = MissionHarness()
        events = [h.step() for _ in range(5)]
        events.append(h.session.close(h.now, "operator_stop"))
        header = {"schema_version": 1, "event": "session_started", "session_id": "mission-test",
                  "roles": h.session.controller.roles, "input_mode": "live_camera",
                  "output_mode": "dry_run_commands", "drive_model": "differential_body",
                  "device_io": False, "motion_permitted": False}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"report.jsonl"
            path.write_text("".join(json.dumps(row)+"\n" for row in [header, *events]), encoding="utf-8")
            result = inspect_report(path)
            self.assertTrue(result["final_stop_recorded"])
            self.assertEqual(result["drive_model"], "differential_body")
            self.assertEqual(result["mission"]["status"], "closed")


@unittest.skipIf(video_tests.cv2 is None, "OpenCV vision extra is not installed")
class MissionVideoIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.video = video_tests.ActualVideoRuntimeTests()
        self.video.setUp()
        self.addCleanup(self.video.doCleanups)

    def test_video_cli_connects_world_task_route_differential_commands_and_report(self):
        v = self.video
        fleet, _, plan = fixture()
        v.fleet.write_text(json.dumps(fleet), encoding="utf-8")
        plan["tasks"][0]["pickup"].update(x_mm=400., y_mm=840.)
        path = v.root / "mission.json"
        path.write_text(json.dumps(plan), encoding="utf-8")
        result = subprocess.run([sys.executable, "-m", "robo_control.runtime", *v.arguments(),
                                 "--mission", str(path)], cwd=ROOT, text=True, encoding="utf-8",
                                 capture_output=True, timeout=20, check=False)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        rows = [json.loads(line) for line in v.report.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(rows[0]["mission_plan"], plan)
        self.assertEqual(rows[0]["drive_model"], "differential_body")
        observations = {r["observation"]["sequence"]: r["observation"]
                        for r in rows[1:] if r["observation"] is not None}
        moving = [r for r in rows[1:] if any(c["angular_velocity_rad_s"] or c["forward_velocity_mm_s"]
                                          for c in r["actuator"]["robots"])]
        self.assertTrue(moving)
        for row in moving:
            self.assertTrue(row["mission"]["world"]["ready"])
            self.assertEqual(row["mission"]["active_task_id"], "move-D1")
            self.assertEqual(row["mission"]["phase"], "approach")
            self.assertEqual(row["mission"]["completed_tasks"], [])
            observation = observations[row["mission"]["world"]["source_sequence"]]
            measured = {r["robot_id"]: r["robot_center_mm"] for r in observation["robots"]}
            for robot in row["mission"]["world"]["robots"]:
                self.assertEqual(robot["position_mm"], measured[robot["robot_id"]])
        self.assertTrue(inspect_report(v.report)["final_stop_recorded"])

    def test_mission_report_cannot_overwrite_plan_and_goals_are_exclusive(self):
        from robo_control.runtime import main
        v = self.video
        fleet, _, plan = fixture()
        v.fleet.write_text(json.dumps(fleet), encoding="utf-8")
        path = v.root/"mission.json"
        path.write_text(json.dumps(plan), encoding="utf-8")
        before = path.read_bytes()
        args = v.arguments()
        args[args.index("--report")+1] = str(path)
        self.assertEqual(main([*args, "--mission", str(path)]), 2)
        self.assertEqual(path.read_bytes(), before)
        with self.assertRaises(SystemExit) as error:
            main([*v.arguments(), "--mission", str(path), "--goals", str(path)])
        self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
