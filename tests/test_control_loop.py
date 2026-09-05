from __future__ import annotations

import copy
import io
import json
import math
import unittest
from contextlib import redirect_stdout
from dataclasses import replace

from robo_control.control_loop import (
    ClosedLoopController, ControlLimits, MockActuatorBank, main,
    mecanum_wheels, predicted_conflicts, run_demo,
)


def measured(ids=("H1",), *, seq=1, stamp=1., positions=None):
    positions = positions or {rid: (0., i * 800., 0.) for i, rid in enumerate(ids)}
    return {"source_name": "camera:test:session-1", "sequence": seq, "captured_at_s": stamp,
            "observation_usable": True, "stop_required": False,
            "tracks": [{"robot_id": rid, "robot_center_mm": list(positions[rid][:2]),
                        "heading_rad": positions[rid][2], "observed_at_s": stamp,
                        "velocity_mm_s": [0., 0.], "angular_velocity_rad_s": 0.,
                        "state": "observed", "valid_for_control": True} for rid in ids]}


def controller(ids=("H1",), **kwargs):
    return ClosedLoopController(roles={rid: "hamster" if rid == "H1" else "beaver" for rid in ids},
                                goals={rid: {"x_mm": 500., "y_mm": i * 800.} for i, rid in enumerate(ids)},
                                session_id="test-session", **kwargs)


class ClosedLoopTests(unittest.TestCase):
    def assert_stopped(self, packet):
        self.assertTrue(all(row["velocity_world_mm_s"] == [0., 0.]
                            and row["angular_velocity_rad_s"] == 0.
                            and row["wheel_velocity_rad_s"] == [0.] * 4 for row in packet["robots"]))

    def test_measured_feedback_outputs_bounded_acceleration_and_speed(self):
        c = controller()
        previous = 0.
        for i in range(30):
            now = 1. + i * .02
            packet = c.tick(measured(seq=i + 1, stamp=now), now)
            speed = packet["robots"][0]["velocity_world_mm_s"][0]
            self.assertLessEqual(speed, c.limits.max_speed_mm_s)
            self.assertLessEqual(speed - previous, c.limits.acceleration_mm_s2 * .02 + 1e-8)
            self.assertFalse(packet["motion_permitted"])
            self.assertFalse(packet["hardware_ready"])
            self.assertFalse(packet["device_io"])
            self.assertTrue(packet["mock_motion_permitted"])
            previous = speed

    def test_hardware_cannot_be_enabled(self):
        for value in (True, "yes", 1, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                controller(enable_hardware=value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                MockActuatorBank(["H1"], session_id="test", enable_hardware=value)

    def test_invalid_goal_and_limit_configuration_is_rejected(self):
        for goals in ({"stranger": {"x_mm": 0, "y_mm": 0}}, {"H1": {"x_mm": True, "y_mm": 0}},
                      {"H1": {"x_mm": float("nan"), "y_mm": 0}}, {"H1": {"x_mm": 0}},
                      {"H1": {"x_mm": 1e308, "y_mm": 0}}):
            with self.subTest(goals=goals), self.assertRaises(ValueError):
                ClosedLoopController(roles={"H1": "hamster"}, goals=goals)
        for bad in (0., float("inf"), True):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                ControlLimits(braking_mm_s2=bad)
        with self.assertRaises(ValueError):
            ControlLimits(command_ttl_s=.5)
        with self.assertRaises(ValueError):
            controller(radii_mm={"stranger": 100.})

    def test_missing_unknown_duplicate_and_malformed_poses_stop_the_entire_fleet(self):
        ids = ("H1", "H2", "B1", "B2", "extra5")
        mutations = [lambda r: r["tracks"].pop(),
                     lambda r: r["tracks"][0].update(robot_id="unknown"),
                     lambda r: r["tracks"][0].update(robot_id="H2"),
                     lambda r: r["tracks"][0].update(robot_center_mm=[float("nan"), 0.]),
                     lambda r: r["tracks"][0].update(robot_center_mm=[1e308, 0.]),
                     lambda r: r["tracks"][0].update(robot_center_mm=[0.]),
                     lambda r: r["tracks"][0].update(heading_rad=True),
                     lambda r: r["tracks"][0].update(velocity_mm_s=None),
                     lambda r: r["tracks"][0].update(valid_for_control=False),
                     lambda r: r["tracks"][0].update(state="missing"),
                     lambda r: r.update(observation_usable=False),
                     lambda r: r.update(stop_required=True),
                     lambda r: r.update(tracks=None)]
        for mutate in mutations:
            c = controller(ids)
            c.tick(measured(ids), 1.)
            record = measured(ids, seq=2, stamp=1.02)
            mutate(record)
            packet = c.tick(record, 1.02)
            self.assertEqual(packet["status"], "hold")
            self.assert_stopped(packet)
            self.assertEqual(len(packet["robots"]), len(ids))

    def test_stale_and_out_of_order_frames_are_not_reused(self):
        c = controller()
        c.tick(measured(), 1.)
        packet = c.tick(measured(), 1.02)
        self.assertEqual(packet["stop_reason"], "out_of_order_frame")
        packet = c.tick(measured(seq=2, stamp=1.1), 1.4)
        self.assertEqual(packet["stop_reason"], "stale_frame")
        self.assert_stopped(packet)
        record = measured(seq=3, stamp=1.41)
        record["tracks"][0]["observed_at_s"] = 1.1
        self.assertEqual(c.tick(record, 1.41)["stop_reason"], "stale_or_missing_pose")
        record = measured(seq=4, stamp=1.42)
        record["source_name"] = "another-camera-session"
        self.assertEqual(c.tick(record, 1.42)["stop_reason"], "source_changed")

    def test_no_observation_and_future_time_fail_closed(self):
        c = controller()
        self.assert_stopped(c.tick(None, 1.))
        self.assert_stopped(c.tick(measured(stamp=2.), 1.1))
        with self.assertRaises(ValueError):
            c.tick(measured(), .5)

    def test_equal_host_stamp_requires_strictly_advancing_sequence(self):
        c = controller()
        first = c.tick(measured(seq=1, stamp=1.), 1.)
        second = c.tick(measured(seq=2, stamp=1.), 1.)
        self.assertTrue(second["mock_motion_permitted"])
        self.assertEqual(first["robots"][0]["velocity_world_mm_s"], second["robots"][0]["velocity_world_mm_s"])
        duplicate = c.tick(measured(seq=2, stamp=1.), 1.)
        self.assertEqual(duplicate["stop_reason"], "out_of_order_frame")
        self.assert_stopped(duplicate)

    def test_terminal_localization_latches_even_with_contradictory_valid_flags(self):
        for terminal in ({"status": "source_closed"}, {"tracking_session_closed": True}, {"localization_session_closed": True}):
            with self.subTest(terminal=terminal):
                c = controller()
                self.assertTrue(c.tick(measured(), 1.)["mock_motion_permitted"])
                record = measured(seq=2, stamp=1.02)
                record.update(terminal)
                closed = c.tick(record, 1.02)
                self.assertTrue(closed["localization_session_closed"])
                self.assertEqual(closed["stop_reason"], "localization_session_closed")
                self.assertFalse(closed["mock_motion_permitted"])
                self.assert_stopped(closed)
                # E-stop reset and apparently valid later frames cannot reopen
                # a closed source session. Construct a new controller instead.
                c.reset_emergency_stop()
                later = c.tick(measured(seq=3, stamp=1.04), 1.04)
                self.assertFalse(later["mock_motion_permitted"])
                self.assert_stopped(later)

    def test_pause_estop_latch_and_explicit_reset(self):
        c = controller()
        c.set_paused()
        self.assertEqual(c.tick(measured(), 1.)["stop_reason"], "paused")
        c.set_paused(False)
        self.assertTrue(c.tick(measured(seq=2, stamp=1.02), 1.02)["mock_motion_permitted"])
        c.emergency_stop()
        self.assertEqual(c.tick(measured(seq=3, stamp=1.04), 1.04)["stop_reason"], "emergency_stop")
        self.assertEqual(c.tick(measured(seq=4, stamp=1.06), 1.06)["stop_reason"], "emergency_stop")
        c.reset_emergency_stop()
        self.assertFalse(c.tick(measured(seq=4, stamp=1.06), 1.08)["mock_motion_permitted"])
        self.assertTrue(c.tick(measured(seq=5, stamp=1.1), 1.1)["mock_motion_permitted"])

    def test_at_goal_and_heading_wrap(self):
        c = controller()
        c.set_goals({"H1": {"x_mm": 0., "y_mm": 0., "heading_rad": -math.pi + .01}})
        packet = c.tick(measured(positions={"H1": (0., 0., math.pi - .01)}), 1.)
        self.assertEqual(packet["status"], "at_goal")
        self.assert_stopped(packet)

    def test_4_5_6_12_all_pairs_are_checked(self):
        for n in (4, 5, 6, 12):
            ids = tuple(f"unit{i}" for i in range(n))
            positions = {rid: (0., 0., 0.) for rid in ids}
            c = controller(ids)
            packet = c.tick(measured(ids, positions=positions), 1.)
            self.assertEqual(packet["stop_reason"], "predicted_collision")
            self.assertEqual(len(packet["conflicts"]), n * (n - 1) // 2)
            self.assert_stopped(packet)

    def test_explicit_subset_goals_leave_other_robots_stationary(self):
        c = controller(("H1", "H2"))
        c.set_goals({"H2": {"x_mm": 500., "y_mm": 800.}})
        packet = c.tick(measured(("H1", "H2")), 1.)
        self.assertEqual(packet["robots"][0]["velocity_world_mm_s"], [0., 0.])
        self.assertGreater(packet["robots"][1]["velocity_world_mm_s"][0], 0.)


class CollisionAndKinematicsTests(unittest.TestCase):
    def test_mecanum_straight_sideways_rotation_and_world_heading(self):
        self.assertEqual(mecanum_wheels(100., 0., 0., 0.), [5.] * 4)
        self.assertEqual(mecanum_wheels(0., 100., 0., 0.), [-5., 5., 5., -5.])
        self.assertEqual(mecanum_wheels(0., 0., 1., 0.), [-5.75, 5.75, -5.75, 5.75])
        for actual in mecanum_wheels(0., 100., 0., math.pi / 2):
            self.assertAlmostEqual(actual, 5.)

    def test_swept_crossing_detected_before_static_contact(self):
        poses = {"a": {"robot_center_mm": [-50., 0.], "velocity_mm_s": [0., 0.]},
                 "b": {"robot_center_mm": [0., -50.], "velocity_mm_s": [0., 0.]}}
        limits = replace(ControlLimits(), command_ttl_s=.001, braking_mm_s2=1e6, clearance_mm=1.)
        self.assertEqual(predicted_conflicts(poses, {"a": [0., 0.], "b": [0., 0.]}, {"a": 5., "b": 5.}, limits), [])
        self.assertEqual(len(predicted_conflicts(poses, {"a": [180., 0.], "b": [0., 180.]}, {"a": 5., "b": 5.}, limits)), 1)

    def test_measured_motion_and_braking_are_checked_even_for_zero_command(self):
        poses = {"a": {"robot_center_mm": [0., 0.], "velocity_mm_s": [100., 0.]},
                 "b": {"robot_center_mm": [130., 0.], "velocity_mm_s": [0., 0.]}}
        commands = {"a": [0., 0.], "b": [0., 0.]}
        radii = {"a": 20., "b": 20.}
        self.assertEqual(len(predicted_conflicts(poses, commands, radii)), 1)
        limits = replace(ControlLimits(), command_ttl_s=.001, braking_mm_s2=1e6)
        self.assertEqual(predicted_conflicts(poses, commands, radii, limits), [])

    def test_prediction_validates_complete_registry(self):
        with self.assertRaises(ValueError):
            predicted_conflicts({"a": {}}, {}, {})


class MockActuatorTests(unittest.TestCase):
    def setUp(self):
        self.c = controller(("H1", "H2", "B1", "B2", "extra5"))
        self.ids = self.c.ids
        self.bank = MockActuatorBank(self.ids, session_id=self.c.session_id)

    def packet(self, seq=1, stamp=1.):
        return self.c.tick(measured(self.ids, seq=seq, stamp=stamp), stamp)

    def assert_zero(self, state):
        self.assertEqual(len(state["robots"]), len(self.ids))
        self.assertTrue(all(r["wheel_velocity_rad_s"] == [0.] * 4 for r in state["robots"]))

    def test_watchdog_runs_without_any_controller_tick_and_at_exact_expiry(self):
        state = self.bank.receive(self.packet(), 1.)
        self.assertEqual(state["reason"], "mock_active")
        self.assertGreater(state["robots"][0]["velocity_world_mm_s"][0], 0.)
        self.assertEqual(self.bank.tick(1.299)["reason"], "mock_active")
        state = self.bank.tick(1.3)
        self.assertEqual(state["reason"], "command_watchdog")
        self.assert_zero(state)

    def test_delayed_delivery_expires_from_issue_not_receive_time(self):
        self.bank.receive(self.packet(), 1.25)
        self.assert_zero(self.bank.tick(1.3))

    def test_stale_duplicate_and_wrong_process_session_stop_all(self):
        packet = self.packet()
        self.bank.receive(packet, 1.)
        self.assert_zero(self.bank.receive(packet, 1.02))
        packet = self.packet(2, 1.04)
        packet["session_id"] = "restarted-controller"
        self.assertEqual(self.bank.receive(packet, 1.04)["reason"], "wrong_session")
        packet = self.packet(3, 1.06)
        self.assertEqual(self.bank.receive(packet, 1.4)["reason"], "stale_or_invalid_command_time")

    def test_estop_requires_independent_explicit_endpoint_reset(self):
        self.bank.receive(self.packet(), 1.)
        self.c.emergency_stop()
        self.assert_zero(self.bank.receive(self.packet(2, 1.02), 1.02))
        self.c.reset_emergency_stop()
        self.assertEqual(self.bank.receive(self.packet(3, 1.04), 1.04)["reason"], "emergency_stop")
        self.bank.reset_emergency_stop()
        self.assert_zero(self.bank.tick(1.05))
        self.assertEqual(self.bank.receive(self.packet(4, 1.06), 1.06)["reason"], "mock_active")

    def test_pause_packet_stops_without_waiting_for_watchdog(self):
        self.bank.receive(self.packet(), 1.)
        self.c.set_paused()
        state = self.bank.receive(self.packet(2, 1.02), 1.02)
        self.assertEqual(state["reason"], "paused")
        self.assert_zero(state)

    def test_forged_hardware_flags_malformed_partial_and_overspeed_rejected(self):
        packet = self.packet()
        mutations = [lambda p: p.update(device_io=True), lambda p: p.update(hardware_ready=True),
                     lambda p: p.update(motion_permitted=True), lambda p: p["robots"].pop(),
                     lambda p: p["robots"][0].update(robot_id="unknown"),
                     lambda p: p["robots"][0].update(robot_id="H2"),
                     lambda p: p["robots"][0].update(velocity_world_mm_s=[float("inf"), 0.]),
                     lambda p: p["robots"][0].update(velocity_world_mm_s=[1000., 0.]),
                     lambda p: p["robots"][0].update(wheel_velocity_rad_s=[99.] * 4),
                     lambda p: p.update(ttl_s=.31), lambda p: p.update(emergency_stop="false"),
                     lambda p: p.update(schema_version=True)]
        for mutate in mutations:
            bank = MockActuatorBank(self.ids, session_id=self.c.session_id)
            bad = copy.deepcopy(packet)
            mutate(bad)
            self.assert_zero(bank.receive(bad, 1.))

    def test_actuator_clock_backwards_clears_outputs(self):
        self.bank.receive(self.packet(), 1.)
        with self.assertRaises(ValueError):
            self.bank.tick(.5)
        self.assert_zero(self.bank.snapshot())

    def test_returned_snapshot_cannot_mutate_actuator_commands(self):
        state = self.bank.receive(self.packet(), 1.)
        state["robots"][0]["velocity_world_mm_s"][0] = 9999.
        state["robots"][0]["wheel_velocity_rad_s"][0] = 9999.
        current = self.bank.snapshot()["robots"][0]
        self.assertLess(current["velocity_world_mm_s"][0], 200.)
        self.assertLess(current["wheel_velocity_rad_s"][0], 20.)


class DemoTests(unittest.TestCase):
    def test_closed_loop_4_5_12_converges_on_synthetic_test_floor(self):
        for n in (4, 5, 12):
            result = run_demo(n)
            self.assertTrue(result["all_at_goal"])
            self.assertEqual(result["robot_count"], n)
            self.assertFalse(result["device_io"])
            self.assertEqual(result["after_command_loss"]["reason"], "command_watchdog")
            self.assertTrue(all(row["wheel_velocity_rad_s"] == [0.] * 4 for row in result["after_command_loss"]["robots"]))
            for rid, pose in result["final_poses"].items():
                goal = result["goals"][rid]
                self.assertLessEqual(math.dist(pose[:2], [goal["x_mm"], goal["y_mm"]]), 3.)
                self.assertLessEqual(abs(pose[2] - goal["heading_rad"]), .03)

    def test_demo_reproducible_json_cli(self):
        self.assertEqual(run_demo(4), run_demo(4))
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(main(["--demo", "--robots", "5", "--compact"]), 0)
        result = json.loads(out.getvalue())
        self.assertEqual(result["mode"], "mock_measured_pose_closed_loop")
        self.assertEqual(result["robot_count"], 5)


if __name__ == "__main__":
    unittest.main()
