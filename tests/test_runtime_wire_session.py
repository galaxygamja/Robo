from __future__ import annotations

import json
import subprocess
import sys
import unittest

import test_runtime_process as video_tests
from test_mission_runtime import ROOT, MissionHarness, fixture

from robo_control.runtime import _parser, main
from robo_control.runtime_report import inspect_report
from robo_control.runtime_session import LiveControlSession


class WireMissionHarness(MissionHarness):
    def __init__(self, *, pickup_x=450., link_options=None):
        super().__init__()
        fleet, roles, plan = fixture()
        plan["tasks"][0]["pickup"]["x_mm"] = pickup_x
        self.session = LiveControlSession(roles=roles, field_size_mm=(1143., 1181.),
            source_name="webcam:test", session_id="mission-test", mission_plan=plan, fleet=fleet,
            wire_fake=True, wire_options=link_options)

    def warm(self):
        for _ in range(4):
            event = self.step()
        return event


def local_zero(event):
    return all(r["forward_velocity_mm_s"] == r["angular_velocity_rad_s"] == 0 for r in event["actuator"]["robots"])


class RuntimeWireSessionTests(unittest.TestCase):
    def test_camera_startup_does_not_spend_handshake_and_initial_three_frames_are_zero(self):
        h = WireMissionHarness()
        for _ in range(100):
            event = h.step(no_record=True)
            self.assertFalse(h.session.wire.started)
            self.assertTrue(local_zero(event))
        for _ in range(3):
            event = h.step()
            self.assertTrue(local_zero(event))
        self.assertTrue(h.session.wire.started)
        event = h.step()
        self.assertFalse(local_zero(event))
        self.assertEqual("fake_wire", event["transport_mode"])

    def test_one_moving_three_idle_robots_keep_fresh_zero_permits(self):
        h = WireMissionHarness()
        for _ in range(45):
            event = h.step()
        self.assertIsNone(h.session.wire.fault)
        self.assertTrue(h.session.wire.ready)
        for robot in event["wire"]["robots"]:
            self.assertEqual("armed", robot["receiver"]["state"])
            if robot["robot_id"] != "H1":
                self.assertEqual(0, robot["receiver"]["v_mm_s"])
            self.assertGreater(robot["receiver"]["accepted_drive_count"], 20)
        self.assertEqual([], event["mission"]["completed_tasks"])

    def test_normal_at_goal_and_sensor_wait_keep_zero_not_repeated_disarm(self):
        h = WireMissionHarness(pickup_x=300.)
        for _ in range(35):
            event = h.step()
        self.assertIsNone(h.session.wire.fault)
        self.assertTrue(h.session.wire.ready)
        self.assertEqual("close_servo", event["mission"]["phase"])
        self.assertTrue(local_zero(event))
        self.assertTrue(all(r["receiver"]["state"] == "armed" for r in event["wire"]["robots"]))
        self.assertFalse(event["mission"]["manipulator_intent"]["dispatch_enabled"])

    def test_ordinary_ack_delay_is_backpressure_not_mission_interruption(self):
        h = WireMissionHarness()
        h.warm()
        h.session.wire.configure_link("H1", h.now, response_delay_s=.06)
        waiting = 0
        for _ in range(30):
            event = h.step()
            waiting += event["status"] == "wire_awaiting_ack"
            self.assertIsNone(h.session.wire.fault)
            self.assertIsNone(event["mission"]["fault"])
        self.assertGreater(waiting, 0)
        self.assertEqual("approach", event["mission"]["phase"])

    def test_lost_ack_latches_all_robots_and_good_images_do_not_restart(self):
        h = WireMissionHarness()
        h.warm()
        h.session.wire.configure_link("H1", h.now, drop_responses=1)
        for _ in range(25):
            event = h.step()
        self.assertIsNotNone(h.session.wire.fault)
        self.assertTrue(local_zero(event))
        counts = [r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"]]
        self.assertTrue(all(r["stop_requested"] for r in event["wire"]["robots"]))
        for _ in range(10):
            event = h.step()
        self.assertEqual(counts, [r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"]])
        self.assertEqual([], event["mission"]["completed_tasks"])

    def test_explicit_reconnect_needs_confirmation_and_three_new_observations(self):
        h = WireMissionHarness()
        h.warm()
        h.session.wire.stop(h.now, "injected_fault")
        event = h.step()
        old_links = {r["robot_id"]: r["receiver"]["link_id"] for r in event["wire"]["robots"]}
        with self.assertRaises(ValueError):
            h.session.reconnect_wire(h.now)
        event = h.session.reconnect_wire(h.now, operator_confirmed=True)
        self.assertTrue(local_zero(event))
        for index in range(3):
            event = h.step()
            self.assertEqual(index < 2, local_zero(event))
        self.assertIsNone(h.session.wire.fault)
        self.assertTrue(all(r["receiver"]["link_id"] != old_links[r["robot_id"]] for r in event["wire"]["robots"]))

    def test_missing_observation_requests_fleet_stop_and_cannot_implicitly_rearm(self):
        h = WireMissionHarness()
        h.warm()
        event = h.step(change=lambda r: r.update(robots=[], observation_complete=False))
        self.assertTrue(local_zero(event))
        self.assertIsNotNone(h.session.wire.fault)
        self.assertTrue(all(r["stop_requested"] for r in event["wire"]["robots"]))
        for _ in range(6):
            event = h.step()
        self.assertTrue(local_zero(event))

    def test_closed_observation_identity_cannot_be_reopened_by_wire_reconnect(self):
        h = WireMissionHarness()
        h.warm()
        h.step(change=lambda r: r.update(source_name="different-camera"))
        with self.assertRaisesRegex(ValueError, "closed observation"):
            h.session.reconnect_wire(h.now, operator_confirmed=True)

    def test_no_frame_polls_service_watchdog_without_resending_controller_command(self):
        h = WireMissionHarness()
        event = h.warm()
        before = [r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"]]
        for _ in range(12):
            event = h.step(no_record=True)
        self.assertTrue(local_zero(event))
        self.assertIsNotNone(h.session.wire.fault)
        self.assertEqual(before, [r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"]])

    def test_disconnect_stops_peers_but_does_not_fake_delivery_to_unreachable_robot(self):
        h = WireMissionHarness()
        h.warm()
        h.session.wire.configure_link("H1", h.now, connected=False)
        event = h.step()
        self.assertTrue(local_zero(event))
        states = {r["robot_id"]: r for r in event["wire"]["robots"]}
        self.assertFalse(states["H1"]["stop_acknowledged"])
        self.assertGreater(states["H1"]["receiver"]["v_mm_s"], 0)
        self.assertTrue(all(states[rid]["receiver"]["v_mm_s"] == 0 for rid in ("H2", "B1", "B2")))

    def test_close_with_lost_stop_keeps_unknown_ack_and_actual_fake_setpoint(self):
        h = WireMissionHarness()
        h.warm()
        h.session.wire.configure_link("H1", h.now, drop_requests=1)
        final = h.session.close(h.now)
        self.assertTrue(local_zero(final))
        self.assertTrue(final["wire"]["closed"])
        row = next(r for r in final["wire"]["robots"] if r["robot_id"] == "H1")
        self.assertTrue(row["stop_requested"])
        self.assertFalse(row["stop_acknowledged"])
        self.assertGreater(row["receiver"]["v_mm_s"], 0)
        self.assertEqual("unknown", row["receiver"]["execution"])
        with self.assertRaises(ValueError):
            h.session.reconnect_wire(h.now, operator_confirmed=True)

    def test_fault_during_manipulation_closes_and_cannot_repeat_action(self):
        h = WireMissionHarness(pickup_x=300.)
        for _ in range(20):
            event = h.step()
        self.assertEqual("close_servo", event["mission"]["phase"])
        h.session.wire.stop(h.now, "injected_link_fault")
        event = h.step()
        self.assertEqual("closed", event["status"])
        self.assertIn("manipulation_interrupted", event["closed_reason"])
        self.assertTrue(local_zero(event))
        with self.assertRaises(ValueError):
            h.session.reconnect_wire(h.now, operator_confirmed=True)

    def test_wire_mode_requires_explicit_differential_mission(self):
        with self.assertRaises(ValueError):
            LiveControlSession(roles={"H1": "hamster"}, field_size_mm=(1143, 1181), source_name="webcam:0", wire_fake=True)
        args = ["--video", "unused.avi", "--calibration", "unused.json", "--fleet", "unused.json",
                "--report", "unused-report.jsonl", "--wire-fake"]
        self.assertEqual(2, main(args))
        self.assertTrue(_parser().parse_args(args).wire_fake)

    def test_new_frame_on_old_observation_expiry_still_requests_fleet_stop(self):
        h = WireMissionHarness()
        h.warm()
        for _ in range(9):
            h.step(no_record=True)
        event = h.step()  # A new valid frame exactly 200 ms after the last one.
        self.assertTrue(local_zero(event))
        self.assertIsNotNone(h.session.wire.fault)
        self.assertTrue(all(r["stop_requested"] for r in event["wire"]["robots"]))

    def test_missed_supervisor_deadline_cancels_queued_drive_before_delivery(self):
        h = WireMissionHarness()
        h.warm()
        h.session.wire.configure_link("H1", h.now, request_delay_s=.05)
        event = h.step()
        before = next(r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
        event = h.session.advance(None, h.now+.11)
        after = next(r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
        self.assertEqual("supervisor_deadline_missed", event["closed_reason"])
        self.assertEqual(before, after)
        self.assertTrue(local_zero(event))

    def test_known_bad_new_frame_cancels_queued_drive_before_processing_ack(self):
        for mutation in (lambda r: r.update(source_name="foreign-camera"),
                         lambda r: r["robots"][0].update(heading_rad=float("nan"))):
            with self.subTest(mutation=mutation):
                h = WireMissionHarness()
                h.warm()
                h.session.wire.configure_link("H1", h.now, request_delay_s=.04)
                event = h.step()
                before = next(r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
                h.step(no_record=True)
                event = h.step(change=mutation)  # queued drive is due at this tick
                after = next(r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
                self.assertEqual(before, after)
                self.assertTrue(all(r["stop_requested"] for r in event["wire"]["robots"]))

    def test_reconnect_after_long_fault_does_not_reuse_old_observation_deadline(self):
        h = WireMissionHarness()
        h.warm()
        h.session.wire.stop(h.now, "test_fault")
        for _ in range(30):
            h.step()
        h.session.reconnect_wire(h.now, operator_confirmed=True)
        for _ in range(3):
            event = h.step()
        self.assertIsNone(h.session.wire.fault)
        self.assertFalse(local_zero(event))

    def test_broken_transport_clock_does_not_prevent_final_runtime_cleanup_record(self):
        h = WireMissionHarness()
        h.warm()
        with self.assertRaises(ValueError):
            h.session.wire.poll(h.now-1.)
        final = h.session.close(h.now, "runtime_failed")
        self.assertTrue(local_zero(final))
        self.assertTrue(final["wire"]["closed"])
        self.assertEqual("invalid_supervisor_clock", final["wire"]["fault"])
        self.assertTrue(all(r["stop_requested"] for r in final["wire"]["robots"]))
        self.assertTrue(any(not r["stop_acknowledged"] for r in final["wire"]["robots"]))


@unittest.skipIf(video_tests.cv2 is None, "OpenCV vision extra is not installed")
class RuntimeWireVideoTests(unittest.TestCase):
    def test_generated_video_to_world_mission_wire_receivers_and_report(self):
        video = video_tests.ActualVideoRuntimeTests()
        video.setUp()
        self.addCleanup(video.doCleanups)
        fleet, _, plan = fixture()
        video.fleet.write_text(json.dumps(fleet), encoding="utf-8")
        plan["tasks"][0]["pickup"].update(x_mm=400., y_mm=840.)
        path = video.root / "wire-mission.json"
        path.write_text(json.dumps(plan), encoding="utf-8")
        process = subprocess.run([sys.executable, "-m", "robo_control.runtime", *video.arguments(),
            "--mission", str(path), "--wire-fake"], cwd=ROOT, text=True, encoding="utf-8",
            capture_output=True, timeout=25, check=False)
        self.assertEqual(0, process.returncode, process.stderr + process.stdout)
        summary = json.loads(process.stdout)
        self.assertIsNone(summary["wire_fault"])
        self.assertTrue(summary["wire_stop_acknowledged"])
        rows = [json.loads(line) for line in video.report.read_text(encoding="utf-8").splitlines()]
        self.assertEqual("fake_wire", rows[0]["transport_mode"])
        moving = [r for r in rows[1:] if any(c["receiver"]["v_mm_s"] or c["receiver"]["omega_rad_s"]
                    for c in r["wire"]["robots"])]
        self.assertTrue(moving)
        self.assertTrue(all(r["mission"]["world"]["ready"] for r in moving))
        self.assertTrue(all(r["mission"]["completed_tasks"] == [] for r in moving))
        self.assertTrue(inspect_report(video.report)["final_wire_zero"])


if __name__ == "__main__":
    unittest.main()
