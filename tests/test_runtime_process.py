"""Real spawned-process failure tests. Fixtures never control a physical device."""

from __future__ import annotations

import json
import math
import multiprocessing
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

from robo_control.runtime import main, supervise
from robo_control.runtime_io import AsyncJsonlReport, LatestRecordMailbox
from robo_control.runtime_report import inspect_report
from robo_control.runtime_session import LiveControlSession

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

ROOT = Path(__file__).resolve().parents[1]


def _record(sequence):
    now = time.monotonic()
    return {"status": "detected", "sequence": sequence, "captured_at_s": now,
        "received_at_s": now, "source_name": "webcam:test", "is_replay": False,
        "coordinate_system": "bottom_left_x_right_y_up_mm", "field_size_mm": [1143., 1181.],
        "registered_robot_ids": ["H1"], "device_io": False, "observation_complete": True,
        "unknown_tag_ids": [], "duplicate_tag_ids": [], "objects": [],
        "robots": [{"robot_id": "H1", "robot_center_mm": [300., 500.], "heading_rad": 0.}]}


def _fault_worker(mailbox, stop, state, mode, ready, begin):
    # Separate Windows/Python import time from the camera fault being tested.
    # Real runtime startup deadlines are unchanged and tested below.
    ready.set()
    if not begin.wait(10.):
        return
    if mode == "startup_hang":
        stop.wait(5.)
        return
    state.value = 1
    for sequence in range(1, 7):
        mailbox.publish(_record(sequence))
        stop.wait(.04)
    if mode == "crash":
        os._exit(23)
    if mode == "dead_lock":
        mailbox.lock.acquire()
        os._exit(24)
    if mode == "close_hang":
        state.value = 2
        time.sleep(5.)
        return
    if mode == "late_result":
        record = _record(7)
        stop.wait(.35)
        # A real latest-only producer may intentionally drop a contended
        # publish. This fault fixture must deliver the stale record so the
        # assertion exercises rejection, not the separate no-input watchdog.
        while not mailbox.publish(record):
            if stop.wait(.002):
                return
    if mode == "uninterruptible":
        # Deliberately ignore the cooperative stop event, like a stuck driver.
        time.sleep(5.)
    else:
        stop.wait(5.)


class WorkerIsolationTests(unittest.TestCase):
    def run_fault(self, mode, duration=1., startup=.5):
        context = multiprocessing.get_context("spawn")
        mailbox, stop, state = LatestRecordMailbox(context), context.Event(), context.RawValue("i", 0)
        ready, begin = context.Event(), context.Event()
        session = LiveControlSession(roles={"H1": "hamster"}, source_name="webcam:test",
            field_size_mm=(1143., 1181.), goals={"H1": {"x_mm": 450., "y_mm": 500.}}, radii_mm={"H1": 40.})
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "runtime.jsonl"
            worker = context.Process(target=_fault_worker, args=(mailbox, stop, state, mode, ready, begin), daemon=True)
            worker.start()
            if not ready.wait(10.):
                worker.terminate()
                worker.join(2.)
                self.fail("Fault fixture Python process did not initialize within 10 seconds")
            report = AsyncJsonlReport(path)
            begin.set()
            summary = supervise(session, worker, mailbox, stop, state, report,
                                duration_s=duration, tick_hz=50., startup_timeout_s=startup)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertTrue(summary["worker_stopped"])
        self.assertTrue(summary["report_complete"])
        self.assertTrue(all(row["velocity_world_mm_s"] == [0., 0.]
                            for row in summary["final"]["actuator"]["robots"]))
        self.assertFalse(summary["device_io"])
        return summary, rows

    def test_blocked_camera_read_still_gets_regular_ticks_and_observation_expiry(self):
        summary, rows = self.run_fault("read_hang")
        self.assertEqual("duration_elapsed", summary["status"])
        moving = [row for row in rows if row["actuator"]["robots"][0]["velocity_world_mm_s"][0] > 0]
        self.assertTrue(moving)
        expired = [row for row in rows if row["status"] == "observation_watchdog"]
        self.assertTrue(expired)
        self.assertLess(expired[0]["last_observation_age_ms"], 260.)
        self.assertGreaterEqual(summary["ticks"], 30)

    def test_process_crash_or_abandoned_lock_stops_without_waiting_for_capture(self):
        for mode in ("crash", "dead_lock"):
            with self.subTest(mode=mode):
                summary, _ = self.run_fault(mode)
                self.assertEqual("camera_worker_exited", summary["status"])
                self.assertLess(summary["elapsed_s"], 1.)

    def test_startup_hang_has_bounded_timeout(self):
        summary, _ = self.run_fault("startup_hang", startup=.2)
        self.assertEqual("camera_startup_timeout", summary["status"])
        self.assertEqual(0, summary["delivered_frames"])

    def test_uninterruptible_capture_is_terminated_after_output_stop(self):
        summary, _ = self.run_fault("uninterruptible", duration=.6)
        self.assertLess(summary["elapsed_s"], 2.)
        self.assertEqual("duration_elapsed", summary["status"])

    def test_camera_release_hang_stops_after_eof_flag_even_without_terminal_message(self):
        summary, _ = self.run_fault("close_hang")
        self.assertEqual("video_eof", summary["status"])
        self.assertLess(summary["elapsed_s"], 1.5)

    def test_late_detector_result_cannot_revive_expired_motion(self):
        _, rows = self.run_fault("late_result", duration=1.6, startup=1.)
        rejected = [row for row in rows if row["status"] == "stale_observation"]
        self.assertTrue(rejected)
        self.assertTrue(all(row["actuator"]["robots"][0]["velocity_world_mm_s"] == [0., 0.]
                            for row in rejected))


@unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
class ActualVideoRuntimeTests(unittest.TestCase):
    def setUp(self):
        from robo_control.vision.calibration import FieldCalibration
        from robo_control.vision.tags import TagDetectorConfig
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.calibration = self.root / "camera.json"
        self.tags = self.root / "tags.json"
        self.fleet = self.root / "fleet.json"
        self.video = self.root / "four-robots.avi"
        self.report = self.root / "runtime.jsonl"
        FieldCalibration((640, 480), ((0, 0), (639, 0), (639, 479), (0, 479)), (1143, 1181)).save(self.calibration)
        TagDetectorConfig(tag_to_robot={0: "H1", 1: "H2", 2: "B1", 3: "B2"}).save(self.tags)
        self.fleet.write_text(json.dumps({"ground_robots": [
            {"id": rid, "tag_id": tag, "role": "hamster" if rid == "H1" else "beaver"}
            for tag, rid in enumerate(("H1", "H2", "B1", "B2"))]}))
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        gray = np.full((480, 640), 255, dtype=np.uint8)
        for tag, (x, y) in enumerate(((100, 100), (420, 100), (100, 320), (420, 320))):
            gray[y:y + 70, x:x + 70] = cv2.aruco.generateImageMarker(dictionary, tag, 70)
        writer = cv2.VideoWriter(str(self.video), cv2.VideoWriter_fourcc(*"MJPG"), 20., (640, 480))
        self.assertTrue(writer.isOpened(), "MJPG codec required for integration test")
        for _ in range(14):
            writer.write(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
        writer.release()

    def arguments(self):
        return ["--video", str(self.video), "--calibration", str(self.calibration),
                "--tags", str(self.tags), "--fleet", str(self.fleet), "--report", str(self.report),
                "--duration-s", "8", "--startup-timeout-s", "5"]

    def test_disk_video_spawn_detection_tracking_command_and_final_stop(self):
        result = subprocess.run([sys.executable, "-m", "robo_control.runtime", *self.arguments()],
            cwd=ROOT, text=True, encoding="utf-8", capture_output=True, timeout=15, check=False)
        self.assertEqual(0, result.returncode, result.stderr + result.stdout)
        summary = json.loads(result.stdout)
        self.assertGreater(summary["usable_observation_frames"], 3)
        self.assertEqual("video_eof", summary["status"])
        rows = [json.loads(line) for line in self.report.read_text().splitlines()]
        self.assertEqual("session_started", rows[0]["event"])
        self.assertEqual(rows[0]["configuration_id"], summary["configuration_id"])
        rows = rows[1:]
        observations = [row["observation"] for row in rows if row["observation"]]
        self.assertTrue(all(len(row["tracks"]) == 4 for row in observations))
        self.assertTrue(all(row["is_replay"] for row in rows))
        self.assertTrue(any(row["command"] is not None for row in rows))
        self.assertEqual("closed", rows[-1]["status"])
        self.assertFalse(rows[-1]["motion_permitted"])
        audit = inspect_report(self.report)
        self.assertTrue(audit["log_complete"])
        self.assertEqual("video_replay", audit["input_mode"])
        self.assertGreater(audit["usable_observation_frames"], 3)
        self.assertFalse(audit["physical_stop_verified"])

    def test_existing_report_and_input_output_alias_are_never_overwritten(self):
        self.report.write_text("keep this report")
        self.assertEqual(2, main(self.arguments()))
        self.assertEqual("keep this report", self.report.read_text())
        args = self.arguments()
        args[args.index("--report") + 1] = str(self.calibration)
        before = self.calibration.read_bytes()
        self.assertEqual(2, main(args))
        self.assertEqual(before, self.calibration.read_bytes())

    def test_explicit_goal_uses_detected_image_position_and_generates_nonzero_dry_run_command(self):
        goals = self.root / "goals.json"
        goals.write_text(json.dumps({"schema_version": 1,
            "coordinate_system": "bottom_left_x_right_y_up_mm",
            "goals": {"H1": {"x_mm": 400, "y_mm": 840}},
            "radii_mm": {"H1": 40, "H2": 40, "B1": 40, "B2": 40}}))
        result = subprocess.run([sys.executable, "-m", "robo_control.runtime", *self.arguments(),
                                "--goals", str(goals)], cwd=ROOT, text=True, encoding="utf-8",
                                capture_output=True, timeout=15, check=False)
        self.assertEqual(0, result.returncode, result.stderr + result.stdout)
        rows = [json.loads(line) for line in self.report.read_text().splitlines()][1:]
        commands = [row["command"] for row in rows if row["command"]]
        self.assertTrue(any(any(robot["velocity_world_mm_s"] != [0., 0.]
                                for robot in command["robots"]) for command in commands))
        positions = [row["observation"]["tracks"] for row in rows if row["observation"]]
        # The recorded image is stationary. Commands must not advance measured
        # poses as if this were a simulator, even with a nonzero target error.
        h1 = [next(track["robot_center_mm"] for track in tracks if track["robot_id"] == "H1")
              for tracks in positions]
        self.assertGreater(len(h1), 3)
        # MJPG compression may vary slightly by encoded frame. Check exact
        # equality to that frame's measured tag, not to a fictional fixed pose.
        for row in rows:
            if row["observation"]:
                observation = row["observation"]
                measured = {robot["robot_id"]: robot["robot_center_mm"] for robot in observation["robots"]}
                for track in observation["tracks"]:
                    self.assertEqual(measured[track["robot_id"]], track["robot_center_mm"])
        self.assertTrue(all(math.dist(point, h1[0]) < 2.0 for point in h1))

    def test_goal_file_rejects_wrong_units_unknown_robot_and_outside_body(self):
        goals = self.root / "bad-goals.json"
        base = {"schema_version": 1, "coordinate_system": "bottom_left_x_right_y_up_mm", "goals": {}}
        for change in ({"coordinate_system": "m"}, {"goals": []},
                       {"goals": {"unknown": {"x_mm": 300, "y_mm": 300}}},
                       {"goals": {"H1": {"x_mm": 5, "y_mm": 300}}}):
            with self.subTest(change=change):
                goals.write_text(json.dumps({**base, **change}))
                self.assertEqual(2, main([*self.arguments(), "--goals", str(goals)]))
                self.assertFalse(self.report.exists())

    def test_static_calibration_cannot_be_used_for_moving_camera(self):
        self.assertEqual(2, main([*self.arguments(), "--moving-camera"]))
        self.assertFalse(self.report.exists())


if __name__ == "__main__":
    unittest.main()
