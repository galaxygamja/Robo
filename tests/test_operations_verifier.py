from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.verify_mission_operations import ROOT, main, run_scenario, write_json

try:
    import cv2
except ImportError:
    cv2 = None


class OperationsVerifierTests(unittest.TestCase):
    def test_rejects_unbounded_clip_lengths_before_creating_output(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)/"new"
            for value in ("nan", "inf", "0", "5.9", "20.1"):
                with self.subTest(value=value), patch("sys.stderr"), self.assertRaises(SystemExit) as result:
                    main(["--output-dir", str(target), "--seconds", value])
                self.assertEqual(2, result.exception.code)
                self.assertFalse(target.exists())

    def test_existing_evidence_directory_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory)/"verification.json"
            marker.write_text("preserve evidence", encoding="utf-8")
            with patch("sys.stderr"):
                self.assertEqual(1, main(["--output-dir", directory]))
            self.assertEqual("preserve evidence", marker.read_text(encoding="utf-8"))

    def test_rejects_invalid_observation_soak_duration_without_creating_output(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)/"new"
            for value in ("nan", "inf", "-1", "4.9", "600.1"):
                with self.subTest(value=value), patch("sys.stderr"), self.assertRaises(SystemExit):
                    main(["--output-dir", str(target), "--observe-only-soak-s", value])
                self.assertFalse(target.exists())

    def test_json_evidence_writer_never_overwrites(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"result.json"
            write_json(path, {"original": True})
            with self.assertRaises(FileExistsError):
                write_json(path, {"original": False})
            self.assertEqual({"original": True}, json.loads(path.read_text(encoding="utf-8")))

    def test_arbitrary_safety_error_does_not_count_as_expected_stop(self):
        for scenario, status, code in (("steady", "video_eof", 1), ("obstacle_lost", "route_unavailable", 1),
                                       ("target_drift", "pickup_anchor_drift_exceeded", 0)):
            with self.subTest(scenario=scenario), tempfile.TemporaryDirectory() as directory:
                process = subprocess.CompletedProcess([], code, json.dumps({"status": status}), "")
                with (patch("tools.verify_mission_operations.subprocess.run", return_value=process),
                      self.assertRaisesRegex(AssertionError, "expected exit")):
                    run_scenario(Path(directory), scenario, 6.)

    def test_runtime_startup_error_preserves_stderr_diagnostic(self):
        with tempfile.TemporaryDirectory() as directory:
            process = subprocess.CompletedProcess([], 2, "", "fixture rejected before camera launch")
            with (patch("tools.verify_mission_operations.subprocess.run", return_value=process),
                  self.assertRaisesRegex(AssertionError, "fixture rejected")):
                run_scenario(Path(directory), "steady", 6.)
            saved = json.loads((Path(directory)/"steady-process.json").read_text(encoding="utf-8"))
            self.assertEqual(process.stderr, saved["stderr"])

    @unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
    def test_real_video_cli_runs_all_three_scenarios_and_writes_honest_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)/"generated"
            process = subprocess.run([sys.executable, str(ROOT/"tools/verify_mission_operations.py"),
                "--output-dir", str(target), "--seconds", "6", "--observe-only-soak-s", "5"], cwd=ROOT,
                text=True, encoding="utf-8", capture_output=True, timeout=45, check=False)
            self.assertEqual(0, process.returncode, process.stdout+process.stderr)
            evidence = json.loads((target/"verification.json").read_text(encoding="utf-8"))
            self.assertEqual("passed", evidence["status"])
            self.assertFalse(evidence["physical_robot_used"])
            self.assertFalse(evidence["physical_stop_verified"])
            self.assertEqual(3, len(evidence["scenarios"]))
            self.assertEqual(3, len({s["checks"]["session_id"] for s in evidence["scenarios"]}))
            soak = evidence["observation_soak"]
            self.assertEqual(5., soak["requested_duration_s"])
            self.assertGreater(soak["ready_obstacle_ticks"], 0)
            self.assertFalse(soak["movement_authorized"])
            for scenario in evidence["scenarios"]:
                self.assertGreater(scenario["checks"]["fake_motion_ticks"], 0)
                self.assertTrue(scenario["report_analysis"]["final_wire_stop_acknowledged"])
                self.assertEqual([], scenario["report_analysis"]["mission"]["completed_tasks"])


if __name__ == "__main__":
    unittest.main()
