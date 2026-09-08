"""Actual pixel/runtime composition plus adversarial checks of the evidence oracle."""
from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.verify_compound_faults import CASES, check_trace, main, robot_wire, run_case
from tools.verify_mission_operations import prepare_fixture

try:
    import cv2
except ImportError:
    cv2 = None


class CompoundBoundaryTests(unittest.TestCase):
    def test_existing_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "verification.json"
            path.write_text("keep", encoding="utf-8")
            with patch("sys.stderr"):
                self.assertEqual(1, main(["--output-dir", folder]))
            self.assertEqual("keep", path.read_text(encoding="utf-8"))

    def test_bad_delay_rejected_before_output_creation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for delay in (True, -1, .079, .161, float("nan"), float("inf")):
                with self.subTest(delay=delay), self.assertRaises(AssertionError):
                    run_case(root, "occlusion_request_delay", delay_s=delay)
            self.assertEqual([], list(root.iterdir()))

    def test_unknown_case_cannot_be_silently_accepted(self):
        with tempfile.TemporaryDirectory() as folder, self.assertRaises(KeyError):
            run_case(Path(folder), "anything_is_a_pass")

    def test_manifest_contains_both_verifiers(self):
        root = Path(__file__).resolve().parents[1]
        manifest = (root / "MANIFEST.in").read_text(encoding="utf-8")
        self.assertIn("include tools/verify_compound_faults.py", manifest)
        self.assertIn("include tools/verify_mission_operations.py", manifest)


@unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
class CompoundPixelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.root = Path(cls.temporary.name)
        prepare_fixture(cls.root, 6.)
        cls.results = {name: run_case(cls.root, name) for name in CASES}
        cls.traces = {name: [json.loads(line) for line in
            (cls.root / name / "shutdown-trace.jsonl").read_text(encoding="utf-8").splitlines()] for name in CASES}

    def test_all_eight_exact_outcomes_and_fresh_sessions(self):
        self.assertEqual(8, len(self.results))
        self.assertEqual(8, len({r["report_analysis"]["session_id"] for r in self.results.values()}))
        for name, result in self.results.items():
            self.assertEqual("passed", result["status"])
            self.assertEqual(CASES[name][2], result["report_analysis"]["closed_reason"])

    def test_pending_request_and_pending_ack_are_both_exercised(self):
        for name in ("occlusion_request_delay", "occlusion_ack_delay"):
            rows = self.traces[name]
            warm, queued = rows[8], rows[9]
            self.assertEqual("drive", robot_wire(queued)["sender"]["pending_request"])
            difference = (robot_wire(queued)["receiver"]["accepted_drive_count"]
                          - robot_wire(warm)["receiver"]["accepted_drive_count"])
            self.assertEqual(0 if "request" in name else 1, difference)

    def test_returned_pixels_do_not_reopen_closed_mission(self):
        for name in CASES:
            if name == "multi_clear":
                continue
            rows = self.traces[name]
            for row in rows[20:]:
                self.assertEqual(CASES[name][2], row["closed_reason"])
                self.assertEqual("closed", row["mission"]["status"])
                self.assertFalse(row["wire"]["ready"])
        path = self.root / "occlusion_request_delay" / "detections.jsonl"
        detections = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(2, len(detections[9]["objects"]))
        self.assertEqual(3, len(detections[19]["objects"]))

    def test_missing_stop_ack_is_unknown_even_after_modeled_watchdog_zero(self):
        result = self.results["occlusion_stop_ack_loss"]
        self.assertEqual(["H1"], result["checks"]["unconfirmed_stop_robot_ids"])
        final = self.traces["occlusion_stop_ack_loss"][-1]
        self.assertEqual(0, robot_wire(final)["receiver"]["v_mm_s"])
        self.assertFalse(robot_wire(final)["stop_acknowledged"])
        self.assertFalse(result["checks"]["physical_stop_verified"])

    def test_normal_report_ends_at_first_closure_without_laundered_acks(self):
        path = self.root / "occlusion_request_delay" / "runtime.jsonl"
        report = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(1, sum(r.get("closed_reason") is not None for r in report))
        self.assertIn("H1", report[-1]["wire"]["unconfirmed_stop_robot_ids"])
        self.assertEqual([], self.traces["occlusion_request_delay"][-1]["wire"]["unconfirmed_stop_robot_ids"])

    def test_inputs_and_injection_evidence_are_saved(self):
        for name in CASES:
            directory = self.root / name
            self.assertTrue((directory / "frame-008.png").is_file())
            evidence = json.loads((directory / "injections.json").read_text(encoding="utf-8"))
            self.assertEqual("deterministic_not_realtime", evidence["clock_mode"])
            self.assertTrue(evidence["synthetic"])

    def modified(self):
        rows = copy.deepcopy(self.traces["occlusion_request_delay"])
        first = next(r for r in rows[1:] if r["closed_reason"] is not None)
        return rows, first

    def rejected(self, rows, message):
        with self.assertRaisesRegex(AssertionError, message):
            check_trace(rows, case="occlusion", expected_reason="object_obstacle_lost", lost_stop_ack=False)

    def test_oracle_rejects_arbitrary_stop(self):
        rows, _ = self.modified()
        rows[-1]["closed_reason"] = "route_unavailable"
        self.rejected(rows, "Wrong final reason")

    def test_oracle_requires_pending_drive(self):
        rows, _ = self.modified()
        robot_wire(rows[9])["sender"]["pending_request"] = None
        self.rejected(rows, "No drive waiting")

    def test_oracle_rejects_drive_on_fault_frame(self):
        rows, first = self.modified()
        robot_wire(first)["receiver"]["accepted_drive_count"] += 1
        self.rejected(rows, "Drive executed")

    def test_oracle_rejects_drive_after_fault(self):
        rows, _ = self.modified()
        robot_wire(rows[-1])["receiver"]["accepted_drive_count"] += 1
        self.rejected(rows, "Drive executed")

    def test_oracle_rejects_silent_obstacle_removal(self):
        rows, first = self.modified()
        first["object_obstacles"]["objects"] = []
        self.rejected(rows, "history was cleared")

    def test_oracle_rejects_target_id_rebinding(self):
        rows, first = self.modified()
        first["bindings"]["mappings"]["D1"] = "O9999"
        self.rejected(rows, "Target silently rebound")

    def test_oracle_rejects_physical_success_claim(self):
        rows, first = self.modified()
        first["mission"]["physical_mission_verified"] = True
        self.rejected(rows, "Invented mission")

    def test_oracle_rejects_missing_fleet_stop(self):
        rows, first = self.modified()
        robot_wire(first, "B2")["stop_requested"] = False
        self.rejected(rows, "Fleet stop")

    def test_oracle_rejects_nonzero_post_stop_output(self):
        rows, first = self.modified()
        first["actuator"]["robots"][0]["forward_velocity_mm_s"] = 1.
        self.rejected(rows, "Nonzero local output")

    def test_oracle_rejects_false_stop_ack_certainty(self):
        rows = copy.deepcopy(self.traces["occlusion_stop_ack_loss"])
        rows[-1]["wire"]["unconfirmed_stop_robot_ids"] = []
        with self.assertRaisesRegex(AssertionError, "ACK certainty"):
            check_trace(rows, case="occlusion", expected_reason="object_obstacle_lost", lost_stop_ack=True)


if __name__ == "__main__":
    unittest.main()
