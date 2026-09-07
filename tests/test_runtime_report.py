from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

from test_runtime_session import detection, make_session

from robo_control.runtime_report import inspect_report


class RuntimeReportTests(unittest.TestCase):
    def setUp(self):
        session = make_session()
        self.rows = [{"schema_version": 1, "event": "session_started",
            "session_id": session.controller.session_id, "roles": {"H1": "hamster"},
            "input_mode": "live_camera", "output_mode": "dry_run_commands",
            "device_io": False, "motion_permitted": False}]
        for seq in (1, 2, 3):
            now = 10. + seq * .02
            self.rows.append(session.advance(detection(seq, now), now + .005))
        self.rows.append(session.close(10.08, "operator_stop"))

    def inspect(self, rows=None, text=None):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "runtime.jsonl"
            path.write_text(text if text is not None else "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            return inspect_report(path)

    def test_reports_host_latency_counts_and_stop_without_claiming_physical_verification(self):
        result = self.inspect(self.rows)
        self.assertEqual(3, result["observation_frames"])
        self.assertEqual(3, result["usable_observation_frames"])
        self.assertAlmostEqual(5., result["mean_host_observation_age_ms"])
        self.assertAlmostEqual(20., result["max_tick_gap_ms"])
        self.assertEqual("operator_stop", result["closed_reason"])
        self.assertTrue(result["log_complete"])
        self.assertFalse(result["physical_stop_verified"])

    def test_truncated_or_unterminated_session_is_not_reported_complete(self):
        for rows in ([], self.rows[:1], self.rows[:-1]):
            with self.subTest(rows=len(rows)), self.assertRaises(ValueError):
                self.inspect(rows)
        text = "".join(json.dumps(row) + "\n" for row in self.rows)
        with self.assertRaisesRegex(ValueError, "incomplete"):
            self.inspect(text=text.rstrip("\n"))

    def test_mixed_session_time_rewind_and_repeated_tick_are_rejected(self):
        for change in ({"session_id": "another-session"}, {"at_s": 9.},
                       {"tick_sequence": 1}, {"input_mode": "video_replay"}, {"motion_permitted": 0}):
            rows = copy.deepcopy(self.rows)
            rows[2].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_missing_duplicate_or_nonzero_final_output_is_rejected(self):
        for mutate in (lambda row: row["actuator"]["robots"].clear(),
                       lambda row: row["actuator"]["robots"].append(copy.deepcopy(row["actuator"]["robots"][0])),
                       lambda row: row["actuator"]["robots"][0].update(velocity_world_mm_s=[1., 0.]),
                       lambda row: row["actuator"].update(device_io=True)):
            rows = copy.deepcopy(self.rows)
            mutate(rows[-1])
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_data_after_closed_and_nonfinite_json_are_rejected(self):
        with self.assertRaises(ValueError):
            self.inspect([*self.rows, self.rows[-1]])
        rows = copy.deepcopy(self.rows)
        rows[1]["at_s"] = float("nan")
        with self.assertRaisesRegex(ValueError, "Nonfinite"):
            self.inspect(rows)
        text = "".join(json.dumps(row) + "\n" for row in self.rows)
        with self.assertRaisesRegex(ValueError, "Nonfinite"):
            self.inspect(text=text.replace('"roles":', '"unrelated": 1e999, "roles":', 1))

    def test_closed_record_requires_a_reason(self):
        for reason in (None, "", 1):
            rows = copy.deepcopy(self.rows)
            rows[-1]["closed_reason"] = reason
            with self.subTest(reason=reason), self.assertRaises(ValueError):
                self.inspect(rows)


if __name__ == "__main__":
    unittest.main()
