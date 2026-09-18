"""Synthetic mathematical fixtures exercise an offline estimator, not hardware."""

from __future__ import annotations

import copy
import io
import json
import math
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from robo_control.hardware_calibration import estimate, main


def records(v=100., omega=0., heading=0., lateral=0.):
    result = []
    for index in range(7):
        elapsed, stamp = index * .05, 10. + index * .05
        angle = heading + omega * elapsed
        if omega:
            x = 500. + v / omega * (math.sin(angle) - math.sin(heading))
            y = 500. - v / omega * (math.cos(angle) - math.cos(heading))
        else:
            x = 500. + (v*math.cos(heading)-lateral*math.sin(heading))*elapsed
            y = 500. + (v*math.sin(heading)+lateral*math.cos(heading))*elapsed
        result.append({"status": "detected", "observation_usable": True, "observation_complete": True,
            "sequence": index+1, "captured_at_s": stamp, "processed_at_s": stamp+.01,
            "is_replay": False, "source_name": "webcam:0", "unknown_tag_ids": [], "duplicate_tag_ids": [],
            "coordinate_system": "bottom_left_x_right_y_up_mm", "field_size_mm": [1143., 1181.],
            "registered_robot_ids": ["H1"], "robots": [{"robot_id": "H1", "robot_center_mm": [x, y],
                                                     "heading_rad": (angle+math.pi)%(2*math.pi)-math.pi}]})
    return result


def run(rows, **changes):
    options = dict(robot_id="H1", track_width_mm=60., start_s=10., end_s=10.3,
                   left_pwm=30, right_pwm=30)
    options.update(changes)
    return estimate(rows, **options)


class HardwareCalibrationTests(unittest.TestCase):
    def test_cli_reads_jsonl_without_mutating_log_and_prints_candidate_only(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "observations.jsonl"
            data = "\n".join(json.dumps(row) for row in records()) + "\n"
            path.write_text(data, encoding="utf-8")
            before = path.read_bytes()
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                code = main(["--log", str(path), "--robot", "H1", "--track-width-mm", "60",
                             "--start-s", "10", "--end-s", "10.3", "--left-pwm", "30", "--right-pwm", "30"])
            result = json.loads(stdout.getvalue())
            self.assertEqual(0, code)
            self.assertFalse(result["device_io"])
            self.assertFalse(result["hardware_result"])
            self.assertEqual(before, path.read_bytes())
            self.assertEqual([path], list(Path(directory).iterdir()))

    def test_straight_forward_and_reverse_return_unsigned_curve_points_without_verification(self):
        for sign in (-1, 1):
            result = run(records(v=sign*100), left_pwm=sign*30, right_pwm=sign*30)
            suffix = "forward" if sign > 0 else "reverse"
            self.assertAlmostEqual(100., result["candidate_curve_points"]["left_"+suffix][1])
            self.assertAlmostEqual(100., result["candidate_curve_points"]["right_"+suffix][1])
            self.assertEqual(30, result["candidate_curve_points"]["left_"+suffix][0])
            self.assertFalse(result["hardware_result"])
            self.assertFalse(result["profile_updated"])
            self.assertFalse(result["motion_calibrated"])
            self.assertFalse(result["high_variability"])

    def test_rotation_unwraps_at_pi_and_arc_separates_left_right(self):
        result = run(records(v=100., omega=1., heading=3.05))
        self.assertAlmostEqual(1., result["statistics"]["yaw_rad_s"]["mean"], places=10)
        self.assertAlmostEqual(70., result["candidate_curve_points"]["left_forward"][1], delta=.02)
        self.assertAlmostEqual(130., result["candidate_curve_points"]["right_forward"][1], delta=.02)
        result = run(records(v=0., omega=1., heading=3.05), left_pwm=-30, right_pwm=30)
        self.assertAlmostEqual(30., result["candidate_curve_points"]["left_reverse"][1])

    def test_invalid_frames_outside_requested_interval_are_filtered_before_validation(self):
        rows = records()
        before = copy.deepcopy(rows[0])
        before.update(captured_at_s=9., is_replay=True, observation_usable=False, robots=[])
        after = copy.deepcopy(before)
        after["captured_at_s"] = 11.
        result = run([before, *rows, after])
        self.assertEqual(7, result["frames"])

    def test_duplicate_backward_gapped_nonfinite_or_missing_selected_measurements_rejected(self):
        mutations = [lambda rows: rows[2].update(sequence=2),
            lambda rows: rows[2].update(captured_at_s=rows[1]["captured_at_s"]),
            lambda rows: rows[2].update(captured_at_s=10.01),
            lambda rows: rows[2].update(robots=[]),
            lambda rows: rows[2]["robots"].append(copy.deepcopy(rows[2]["robots"][0])),
            lambda rows: rows[2]["robots"][0].update(heading_rad=float("nan")),
            lambda rows: rows[2]["robots"][0].update(robot_center_mm=[float("inf"), 500]),
            lambda rows: rows[2].update(observation_usable=False),
            lambda rows: rows[2].update(processed_at_s=10.8),
            lambda rows: rows[2].update(is_replay=True),
            lambda rows: rows[2].update(source_name="synthetic-closed-loop-session")]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                rows = records()
                mutation(rows)
                with self.assertRaises(ValueError):
                    run(rows)
        rows = records()
        for row in rows[3:]:
            row["captured_at_s"] += .21
            row["processed_at_s"] += .21
        with self.assertRaisesRegex(ValueError, "gap"):
            run(rows, end_s=10.6)

    def test_short_window_opposite_direction_and_lateral_slide_are_rejected(self):
        with self.assertRaises(ValueError):
            run(records(), end_s=10.1)
        with self.assertRaisesRegex(ValueError, "direction"):
            run(records(v=-100))
        with self.assertRaisesRegex(ValueError, "lateral"):
            run(records(lateral=40.))
        with self.assertRaises(ValueError):
            run(records(), left_pwm=0, right_pwm=0)

    def test_noisy_window_is_labelled_and_zero_pwm_side_is_not_calibrated(self):
        rows = records()
        rows[3]["robots"][0]["robot_center_mm"][0] += 8.
        result = run(rows, right_pwm=0)
        self.assertTrue(result["high_variability"])
        self.assertGreater(result["statistics"]["left_mm_s"]["stddev"], 0)
        self.assertEqual({"left_forward"}, set(result["candidate_curve_points"]))
        self.assertTrue(result["review_required"])

    def test_runtime_wrapper_keeps_observations_but_rejects_lost_observation_tick(self):
        rows = [{"event": "runtime_tick", "observation": row} for row in records()]
        result = run(rows)
        self.assertEqual(7, result["frames"])
        rows.insert(2, {"event": "runtime_tick", "at_s": 10.075, "observation": None, "fresh_streak": 0})
        with self.assertRaisesRegex(ValueError, "lost/invalid"):
            run(rows)


if __name__ == "__main__":
    unittest.main()
