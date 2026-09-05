from __future__ import annotations

import io
import json
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack, redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from robo_control.vision.__main__ import _pick_corners, main


@unittest.skipUnless(cv2 is not None and np is not None, "install the vision extra")
class VisionCliTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.rectangle(self.image, (20, 20), (300, 220), (255, 255, 255), 2)
        cv2.circle(self.image, (90, 170), 10, (0, 0, 255), -1)
        cv2.circle(self.image, (230, 60), 10, (0, 255, 0), -1)
        # imencode + bytes also tests the Windows non-ASCII image path.
        self.still = self.root / "경기장.png"
        self.still.write_bytes(cv2.imencode(".png", self.image)[1].tobytes())
        self.calibration = self.root / "camera.json"

    def invoke(self, *args):
        output, errors = io.StringIO(), io.StringIO()
        with redirect_stdout(output), redirect_stderr(errors):
            status = main([str(arg) for arg in args])
        return status, output.getvalue(), errors.getvalue()

    def calibrate(self):
        status, output, errors = self.invoke(
            "calibrate", "--image", self.still, "--corners",
            20, 20, 300, 20, 300, 220, 20, 220,
            "--field-size-mm", 280, 200, "--output", self.calibration,
            "--preview-output", self.root / "보정.png",
        )
        self.assertEqual(0, status, errors)
        return json.loads(output)

    def write_video(self, name="frames.avi", count=6):
        path = self.root / name
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30.0, (320, 240))
        self.assertTrue(writer.isOpened(), "MJPG codec required for real file IO integration test")
        try:
            for _ in range(count):
                writer.write(self.image)
        finally:
            writer.release()
        return path

    def test_calibrate_saves_geometry_and_preserves_source(self):
        original = self.still.read_bytes()
        result = self.calibrate()
        self.assertEqual("calibrated", result["status"])
        self.assertFalse(result["physical_accuracy_verified"])
        self.assertEqual([320, 240], result["calibration"]["image_size_px"])
        self.assertEqual(original, self.still.read_bytes())
        preview = cv2.imdecode(np.frombuffer((self.root / "보정.png").read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual((201, 281, 3), preview.shape)
        np.testing.assert_array_equal(preview[150, 70], (0, 0, 255))

    def test_actual_video_decode_rectification_jsonl_and_eof(self):
        self.calibrate()
        video = self.write_video()
        report, image = self.root / "frames.jsonl", self.root / "last.png"
        status, output, errors = self.invoke(
            "run", "--video", video, "--calibration", self.calibration,
            "--frames", 10, "--max-age-ms", 2000, "--report", report, "--output", image,
        )
        self.assertEqual(0, status, errors)
        result = json.loads(output)
        self.assertEqual("eof_or_decode_failure", result["status"])
        self.assertEqual(6, result["accepted_frames"])
        self.assertEqual(0, result["rejected_frames"])
        self.assertTrue(result["is_replay"])
        records = [json.loads(line) for line in report.read_text().splitlines()]
        self.assertEqual(list(range(1, 7)), [record["sequence"] for record in records])
        for record in records:
            self.assertEqual("host_read_start", record["timestamp_basis"])
            self.assertEqual("accepted", record["status"])
            self.assertTrue(record["is_replay"])
            self.assertLessEqual(record["captured_at_s"], record["received_at_s"])
            self.assertLessEqual(record["received_at_s"], record["processed_at_s"])
        self.assertTrue(image.is_file())

    def test_video_frame_limit_and_module_entrypoint(self):
        self.calibrate()
        video = self.write_video()
        completed = subprocess.run(
            [sys.executable, "-m", "robo_control.vision", "run", "--video", str(video),
             "--calibration", str(self.calibration), "--frames", "2", "--max-age-ms", "2000"],
            capture_output=True, text=True, timeout=30, check=False,
        )
        self.assertEqual(0, completed.returncode, completed.stderr)
        result = json.loads(completed.stdout)
        self.assertEqual("frame_limit", result["status"])
        self.assertEqual(2, result["accepted_frames"])

    def test_independent_landmark_check_passes_and_fails(self):
        self.calibrate()
        path = self.root / "landmarks.json"
        path.write_text(json.dumps({"pixel_points": [[90, 170], [230, 60]],
                                    "expected_mm": [[70, 50], [210, 160]]}))
        status, output, errors = self.invoke("check", "--calibration", self.calibration, "--points", path)
        self.assertEqual(0, status, errors)
        self.assertTrue(json.loads(output)["passed"])
        path.write_text(json.dumps({"pixel_points": [[90, 170]], "expected_mm": [[100, 50]]}))
        status, output, _ = self.invoke("check", "--calibration", self.calibration, "--points", path)
        self.assertEqual(1, status)
        self.assertAlmostEqual(30, json.loads(output)["max_error_mm"], places=4)

    def test_resolution_mismatch_is_reported_without_output_image(self):
        self.calibrate()
        raw = json.loads(self.calibration.read_text())
        raw["image_size_px"] = [321, 240]
        self.calibration.write_text(json.dumps(raw))
        video = self.write_video(count=1)
        output_image, report = self.root / "invalid.png", self.root / "invalid.jsonl"
        status, output, _ = self.invoke(
            "run", "--video", video, "--calibration", self.calibration,
            "--output", output_image, "--report", report, "--max-age-ms", 2000,
        )
        self.assertEqual(1, status)
        self.assertEqual(1, json.loads(output)["rejected_frames"])
        self.assertIn("resolution", report.read_text())
        self.assertFalse(output_image.exists())

    def test_missing_input_returns_clear_error(self):
        self.calibrate()
        status, _, errors = self.invoke(
            "run", "--video", self.root / "missing.avi", "--calibration", self.calibration,
        )
        self.assertEqual(2, status)
        self.assertIn("vision:", errors)

    def test_output_cannot_replace_input_image(self):
        original = self.still.read_bytes()
        status, _, errors = self.invoke(
            "calibrate", "--image", self.still, "--output", self.still,
            "--corners", 20, 20, 300, 20, 300, 220, 20, 220,
        )
        self.assertEqual(2, status)
        self.assertIn("distinct", errors)
        self.assertEqual(original, self.still.read_bytes())

    def test_invalid_corners_do_not_create_calibration(self):
        status, _, _ = self.invoke(
            "calibrate", "--image", self.still, "--output", self.calibration,
            "--corners", 20, 20, 300, 220, 300, 20, 20, 220,
        )
        self.assertEqual(2, status)
        self.assertFalse(self.calibration.exists())

    def test_large_calibration_image_clicks_map_back_to_original_pixels(self):
        image = np.zeros((2160, 3840, 3), dtype=np.uint8)
        clicks = ((100, 100), (900, 100), (900, 500), (100, 500))

        def register_callback(window, callback):
            for x, y in clicks:
                callback(cv2.EVENT_LBUTTONDOWN, x, y, 0, None)

        with ExitStack() as stack:
            for name in ("namedWindow", "destroyWindow"):
                stack.enter_context(patch.object(cv2, name))
            shown = stack.enter_context(patch.object(cv2, "imshow"))
            stack.enter_context(patch.object(cv2, "setMouseCallback", side_effect=register_callback))
            stack.enter_context(patch.object(cv2, "waitKey", return_value=13))
            actual = _pick_corners(image)
        displayed = shown.call_args.args[1]
        height, width = displayed.shape[:2]
        self.assertLessEqual(width, 1000)
        self.assertLessEqual(height, 650)
        expected = [((x + 0.5) * 3840 / width - 0.5,
                     (y + 0.5) * 2160 / height - 0.5) for x, y in clicks]
        np.testing.assert_allclose(actual, expected, atol=1e-10)

    def test_closing_preview_window_stops_after_one_frame(self):
        self.calibrate()
        video = self.write_video()
        with ExitStack() as stack:
            for name in ("namedWindow", "resizeWindow", "imshow", "destroyAllWindows"):
                stack.enter_context(patch.object(cv2, name))
            stack.enter_context(patch.object(cv2, "waitKey", return_value=-1))
            stack.enter_context(patch.object(cv2, "getWindowProperty", return_value=-1))
            status, output, errors = self.invoke(
                "run", "--video", video, "--calibration", self.calibration,
                "--preview", "--max-age-ms", 2000,
            )
        self.assertEqual(0, status, errors)
        self.assertEqual("user_stopped", json.loads(output)["status"])
        self.assertEqual(1, json.loads(output)["accepted_frames"])


if __name__ == "__main__":
    unittest.main()
