from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from robo_control.adapters import CameraFrame
from robo_control.vision.calibration import CalibrationError
from robo_control.vision.lens_capture import ChessboardCapture


@unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
class ChessboardCaptureTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name) / "new-capture"
        self.capture = ChessboardCapture(self.root, source_name="webcam:0", camera_label="test fixture",
                                         inner_corners=(7, 5), max_age_s=.2)
        self.addCleanup(self.capture.close)
        self.image = np.full((480, 640, 3), (5, 100, 180), np.uint8)
        self.points = np.array([(100 + x * 40, 100 + y * 40) for y in range(5) for x in range(7)], float)

    def frame(self, sequence=1, stamp=10., **kwargs):
        args = {"image": self.image, "sequence": sequence, "captured_at_s": stamp,
                "received_at_s": stamp + .005, "source_name": "webcam:0"}
        args.update(kwargs)
        return CameraFrame(**args)

    def test_saved_photo_is_exact_raw_pixels_not_the_resized_or_annotated_preview(self):
        result = self.capture.save_frame(self.frame(), self.points, 10.02)
        self.assertEqual("image_saved", result["event"])
        decoded = cv2.imdecode(np.frombuffer((self.root / result["file"]).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(decoded, self.image)
        self.capture.close()
        rows = [json.loads(line) for line in (self.root / "capture.jsonl").read_text().splitlines()]
        self.assertEqual(["capture_started", "image_saved", "capture_closed"], [row["event"] for row in rows])
        self.assertEqual(self.points.tolist(), rows[1]["corners_raw_px"])
        self.assertFalse(rows[-1]["physical_accuracy_verified"])

    def test_duplicate_pose_and_repeated_frame_do_not_inflate_dataset(self):
        self.capture.save_frame(self.frame(), self.points, 10.02)
        self.assertEqual("duplicate_board_pose", self.capture.save_frame(self.frame(2, 10.1), self.points + .2, 10.12)["reason"])
        self.assertEqual("repeated_or_out_of_order_frame", self.capture.save_frame(self.frame(), self.points + 10, 10.02)["reason"])
        self.assertEqual(1, len(list(self.root.glob("*.png"))))

    def test_stale_missing_future_and_tiny_board_are_not_saved(self):
        cases = [(self.frame(1), self.points, 10.3), (self.frame(2), None, 10.02),
                 (self.frame(3, 11), self.points, 10.02), (self.frame(4), self.points * .01, 10.02)]
        for frame, points, now in cases:
            with self.subTest(sequence=frame.sequence):
                self.assertEqual("save_rejected", self.capture.save_frame(frame, points, now)["event"])
        self.assertEqual([], list(self.root.glob("*.png")))

    def test_resolution_source_and_replay_changes_close_the_collection(self):
        self.capture.save_frame(self.frame(), self.points, 10.02)
        with self.assertRaises(CalibrationError):
            self.capture.save_frame(self.frame(2, 10.1, image=np.zeros((240, 320, 3), np.uint8)), self.points, 10.12)
        self.assertTrue(self.capture.closed)
        with self.assertRaises(CalibrationError):
            self.capture.save_frame(self.frame(3, 10.2), self.points, 10.22)
        for changes in ({"source_name": "webcam:1"}, {"is_replay": True}):
            target = Path(self.folder.name) / ("other-source" if "source_name" in changes else "replay")
            c = ChessboardCapture(target, source_name="webcam:0", camera_label="fixture", inner_corners=(7, 5))
            with self.subTest(changes=changes), self.assertRaises(CalibrationError):
                c.save_frame(self.frame(**changes), self.points, 10.02)
            self.assertTrue(c.closed)

    def test_existing_folder_is_not_overwritten_and_early_close_preserves_photos(self):
        self.capture.save_frame(self.frame(), self.points, 10.02)
        self.capture.close("operator_finished")
        with self.assertRaises(FileExistsError):
            ChessboardCapture(self.root, source_name="webcam:0", camera_label="fixture", inner_corners=(7, 5))
        self.assertTrue((self.root / "board-001.png").is_file())


if __name__ == "__main__":
    unittest.main()
