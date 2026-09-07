from __future__ import annotations

import io
import json
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from test_lens_calibration import board_dataset, known_lens, reference_distortion

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from robo_control.vision.__main__ import main as vision_main
from robo_control.vision.calibration import CalibrationError, FieldCalibration
from robo_control.vision.lens import LensCalibration
from robo_control.vision.lens_calibrate import calibrate_images
from robo_control.vision.lens_calibrate import main as lens_main
from robo_control.vision.tags import TagDetectorConfig


def write_image(path, image):
    path.write_bytes(cv2.imencode(".png", image)[1].tobytes())


@unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
class LensImageIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.folder.cleanup)
        cls.root = Path(cls.folder.name)
        _, _, images = board_dataset(render=True)
        cls.photos = cls.root / "체커보드"
        cls.photos.mkdir()
        cls.paths = []
        for i, image in enumerate(images):
            path = cls.photos / f"board-{i:02}.png"
            write_image(path, image)
            cls.paths.append(path)

    def test_real_image_decode_corner_detection_fit_holdout_and_cli_save(self):
        output, report = self.root / "fitted-lens.json", self.root / "fit-report.json"
        stream = io.StringIO()
        with redirect_stdout(stream):
            result = lens_main(["--images", str(self.photos), "--inner-corners", "7", "5",
                "--square-size-mm", "24", "--camera-label", "generated-raster-board-fixture",
                "--output", str(output), "--report", str(report)])
        self.assertEqual(0, result, stream.getvalue())
        profile = LensCalibration.load(output)
        data = json.loads(report.read_text())
        self.assertEqual(24, len(data["images"]))
        self.assertEqual(0, data["undetected_images"])
        self.assertEqual(4, len(data["held_out_view_indices"]))
        self.assertLess(max(data["per_view_rms_px"]), 1.)
        self.assertEqual(profile.fingerprint, data["lens_calibration_id"])
        self.assertFalse(data["physical_accuracy_verified"])
        before = output.read_bytes()
        with redirect_stderr(io.StringIO()):
            self.assertEqual(2, lens_main(["--images", str(self.photos), "--inner-corners", "7", "5",
                "--square-size-mm", "24", "--camera-label", "fixture", "--output", str(output),
                "--report", str(self.root / "another.json")]))
        self.assertEqual(before, output.read_bytes())

    def test_duplicate_image_content_and_mixed_resolutions_are_rejected(self):
        for paths in ([self.paths[0]] * 12, [*self.paths[:11], self.root / "resized.png"]):
            if paths[-1].name == "resized.png":
                write_image(paths[-1], np.zeros((240, 320), np.uint8))
            with self.subTest(paths=[p.name for p in paths]), self.assertRaises(CalibrationError):
                calibrate_images(paths, (7, 5), 24., "fixture")


@unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
class LensRuntimeIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.lens = known_lens()
        self.lens_path = self.root / "lens.json"
        self.lens.save(self.lens_path)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        ideal = np.full((480, 640), 255, np.uint8)
        self.centres_ideal = {}
        for tag, (x, y) in enumerate(((100, 80), (440, 80), (100, 340), (440, 340))):
            ideal[y:y + 72, x:x + 72] = cv2.aruco.generateImageMarker(dictionary, tag, 72)
            self.centres_ideal[('H1', 'H2', 'B1', 'B2')[tag]] = (x + 35.5, y + 35.5)
        # A RAW output pixel samples its undistorted location in the ideal
        # fixture. This produces a nonlinear image, not only shifted tag boxes.
        x, y = np.meshgrid(np.arange(640), np.arange(480))
        points = self.lens.undistort_points(np.column_stack((x.ravel(), y.ravel()))).reshape(480, 640, 2)
        gray = cv2.remap(ideal, points[:, :, 0].astype(np.float32), points[:, :, 1].astype(np.float32),
                         cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=255)
        self.image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        self.still = self.root / "왜곡된-태그.png"
        write_image(self.still, self.image)
        self.corners = reference_distortion(((60, 40), (580, 40), (580, 440), (60, 440)), self.lens)
        self.calibration = self.root / "field.json"
        FieldCalibration((640, 480), self.corners, (1143, 1181), self.lens).save(self.calibration)
        self.tags = self.root / "tags.json"
        TagDetectorConfig(tag_to_robot={0: "H1", 1: "H2", 2: "B1", 3: "B2"}).save(self.tags)
        self.fleet = self.root / "fleet.json"
        self.fleet.write_text(json.dumps({"ground_robots": [
            {"id": rid, "tag_id": tag, "role": "hamster" if rid == "H1" else "beaver"}
            for tag, rid in enumerate(("H1", "H2", "B1", "B2"))]}))

    def test_field_calibration_cli_embeds_lens_and_check_uses_raw_landmarks(self):
        output = self.root / "cli-field.json"
        stream = io.StringIO()
        with redirect_stdout(stream):
            result = vision_main(["calibrate", "--image", str(self.still), "--lens", str(self.lens_path),
                "--corners", *[str(v) for v in self.corners.ravel()], "--output", str(output)])
        self.assertEqual(0, result)
        self.assertEqual(self.lens, FieldCalibration.load(output).lens)
        self.assertEqual(2, json.loads(stream.getvalue())["calibration"]["schema_version"])
        expected = np.array(((200, 300), (900, 1000)), float)
        ideal = np.column_stack((60 + expected[:, 0] * 520 / 1143, 440 - expected[:, 1] * 400 / 1181))
        points = self.root / "landmarks.json"
        points.write_text(json.dumps({"pixel_points": reference_distortion(ideal, self.lens).tolist(),
                                     "expected_mm": expected.tolist()}))
        with redirect_stdout(io.StringIO()):
            self.assertEqual(0, vision_main(["check", "--calibration", str(output), "--points", str(points),
                                            "--max-error-mm", ".01"]))

    def test_distorted_video_spawns_with_embedded_lens_and_measures_all_four_robots(self):
        video, report = self.root / "distorted.avi", self.root / "runtime.jsonl"
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 20., (640, 480))
        self.assertTrue(writer.isOpened())
        for _ in range(16):
            writer.write(self.image)
        writer.release()
        args = [sys.executable, "-m", "robo_control.runtime", "--video", str(video),
            "--calibration", str(self.calibration), "--tags", str(self.tags), "--fleet", str(self.fleet),
            "--report", str(report), "--duration-s", "8", "--startup-timeout-s", "5"]
        result = subprocess.run(args, capture_output=True, text=True, timeout=15, check=False)
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        summary = json.loads(result.stdout)
        self.assertGreater(summary["usable_observation_frames"], 3)
        rows = [json.loads(line) for line in report.read_text().splitlines()]
        self.assertEqual(2, rows[0]["calibration"]["schema_version"])
        self.assertEqual(self.lens.as_dict(), rows[0]["calibration"]["lens"])
        for row in rows[1:]:
            if not row["observation"]:
                continue
            self.assertTrue(row["observation"]["lens_correction_applied"])
            self.assertEqual(self.lens.fingerprint, row["observation"]["lens_calibration_id"])
            for robot in row["observation"]["robots"]:
                x, y = self.centres_ideal[robot["robot_id"]]
                expected = ((x - 60) * 1143 / 520, (440 - y) * 1181 / 400)
                np.testing.assert_allclose(robot["robot_center_mm"], expected, atol=2.)
        self.assertEqual("closed", rows[-1]["status"])
        self.assertTrue(all(robot["velocity_world_mm_s"] == [0., 0.] for robot in rows[-1]["actuator"]["robots"]))
        self.assertFalse(summary["motion_permitted"])

    def test_encoded_lens_color_fixture_confirms_objects_and_keeps_stop_invariants(self):
        tool = Path(__file__).resolve().parents[1] / "tools" / "verify_camera_runtime.py"
        output = self.root / "lens-color-audit"
        result = subprocess.run([sys.executable, str(tool), "--output-dir", str(output),
            "--seconds", "3", "--lens-fixture", "--colors-fixture"],
            capture_output=True, text=True, timeout=30, check=False)
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        evidence = json.loads((output / "verification.json").read_text())
        checks = evidence["invariant_checks"]
        self.assertGreater(checks["confirmed_object_ticks"], 0)
        self.assertTrue(checks["all_motion_fresh_and_confirmed"])
        self.assertTrue(checks["no_synthetic_pose_replacement"])
        self.assertFalse(evidence["physical_camera_used"])
        self.assertFalse(evidence["physical_robot_used"])

    def test_audit_accepts_rejected_frame_only_with_stopped_output(self):
        from tools.verify_camera_runtime import check_observation
        record = {"status": "rejected_frame", "reason": "stale_frame", "observation_usable": False,
                  "lens_correction_applied": True, "lens_calibration_id": self.lens.fingerprint}
        check_observation(record, self.lens, moving=False)
        with self.assertRaises(AssertionError):
            check_observation(record, self.lens, moving=True)
        with self.assertRaises(AssertionError):
            check_observation({**record, "observation_usable": True}, self.lens, moving=False)


if __name__ == "__main__":
    unittest.main()
