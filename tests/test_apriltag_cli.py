from __future__ import annotations

import io
import json
import math
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack, redirect_stderr, redirect_stdout
from pathlib import Path
from typing import ClassVar
from unittest.mock import patch

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from robo_control.vision.__main__ import main
from robo_control.vision.calibration import FieldCalibration

HAS_APRILTAG = (
    cv2 is not None
    and np is not None
    and hasattr(cv2, "aruco")
    and hasattr(cv2.aruco, "DICT_APRILTAG_36h11")
    and hasattr(cv2.aruco, "generateImageMarker")
)


@unittest.skipUnless(HAS_APRILTAG, "install the vision extra with AprilTag-enabled OpenCV")
class AprilTagCliIntegrationTests(unittest.TestCase):
    WIDTH = 640
    HEIGHT = 480
    TAG_PX = 84
    ROBOTS: ClassVar = {0: "H1", 1: "H2", 2: "B1", 3: "B2"}
    PLACEMENTS: ClassVar = {
        0: (64, 56),
        1: (476, 56),
        2: (476, 332),
        3: (64, 332),
    }

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.calibration = self.root / "camera.json"
        FieldCalibration(
            (self.WIDTH, self.HEIGHT),
            ((0, 0), (self.WIDTH - 1, 0),
             (self.WIDTH - 1, self.HEIGHT - 1), (0, self.HEIGHT - 1)),
            (self.WIDTH - 1, self.HEIGHT - 1),
        ).save(self.calibration)
        self.tags = self.root / "robot-tags.json"
        self.write_tag_config()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    def write_tag_config(self, **updates):
        config = {
            "schema_version": 1,
            "dictionary_name": "DICT_APRILTAG_36h11",
            # The identity-like calibration makes one image pixel one mm.
            "tag_size_mm": float(self.TAG_PX - 1),
            "tag_to_robot": {str(tag_id): robot_id for tag_id, robot_id in self.ROBOTS.items()},
        }
        config.update(updates)
        self.tags.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")

    def render(self, tags=()):
        image = np.full((self.HEIGHT, self.WIDTH, 3), 255, dtype=np.uint8)
        for tag_id, (x, y) in tags:
            marker = cv2.aruco.generateImageMarker(self.dictionary, tag_id, self.TAG_PX)
            marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
            image[y:y + self.TAG_PX, x:x + self.TAG_PX] = marker
        return image

    def write_image(self, image, name="태그-경기장.png"):
        path = self.root / name
        ok, encoded = cv2.imencode(".png", image)
        self.assertTrue(ok)
        path.write_bytes(encoded.tobytes())
        return path

    def write_video(self, frames, name="태그-프레임.avi"):
        path = self.root / name
        writer = cv2.VideoWriter(
            str(path), cv2.VideoWriter_fourcc(*"MJPG"), 20.0, (self.WIDTH, self.HEIGHT)
        )
        self.assertTrue(writer.isOpened(), "MJPG codec required for real file IO integration test")
        try:
            for frame in frames:
                writer.write(frame)
        finally:
            writer.release()
        return path

    def invoke(self, *args):
        output, errors = io.StringIO(), io.StringIO()
        with redirect_stdout(output), redirect_stderr(errors):
            status = main([str(arg) for arg in args])
        return status, output.getvalue(), errors.getvalue()

    @staticmethod
    def read_records(path):
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]

    def assert_four_robot_record(self, record):
        self.assertEqual("DICT_APRILTAG_36h11", record["dictionary_name"])
        self.assertEqual(float(self.TAG_PX - 1), record["tag_size_mm"])
        self.assertFalse(record["hardware_verified"])
        self.assertTrue(record["observation_complete"])
        self.assertEqual([], record["unknown_tag_ids"])
        self.assertEqual([], record["duplicate_tag_ids"])
        self.assertEqual([], record["missing_robot_ids"])
        robots = {robot["robot_id"]: robot for robot in record["robots"]}
        self.assertEqual(set(self.ROBOTS.values()), set(robots))
        for tag_id, robot_id in self.ROBOTS.items():
            robot = robots[robot_id]
            self.assertEqual(tag_id, robot["tag_id"])
            self.assertEqual(2, len(robot["tag_center_px"]))
            self.assertEqual(2, len(robot["tag_center_mm"]))
            self.assertEqual(2, len(robot["robot_center_px"]))
            self.assertEqual(2, len(robot["robot_center_mm"]))
            self.assertEqual((4, 2), np.asarray(robot["corners_px"]).shape)
            self.assertEqual((4, 2), np.asarray(robot["corners_mm"]).shape)
            self.assertTrue(math.isfinite(robot["heading_rad"]))
            self.assertGreaterEqual(robot["heading_rad"], -math.pi)
            self.assertLess(robot["heading_rad"], math.pi)
            x, y = self.PLACEMENTS[tag_id]
            expected_px = (x + (self.TAG_PX - 1) / 2, y + (self.TAG_PX - 1) / 2)
            np.testing.assert_allclose(robot["tag_center_px"], expected_px, atol=2.5, rtol=0)
            np.testing.assert_allclose(robot["robot_center_px"], expected_px, atol=2.5, rtol=0)
            np.testing.assert_allclose(
                robot["tag_center_mm"],
                (expected_px[0], self.HEIGHT - 1 - expected_px[1]),
                atol=2.5,
                rtol=0,
            )
            np.testing.assert_allclose(
                robot["robot_center_mm"],
                (expected_px[0], self.HEIGHT - 1 - expected_px[1]),
                atol=2.5,
                rtol=0,
            )

    def test_image_detects_four_robots_writes_jsonl_and_annotated_png_headlessly(self):
        image = self.render((tag_id, point) for tag_id, point in self.PLACEMENTS.items())
        source = self.write_image(image)
        report = self.root / "detections.jsonl"
        annotated = self.root / "annotated.png"

        with ExitStack() as stack:
            highgui = [
                stack.enter_context(patch.object(cv2, name))
                for name in ("namedWindow", "resizeWindow", "imshow", "waitKey", "destroyAllWindows")
            ]
            status, output, errors = self.invoke(
                "detect", "--image", source,
                "--calibration", self.calibration, "--tags", self.tags,
                "--report", report, "--output", annotated,
            )

        self.assertEqual(0, status, errors)
        self.assertTrue(all(not function.called for function in highgui),
                        "detect without --preview must remain headless")
        summary = json.loads(output)
        self.assertEqual("image_complete", summary["status"])
        self.assertEqual(1, summary["processed_frames"])
        self.assertEqual(1, summary["complete_observation_frames"])
        self.assertEqual(4, summary["total_robot_detections"])
        self.assertEqual(0, summary["unknown_tag_detections"])
        self.assertEqual(0, summary["duplicate_tag_detections"])
        self.assertEqual(0, summary["missing_robot_frames"])
        self.assertTrue(summary["is_replay"])
        self.assertEqual("DICT_APRILTAG_36h11", summary["dictionary_name"])
        self.assertEqual(["B1", "B2", "H1", "H2"], summary["configured_robot_ids"])
        self.assertEqual(float(self.TAG_PX - 1), summary["tag_size_mm"])
        self.assertFalse(summary["hardware_verified"])
        records = self.read_records(report)
        self.assertEqual(1, len(records))
        self.assert_four_robot_record(records[0])
        rendered = cv2.imdecode(np.frombuffer(annotated.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(image.shape, rendered.shape)

    def test_robot_heading_offset_from_config_is_applied_in_field_coordinates(self):
        offset = 0.35
        self.write_tag_config(heading_offsets_rad={"H1": offset})
        source = self.write_image(self.render(((0, self.PLACEMENTS[0]),)), "heading.png")
        report = self.root / "heading.jsonl"
        status, _, errors = self.invoke(
            "detect", "--image", source, "--calibration", self.calibration,
            "--tags", self.tags, "--report", report,
        )
        self.assertEqual(0, status, errors)
        robot = self.read_records(report)[0]["robots"][0]
        self.assertEqual("H1", robot["robot_id"])
        # The configured forward axis follows canonical corner 0 -> corner 1.
        # For an upright generated marker this is +X in field coordinates.
        expected = offset
        difference = (robot["heading_rad"] - expected + math.pi) % (2 * math.pi) - math.pi
        self.assertAlmostEqual(0.0, difference, delta=0.05)

    def test_video_real_file_io_and_module_entrypoint_report_every_frame(self):
        frame = self.render((tag_id, point) for tag_id, point in self.PLACEMENTS.items())
        video = self.write_video([frame, frame, frame])
        report = self.root / "video-detections.jsonl"
        annotated = self.root / "video-last.png"
        completed = subprocess.run(
            [
                sys.executable, "-m", "robo_control.vision", "detect",
                "--video", str(video), "--calibration", str(self.calibration),
                "--tags", str(self.tags), "--frames", "10", "--max-age-ms", "2000",
                "--report", str(report), "--output", str(annotated),
            ],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
        self.assertEqual(0, completed.returncode, completed.stderr)
        summary = json.loads(completed.stdout)
        self.assertEqual("eof_or_decode_failure", summary["status"])
        self.assertEqual(3, summary["processed_frames"])
        self.assertEqual(3, summary["complete_observation_frames"])
        self.assertEqual(12, summary["total_robot_detections"])
        self.assertEqual(0, summary["missing_robot_frames"])
        self.assertTrue(summary["is_replay"])
        records = self.read_records(report)
        self.assertEqual([1, 2, 3], [record["sequence"] for record in records])
        for record in records:
            self.assert_four_robot_record(record)
        self.assertTrue(annotated.is_file())

    def test_unknown_tag_keeps_known_poses_but_makes_observation_incomplete(self):
        placements = list(self.PLACEMENTS.items()) + [(17, (278, 198))]
        source = self.write_image(self.render(placements), "unknown.png")
        report = self.root / "unknown.jsonl"
        status, output, errors = self.invoke(
            "detect", "--image", source, "--calibration", self.calibration,
            "--tags", self.tags, "--report", report,
        )
        self.assertEqual(0, status, errors)
        summary = json.loads(output)
        self.assertEqual(4, summary["total_robot_detections"])
        self.assertEqual(1, summary["unknown_tag_detections"])
        self.assertEqual(0, summary["complete_observation_frames"])
        record = self.read_records(report)[0]
        self.assertEqual([17], record["unknown_tag_ids"])
        self.assertFalse(record["observation_complete"])
        self.assertEqual(set(self.ROBOTS.values()), {robot["robot_id"] for robot in record["robots"]})

    def test_duplicate_tag_is_discarded_and_makes_observation_incomplete(self):
        placements = list(self.PLACEMENTS.items())
        placements.append((0, (278, 198)))
        source = self.write_image(self.render(placements), "duplicate.png")
        report = self.root / "duplicate.jsonl"
        status, output, errors = self.invoke(
            "detect", "--image", source, "--calibration", self.calibration,
            "--tags", self.tags, "--report", report,
        )
        self.assertEqual(0, status, errors)
        summary = json.loads(output)
        self.assertEqual(3, summary["total_robot_detections"])
        self.assertEqual(1, summary["duplicate_tag_detections"])
        self.assertEqual(0, summary["complete_observation_frames"])
        self.assertEqual(1, summary["missing_robot_frames"])
        record = self.read_records(report)[0]
        self.assertFalse(record["observation_complete"])
        self.assertEqual([0], record["duplicate_tag_ids"])
        self.assertEqual(["H1"], record["missing_robot_ids"])
        self.assertNotIn("H1", {robot["robot_id"] for robot in record["robots"]})

    def test_no_tag_frame_succeeds_but_observation_is_incomplete(self):
        source = self.write_image(self.render(), "empty.png")
        report = self.root / "empty.jsonl"
        status, output, errors = self.invoke(
            "detect", "--image", source, "--calibration", self.calibration,
            "--tags", self.tags, "--report", report,
        )
        self.assertEqual(0, status, errors)
        summary = json.loads(output)
        self.assertEqual("image_complete", summary["status"])
        self.assertEqual(1, summary["processed_frames"])
        self.assertEqual(0, summary["complete_observation_frames"])
        self.assertEqual(0, summary["total_robot_detections"])
        self.assertEqual(1, summary["missing_robot_frames"])
        record = self.read_records(report)[0]
        self.assertEqual([], record["robots"])
        self.assertEqual([], record["unknown_tag_ids"])
        self.assertEqual([], record["duplicate_tag_ids"])
        self.assertEqual(["B1", "B2", "H1", "H2"], record["missing_robot_ids"])
        self.assertFalse(record["observation_complete"])

    def test_missing_input_or_config_and_bad_tag_config_fail_without_outputs(self):
        source = self.write_image(
            self.render((tag_id, point) for tag_id, point in self.PLACEMENTS.items())
        )
        invalid_configs = (
            (self.root / "missing-tags.json", ""),
            (self.root / "bad-json.json", "{"),
            (self.root / "wrong-schema.json", json.dumps({"schema_version": 2})),
            (
                self.root / "unknown-dictionary.json",
                json.dumps({
                    "schema_version": 1,
                    "dictionary_name": "DICT_NOT_REAL",
                    "tag_to_robot": {"0": "H1"},
                }),
            ),
            (
                self.root / "duplicate-robot.json",
                json.dumps({
                    "schema_version": 1,
                    "dictionary_name": "DICT_APRILTAG_36h11",
                    "tag_to_robot": {"0": "H1", "1": "H1"},
                }),
            ),
        )
        for config, contents in invalid_configs:
            if contents:
                config.write_text(contents, encoding="utf-8")
            report = self.root / f"{config.stem}-report.jsonl"
            annotated = self.root / f"{config.stem}-output.png"
            with self.subTest(config=config.name):
                status, output, errors = self.invoke(
                    "detect", "--image", source, "--calibration", self.calibration,
                    "--tags", config, "--report", report, "--output", annotated,
                )
                self.assertEqual(2, status)
                self.assertEqual("", output)
                self.assertIn("vision:", errors)
                self.assertFalse(report.exists())
                self.assertFalse(annotated.exists())

        status, output, errors = self.invoke(
            "detect", "--image", self.root / "missing.png",
            "--calibration", self.calibration, "--tags", self.tags,
        )
        self.assertEqual(2, status)
        self.assertEqual("", output)
        self.assertIn("vision:", errors)

        status, output, errors = self.invoke(
            "detect", "--image", source,
            "--calibration", self.root / "missing-calibration.json", "--tags", self.tags,
        )
        self.assertEqual(2, status)
        self.assertEqual("", output)
        self.assertIn("vision:", errors)


if __name__ == "__main__":
    unittest.main()
