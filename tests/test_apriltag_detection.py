from __future__ import annotations

import io
import json
import math
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from typing import ClassVar
from unittest.mock import patch

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from robo_control.adapters import CameraFrame

if cv2 is not None and np is not None and hasattr(cv2, "aruco"):
    from robo_control.vision.calibration import CalibrationError, FieldCalibration
    from robo_control.vision.tags import (
        AprilTagDetector,
        TagDetectionError,
        TagDetectorConfig,
        default_tag_config_path,
    )
    from tools.generate_robot_tags import main as generate_robot_tags_main


HAS_ARUCO = (
    cv2 is not None
    and np is not None
    and hasattr(cv2, "aruco")
    and hasattr(cv2.aruco, "generateImageMarker")
)


@unittest.skipUnless(HAS_ARUCO, "install the vision extra with OpenCV ArUco support")
class AprilTagDetectorTests(unittest.TestCase):
    IMAGE_SIZE = (960, 720)
    FIELD_SIZE = (800.0, 600.0)
    FIELD_CORNERS_PX = ((110, 60), (860, 95), (900, 660), (70, 625))
    TAG_TO_ROBOT: ClassVar[dict[int, str]] = {0: "H1", 1: "H2", 2: "B1", 3: "B2"}
    DICTIONARY_NAME = "DICT_APRILTAG_36h11"
    TAG_SIZE_MM = 76.0

    def setUp(self) -> None:
        self.calibration = FieldCalibration(
            image_size_px=self.IMAGE_SIZE,
            corners_px=self.FIELD_CORNERS_PX,
            field_size_mm=self.FIELD_SIZE,
        )
        self.config = TagDetectorConfig(
            dictionary_name=self.DICTIONARY_NAME,
            tag_to_robot=self.TAG_TO_ROBOT,
        )
        self.detector = AprilTagDetector(self.config, self.calibration)

    def frame(self, image, *, sequence: int = 17) -> CameraFrame:
        return CameraFrame(
            image=image,
            captured_at_s=123.25,
            received_at_s=123.27,
            sequence=sequence,
            source_name="camera:ceiling:session-1",
        )

    def blank_image(self):
        width, height = self.IMAGE_SIZE
        return np.full((height, width, 3), 255, dtype=np.uint8)

    @staticmethod
    def _marker_field_corners(center_mm, heading_rad: float, size_mm: float):
        """Return canonical marker corners TL, TR, BR, BL in field coordinates."""
        half = size_mm / 2.0
        local = np.asarray(
            ((-half, half), (half, half), (half, -half), (-half, -half)),
            dtype=np.float64,
        )
        cosine, sine = math.cos(heading_rad), math.sin(heading_rad)
        rotation = np.asarray(((cosine, -sine), (sine, cosine)))
        return local @ rotation.T + np.asarray(center_mm, dtype=np.float64)

    def add_marker(
        self,
        image,
        tag_id: int,
        center_mm,
        heading_rad: float = 0.0,
        *,
        size_mm: float | None = None,
    ):
        """Project a ground-plane marker through the independent field homography."""
        dictionary_id = getattr(cv2.aruco, self.DICTIONARY_NAME)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        marker_size_px = 180
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_size_px)
        field_corners = self._marker_field_corners(
            center_mm, heading_rad, size_mm or self.TAG_SIZE_MM
        )
        target = self.calibration.field_mm_to_pixel(field_corners).astype(np.float32)
        source = np.asarray(
            (
                (0, 0),
                (marker_size_px - 1, 0),
                (marker_size_px - 1, marker_size_px - 1),
                (0, marker_size_px - 1),
            ),
            dtype=np.float32,
        )
        transform = cv2.getPerspectiveTransform(source, target)
        width, height = self.IMAGE_SIZE
        projected = cv2.warpPerspective(
            marker,
            transform,
            (width, height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=255,
        )
        projected = cv2.cvtColor(projected, cv2.COLOR_GRAY2BGR)
        np.minimum(image, projected, out=image)
        return target

    @staticmethod
    def angle_error(actual: float, expected: float) -> float:
        return (actual - expected + math.pi) % (2.0 * math.pi) - math.pi

    def test_detects_four_projected_tags_with_field_positions_and_headings(self) -> None:
        # Marker order in the image is deliberately unrelated to tag ID order.
        specifications = {
            3: ((620.0, 150.0), math.radians(-55.0)),
            0: ((145.0, 485.0), math.radians(0.0)),
            2: ((590.0, 475.0), math.radians(92.0)),
            1: ((325.0, 265.0), math.radians(34.0)),
        }
        image = self.blank_image()
        expected_corners = {}
        for tag_id, (center, heading) in specifications.items():
            expected_corners[tag_id] = self.add_marker(image, tag_id, center, heading)
        original = image.copy()

        batch = self.detector.detect(self.frame(image))

        self.assertEqual(17, batch.frame_sequence)
        self.assertEqual("camera:ceiling:session-1", batch.source_name)
        self.assertEqual((0, 1, 2, 3), tuple(item.tag_id for item in batch.observations))
        self.assertEqual((), batch.unknown_tag_ids)
        self.assertEqual((), batch.duplicate_tag_ids)
        self.assertEqual((), batch.missing_robot_ids)
        self.assertEqual((), batch.rejected)
        self.assertTrue(batch.observation_complete)
        np.testing.assert_array_equal(original, image)

        for observation in batch.observations:
            expected_center, expected_heading = specifications[observation.tag_id]
            with self.subTest(tag_id=observation.tag_id):
                self.assertEqual(self.TAG_TO_ROBOT[observation.tag_id], observation.robot_id)
                self.assertEqual(123.25, observation.captured_at_s)
                np.testing.assert_allclose(
                    observation.tag_center_mm, expected_center, atol=2.5, rtol=0
                )
                np.testing.assert_allclose(
                    observation.robot_center_mm, expected_center, atol=2.5, rtol=0
                )
                self.assertLess(
                    abs(self.angle_error(observation.heading_rad, expected_heading)),
                    math.radians(3.0),
                )
                self.assertEqual((2,), np.asarray(observation.tag_center_px).shape)
                np.testing.assert_allclose(
                    observation.robot_center_px, observation.tag_center_px, atol=1e-6, rtol=0
                )
                np.testing.assert_allclose(
                    observation.robot_center_mm, observation.tag_center_mm, atol=1e-6, rtol=0
                )
                self.assertEqual((4, 2), np.asarray(observation.corners_px).shape)
                np.testing.assert_allclose(
                    observation.corners_px,
                    expected_corners[observation.tag_id],
                    atol=3.0,
                    rtol=0,
                )

    def test_heading_and_robot_center_offsets_are_applied_in_robot_axes(self) -> None:
        image = self.blank_image()
        self.add_marker(image, 0, (400.0, 300.0), math.radians(170.0))
        detector = AprilTagDetector(
            TagDetectorConfig(
                dictionary_name=self.DICTIONARY_NAME,
                tag_to_robot={0: "H1"},
                heading_offsets_rad={"H1": math.radians(40.0)},
                robot_center_from_tag_mm={"H1": (100.0, 20.0)},
            ),
            self.calibration,
        )

        observation = detector.detect(self.frame(image)).observations[0]

        self.assertAlmostEqual(
            math.radians(-150.0), observation.heading_rad, delta=math.radians(3.0)
        )
        np.testing.assert_allclose(observation.tag_center_mm, (400.0, 300.0), atol=2.5, rtol=0)
        expected_center = (
            400.0 + 100.0 * math.cos(math.radians(-150.0))
            - 20.0 * math.sin(math.radians(-150.0)),
            300.0 + 100.0 * math.sin(math.radians(-150.0))
            + 20.0 * math.cos(math.radians(-150.0)),
        )
        np.testing.assert_allclose(
            observation.robot_center_mm, expected_center, atol=3.0, rtol=0
        )
        np.testing.assert_allclose(
            observation.robot_center_px,
            self.calibration.field_mm_to_pixel((expected_center,))[0],
            atol=3.0,
            rtol=0,
        )
        self.assertGreaterEqual(observation.heading_rad, -math.pi)
        self.assertLess(observation.heading_rad, math.pi)

    def test_annotation_uses_adjusted_heading_and_robot_center_without_mutating_input(self) -> None:
        image = self.blank_image()
        self.add_marker(image, 0, (350.0, 280.0), math.radians(12.0))
        detector = AprilTagDetector(
            TagDetectorConfig(
                dictionary_name=self.DICTIONARY_NAME,
                tag_to_robot={0: "H1"},
                heading_offsets_rad={"H1": math.radians(78.0)},
                robot_center_from_tag_mm={"H1": (90.0, 25.0)},
            ),
            self.calibration,
        )
        batch = detector.detect(self.frame(image))
        observation = batch.observations[0]
        original = image.copy()
        expected_start = tuple(np.rint(observation.robot_center_px).astype(int))
        arrow_tip_mm = np.asarray(observation.robot_center_mm) + 75.0 * np.asarray(
            (math.cos(observation.heading_rad), math.sin(observation.heading_rad))
        )
        expected_tip = tuple(
            np.rint(self.calibration.field_mm_to_pixel((arrow_tip_mm,))[0]).astype(int)
        )

        with patch.object(cv2, "arrowedLine", wraps=cv2.arrowedLine) as arrowed_line:
            annotated = detector.annotate(image, batch)

        self.assertAlmostEqual(math.pi / 2.0, observation.heading_rad, delta=math.radians(3.0))
        self.assertGreater(
            float(
                np.linalg.norm(
                    np.asarray(observation.robot_center_px)
                    - np.asarray(observation.tag_center_px)
                )
            ),
            20.0,
        )
        arrowed_line.assert_called_once()
        call = arrowed_line.call_args.args
        self.assertEqual(expected_start, call[1])
        self.assertEqual(expected_tip, call[2])
        self.assertFalse(np.shares_memory(annotated, image))
        np.testing.assert_array_equal(original, image)
        self.assertFalse(np.array_equal(annotated, original))

    def test_unknown_tag_is_reported_and_never_becomes_a_robot_observation(self) -> None:
        image = self.blank_image()
        self.add_marker(image, 0, (180.0, 300.0))
        self.add_marker(image, 7, (600.0, 300.0))

        batch = self.detector.detect(self.frame(image))

        self.assertEqual((0,), tuple(item.tag_id for item in batch.observations))
        self.assertEqual((7,), batch.unknown_tag_ids)
        self.assertEqual(("B1", "B2", "H2"), batch.missing_robot_ids)
        self.assertTrue(any(item.tag_id == 7 and item.reason == "unknown_tag" for item in batch.rejected))
        self.assertFalse(batch.observation_complete)

    def test_duplicate_tag_id_is_fail_closed(self) -> None:
        image = self.blank_image()
        self.add_marker(image, 0, (180.0, 420.0))
        self.add_marker(image, 0, (610.0, 180.0), math.radians(35.0))

        batch = self.detector.detect(self.frame(image))

        self.assertEqual((), batch.observations)
        self.assertEqual((0,), batch.duplicate_tag_ids)
        self.assertIn("H1", batch.missing_robot_ids)
        self.assertTrue(any(item.tag_id == 0 and item.reason == "duplicate_tag" for item in batch.rejected))
        self.assertFalse(batch.observation_complete)

    def test_out_of_field_center_is_rejected(self) -> None:
        image = self.blank_image()
        self.add_marker(image, 0, (400.0, 640.0))

        batch = self.detector.detect(self.frame(image))

        self.assertEqual((), batch.observations)
        self.assertEqual(("B1", "B2", "H1", "H2"), batch.missing_robot_ids)
        self.assertTrue(any(item.tag_id == 0 and item.reason == "out_of_field" for item in batch.rejected))
        self.assertFalse(batch.observation_complete)

    def test_inverse_projection_failure_rejects_only_the_affected_tag(self) -> None:
        image = self.blank_image()
        self.add_marker(image, 0, (180.0, 300.0))
        self.add_marker(image, 1, (610.0, 300.0))
        detector = AprilTagDetector(
            TagDetectorConfig(
                dictionary_name=self.DICTIONARY_NAME,
                tag_to_robot={0: "H1", 1: "H2"},
            ),
            self.calibration,
        )
        original_projection = FieldCalibration.field_mm_to_pixel

        def fail_only_for_h1(calibration, points):
            projected_point = np.asarray(points, dtype=np.float64).reshape(-1, 2)[0]
            if len(points) == 1 and projected_point[0] < 300.0:
                raise CalibrationError("forced inverse projection failure")
            return original_projection(calibration, points)

        with patch.object(
            FieldCalibration,
            "field_mm_to_pixel",
            autospec=True,
            side_effect=fail_only_for_h1,
        ):
            batch = detector.detect(self.frame(image))

        self.assertEqual((1,), tuple(item.tag_id for item in batch.observations))
        self.assertEqual(("H1",), batch.missing_robot_ids)
        self.assertTrue(
            any(
                item.tag_id == 0 and item.reason == "invalid_projection"
                for item in batch.rejected
            )
        )
        self.assertFalse(batch.observation_complete)

    def test_configured_physical_tag_size_rejects_a_wrong_scale(self) -> None:
        image = self.blank_image()
        self.add_marker(image, 0, (400.0, 300.0), size_mm=self.TAG_SIZE_MM)
        matching_detector = AprilTagDetector(
            TagDetectorConfig(
                dictionary_name=self.DICTIONARY_NAME,
                tag_to_robot={0: "H1"},
                tag_size_mm=self.TAG_SIZE_MM,
                tag_size_tolerance_fraction=0.1,
            ),
            self.calibration,
        )
        wrong_scale_detector = AprilTagDetector(
            TagDetectorConfig(
                dictionary_name=self.DICTIONARY_NAME,
                tag_to_robot={0: "H1"},
                tag_size_mm=40.0,
                tag_size_tolerance_fraction=0.1,
            ),
            self.calibration,
        )

        accepted = matching_detector.detect(self.frame(image))
        rejected = wrong_scale_detector.detect(self.frame(image))

        self.assertAlmostEqual(
            self.TAG_SIZE_MM,
            accepted.observations[0].observed_tag_size_mm,
            delta=2.5,
        )
        self.assertTrue(accepted.observation_complete)
        self.assertEqual((), rejected.observations)
        self.assertEqual(("H1",), rejected.missing_robot_ids)
        self.assertTrue(
            any(
                item.tag_id == 0 and item.reason == "tag_size_mismatch"
                for item in rejected.rejected
            )
        )
        self.assertFalse(rejected.observation_complete)

    def test_configured_margin_can_accept_a_small_coordinate_overshoot(self) -> None:
        image = self.blank_image()
        self.add_marker(image, 0, (400.0, 620.0))
        detector = AprilTagDetector(
            TagDetectorConfig(
                dictionary_name=self.DICTIONARY_NAME,
                tag_to_robot={0: "H1"},
                allowed_margin_mm=25.0,
            ),
            self.calibration,
        )

        batch = detector.detect(self.frame(image))

        self.assertEqual((0,), tuple(item.tag_id for item in batch.observations))
        self.assertEqual((), batch.rejected)
        self.assertTrue(batch.observation_complete)

    def test_blank_frame_is_a_normal_lost_observation_not_a_detector_error(self) -> None:
        batch = self.detector.detect(self.frame(self.blank_image()))

        self.assertEqual((), batch.observations)
        self.assertEqual((), batch.unknown_tag_ids)
        self.assertEqual((), batch.duplicate_tag_ids)
        self.assertEqual(("B1", "B2", "H1", "H2"), batch.missing_robot_ids)
        self.assertEqual((), batch.rejected)
        self.assertFalse(batch.observation_complete)

    def test_rejects_invalid_images_and_resolution_changes(self) -> None:
        width, height = self.IMAGE_SIZE
        invalid_images = (
            None,
            {"pixels": []},
            np.zeros((height, width, 3), dtype=np.float32),
            np.zeros((height, width, 2), dtype=np.uint8),
            np.zeros((height, width, 3, 1), dtype=np.uint8),
            np.zeros((height - 1, width, 3), dtype=np.uint8),
            np.zeros((height, width - 1, 3), dtype=np.uint8),
        )
        for image in invalid_images:
            with (
                self.subTest(type=type(image), shape=getattr(image, "shape", None)),
                self.assertRaises(TagDetectionError),
            ):
                self.detector.detect(self.frame(image))

    def test_grayscale_and_bgra_frames_are_supported(self) -> None:
        bgr = self.blank_image()
        self.add_marker(bgr, 0, (400.0, 300.0))
        images = (
            cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY),
            cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA),
        )
        for sequence, image in enumerate(images, 1):
            with self.subTest(shape=image.shape):
                detector = AprilTagDetector(
                    TagDetectorConfig(
                        dictionary_name=self.DICTIONARY_NAME,
                        tag_to_robot={0: "H1"},
                    ),
                    self.calibration,
                )
                batch = detector.detect(self.frame(image, sequence=sequence))
                self.assertEqual((0,), tuple(item.tag_id for item in batch.observations))
                self.assertTrue(batch.observation_complete)

    def test_config_rejects_invalid_dictionary_and_robot_mapping(self) -> None:
        invalid_cases = (
            {"dictionary_name": "NOT_A_DICTIONARY"},
            {"dictionary_name": ""},
            {"dictionary_name": 36},
            {"tag_to_robot": {}},
            {"tag_to_robot": []},
            {"tag_to_robot": {True: "H1"}},
            {"tag_to_robot": {-1: "H1"}},
            {"tag_to_robot": {0: ""}},
            {"tag_to_robot": {0: 1}},
            {"tag_to_robot": {0: "H1", 1: "H1"}},
        )
        for overrides in invalid_cases:
            arguments = {
                "dictionary_name": self.DICTIONARY_NAME,
                "tag_to_robot": self.TAG_TO_ROBOT,
            }
            arguments.update(overrides)
            with self.subTest(overrides=overrides), self.assertRaises(TagDetectionError):
                TagDetectorConfig(**arguments)

    def test_config_rejects_invalid_heading_offsets(self) -> None:
        invalid_offsets = (
            {"UNKNOWN": 0.0},
            {"H1": True},
            {"H1": "0.1"},
            {"H1": float("nan")},
            {"H1": float("inf")},
            [],
        )
        for offsets in invalid_offsets:
            with self.subTest(offsets=offsets), self.assertRaises(TagDetectionError):
                TagDetectorConfig(
                    dictionary_name=self.DICTIONARY_NAME,
                    tag_to_robot=self.TAG_TO_ROBOT,
                    heading_offsets_rad=offsets,
                )

    def test_config_rejects_invalid_allowed_margin(self) -> None:
        for margin in (-0.01, True, "0", float("nan"), float("inf")):
            with self.subTest(margin=margin), self.assertRaises(TagDetectionError):
                TagDetectorConfig(
                    dictionary_name=self.DICTIONARY_NAME,
                    tag_to_robot=self.TAG_TO_ROBOT,
                    allowed_margin_mm=margin,
                )

    def test_config_save_and_load_preserve_mounting_and_safety_values(self) -> None:
        config = TagDetectorConfig(
            dictionary_name=self.DICTIONARY_NAME,
            tag_to_robot={3: "B2", 0: "H1"},
            heading_offsets_rad={"H1": 0.125, "B2": -0.25},
            robot_center_from_tag_mm={"H1": (42.5, -11.0), "B2": (-7.0, 18.25)},
            allowed_margin_mm=9.5,
            tag_size_mm=76.0,
            tag_size_tolerance_fraction=0.12,
            hardware_verified=True,
        )
        tests_directory = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory(prefix=".tmp-tag-config-", dir=tests_directory) as directory:
            path = Path(directory) / "nested" / "robot-tags.json"
            config.save(path)
            encoded = json.loads(path.read_text(encoding="utf-8"))
            loaded = TagDetectorConfig.load(path)

        self.assertEqual(config.as_dict(), loaded.as_dict())
        self.assertEqual(
            {"forward_mm": 42.5, "left_mm": -11.0},
            encoded["robot_center_from_tag_mm"]["H1"],
        )
        self.assertTrue(encoded["hardware_verified"])

    def test_default_source_and_packaged_tag_configs_are_byte_identical(self) -> None:
        repository_root = Path(__file__).resolve().parents[1]
        source = repository_root / "config" / "robot_tags.json"
        packaged = repository_root / "robo_control" / "data" / "robot_tags.json"

        self.assertEqual(source.resolve(), default_tag_config_path().resolve())
        self.assertEqual(source.read_bytes(), packaged.read_bytes())
        loaded = TagDetectorConfig.load(source)
        self.assertEqual(self.TAG_TO_ROBOT, dict(loaded.tag_to_robot))
        self.assertEqual(
            {robot_id: (0.0, 0.0) for robot_id in self.TAG_TO_ROBOT.values()},
            dict(loaded.robot_center_from_tag_mm),
        )
        self.assertFalse(loaded.hardware_verified)

    def test_load_rejects_malformed_robot_center_json(self) -> None:
        malformed_values = (
            [],
            {"H1": [1.0, 2.0]},
            {"H1": {"forward_mm": 1.0}},
            {"H1": {"forward_mm": 1.0, "left_mm": 2.0, "units": "mm"}},
            {"UNKNOWN": {"forward_mm": 1.0, "left_mm": 2.0}},
            {"H1": {"forward_mm": "ahead", "left_mm": 2.0}},
        )
        tests_directory = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory(prefix=".tmp-tag-json-", dir=tests_directory) as directory:
            path = Path(directory) / "robot-tags.json"
            for malformed in malformed_values:
                raw = self.config.as_dict()
                raw["robot_center_from_tag_mm"] = malformed
                path.write_text(json.dumps(raw), encoding="utf-8")
                with self.subTest(value=malformed), self.assertRaises(TagDetectionError) as error:
                    TagDetectorConfig.load(path)
                self.assertEqual("invalid_config", error.exception.reason)

    def test_tag_generator_rejects_too_narrow_quiet_zone_without_outputs(self) -> None:
        tests_directory = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory(prefix=".tmp-tag-quiet-", dir=tests_directory) as directory:
            output_directory = Path(directory) / "generated"
            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr):
                status = generate_robot_tags_main(
                    [
                        "--config",
                        str(default_tag_config_path()),
                        "--output-dir",
                        str(output_directory),
                        "--side-px",
                        "800",
                        "--quiet-zone-px",
                        "99",
                    ]
                )

            self.assertEqual(2, status)
            self.assertEqual("", stdout.getvalue())
            self.assertIn("quiet-zone-px must be >= 100", stderr.getvalue())
            self.assertFalse(output_directory.exists())


if __name__ == "__main__":
    unittest.main()
