from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

if cv2 is not None and np is not None:
    from robo_control.vision.calibration import CalibrationError, FieldCalibration


@unittest.skipUnless(cv2 is not None and np is not None, "install the vision extra")
class FieldCalibrationTests(unittest.TestCase):
    IMAGE_SIZE = (640, 480)
    FIELD_SIZE = (1143.0, 1181.0)
    CORNERS = ((80, 40), (560, 60), (600, 440), (40, 420))

    def make_calibration(self, **overrides):
        arguments = {
            "image_size_px": self.IMAGE_SIZE,
            "corners_px": self.CORNERS,
            "field_size_mm": self.FIELD_SIZE,
        }
        arguments.update(overrides)
        return FieldCalibration(**arguments)

    @staticmethod
    def project_reference(points_mm):
        """Independent camera projection, not a transform from the implementation."""
        points = np.asarray(points_mm, dtype=np.float64)
        x, y = points[:, 0], points[:, 1]
        denominator = 1.0 + 0.00012 * x + 0.00006 * y
        return np.column_stack(
            (
                (0.38 * x + 0.015 * y + 58.0) / denominator,
                (0.018 * x - 0.28 * y + 390.0) / denominator,
            )
        )

    def reference_calibration(self):
        width, height = self.FIELD_SIZE
        corners_mm = ((0, height), (width, height), (width, 0), (0, 0))
        return self.make_calibration(corners_px=self.project_reference(corners_mm))

    def test_error_is_a_value_error(self):
        self.assertTrue(issubclass(CalibrationError, ValueError))

    def test_named_corners_use_bottom_left_field_origin(self):
        calibration = self.make_calibration()
        expected = ((0, 1181), (1143, 1181), (1143, 0), (0, 0))
        actual = calibration.pixel_to_field_mm(self.CORNERS)
        self.assertIsInstance(actual, np.ndarray)
        np.testing.assert_allclose(actual, expected, atol=1e-4, rtol=0)

    def test_internal_points_match_independent_projective_ground_truth(self):
        calibration = self.reference_calibration()
        points_mm = np.array(
            ((73, 911), (936, 128), (572, 590), (1001, 1013), (164, 246)),
            dtype=np.float64,
        )
        image_points = self.project_reference(points_mm)
        np.testing.assert_allclose(
            calibration.pixel_to_field_mm(image_points),
            points_mm,
            atol=1e-3,
            rtol=0,
        )
        np.testing.assert_allclose(
            calibration.field_mm_to_pixel(points_mm),
            image_points,
            atol=1e-3,
            rtol=0,
        )

    def test_rectified_pixels_flip_y_and_apply_scale(self):
        calibration = self.make_calibration()
        actual = calibration.field_mm_to_rectified_px(
            ((0, 1181), (1143, 0), (300, 581)), pixels_per_mm=0.5
        )
        np.testing.assert_allclose(
            actual, ((0, 0), (571.5, 590.5), (150, 300)), atol=1e-6, rtol=0
        )

    def test_warp_preserves_marker_positions_and_field_orientation(self):
        calibration = self.reference_calibration()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        marker_mm = np.array(((100, 100), (850, 160), (220, 900), (950, 920)))
        colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255))
        for center, color in zip(self.project_reference(marker_mm), colors):
            cv2.circle(image, tuple(np.rint(center).astype(int)), 7, color, -1)

        scale = 0.25
        warped = calibration.warp(image, pixels_per_mm=scale)
        self.assertEqual((math.ceil(1181 * scale) + 1, math.ceil(1143 * scale) + 1, 3), warped.shape)
        self.assertEqual(np.uint8, warped.dtype)
        for (x_mm, y_mm), color in zip(marker_mm, colors):
            # Derive the expected pixel without calling another calibration method.
            x_px = round(x_mm * scale)
            y_px = round((1181 - y_mm) * scale)
            with self.subTest(position_mm=(int(x_mm), int(y_mm))):
                np.testing.assert_array_equal(warped[y_px, x_px], color)

    def test_warp_rejects_a_frame_from_another_resolution(self):
        calibration = self.make_calibration()
        for shape in ((479, 640, 3), (480, 639, 3), (640, 480, 3)):
            with self.subTest(shape=shape), self.assertRaises(CalibrationError):
                calibration.warp(np.zeros(shape, dtype=np.uint8))

    def test_warp_rejects_non_uint8_images(self):
        calibration = self.make_calibration()
        for dtype in (np.float32, np.uint16, np.bool_):
            with self.subTest(dtype=dtype), self.assertRaises(CalibrationError):
                calibration.warp(np.zeros((480, 640, 3), dtype=dtype))

    def test_rectification_rejects_invalid_scales(self):
        calibration = self.make_calibration()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        for scale in (0, -1, True, float("nan"), float("inf")):
            with self.subTest(scale=scale, method="points"), self.assertRaises(CalibrationError):
                calibration.field_mm_to_rectified_px(((20, 30),), pixels_per_mm=scale)
            with self.subTest(scale=scale, method="image"), self.assertRaises(CalibrationError):
                calibration.warp(image, pixels_per_mm=scale)

    def test_reference_checks_measure_euclidean_error_in_millimetres(self):
        calibration = self.reference_calibration()
        points_mm = np.array(((200, 300), (400, 500), (600, 700)), dtype=float)
        # Deliberate independent measurement errors of 5, 10, and 0 mm.
        expected_mm = points_mm + np.array(((3, 4), (-6, 8), (0, 0)))
        report = calibration.check_points(self.project_reference(points_mm), expected_mm)
        self.assertEqual(3, report["count"])
        self.assertAlmostEqual(float(np.sqrt(125.0 / 3.0)), report["rms_error_mm"], delta=1e-3)
        self.assertAlmostEqual(10.0, report["max_error_mm"], delta=1e-3)

    def test_independent_internal_checks_detect_wrong_corner_selection(self):
        width, height = self.FIELD_SIZE
        corners = self.project_reference(((0, height), (width, height), (width, 0), (0, 0)))
        corners[1] += (25, 12)
        calibration = self.make_calibration(corners_px=corners)
        reference_mm = np.array(((250, 900), (850, 950), (900, 350), (400, 400)))
        report = calibration.check_points(self.project_reference(reference_mm), reference_mm)
        self.assertEqual(4, report["count"])
        self.assertGreater(report["rms_error_mm"], 10)
        self.assertGreaterEqual(report["max_error_mm"], report["rms_error_mm"])

    def test_reference_check_rejects_different_point_counts(self):
        calibration = self.make_calibration()
        with self.assertRaises(CalibrationError):
            calibration.check_points(((100, 100), (200, 200)), ((100, 100),))

    def test_coordinate_methods_reject_malformed_or_nonfinite_points(self):
        calibration = self.make_calibration()
        invalid_points = (
            (20, 30),
            ((20, 30, 40),),
            ((20,), (30, 40)),
            ((float("nan"), 30),),
            ((20, float("inf")),),
        )
        for method in (calibration.pixel_to_field_mm, calibration.field_mm_to_pixel):
            for points in invalid_points:
                with self.subTest(method=method.__name__, points=points), self.assertRaises(CalibrationError):
                    method(points)

    def test_image_size_requires_two_positive_integers(self):
        for size in ((0, 480), (-640, 480), (640, 0), (True, 480), (640.5, 480), (640,), (640, 480, 3)):
            with self.subTest(size=size), self.assertRaises(CalibrationError):
                self.make_calibration(image_size_px=size)

    def test_field_size_requires_two_finite_positive_numbers(self):
        for size in ((0, 1181), (1143, -1), (True, 1181), (float("nan"), 1181), (1143, float("inf")), (1143,)):
            with self.subTest(size=size), self.assertRaises(CalibrationError):
                self.make_calibration(field_size_mm=size)

    def test_corners_require_four_finite_xy_pairs(self):
        for corners in (
            self.CORNERS[:3],
            self.CORNERS + ((320, 240),),
            ((80,),) + self.CORNERS[1:],
            ((80, 40, 0),) + self.CORNERS[1:],
            ((float("nan"), 40),) + self.CORNERS[1:],
            ((80, float("inf")),) + self.CORNERS[1:],
        ):
            with self.subTest(corners=corners), self.assertRaises(CalibrationError):
                self.make_calibration(corners_px=corners)

    def test_corners_reject_duplicate_collinear_and_nonconvex_geometry(self):
        invalid_quadrilaterals = {
            "duplicate": ((80, 40), (560, 60), (560, 60), (40, 420)),
            "collinear": ((50, 80), (200, 80), (350, 80), (500, 80)),
            "three_collinear": ((50, 50), (300, 50), (550, 50), (50, 400)),
            "concave": ((80, 40), (560, 60), (250, 180), (40, 420)),
            "crossed": ((80, 40), (600, 440), (560, 60), (40, 420)),
        }
        for name, corners in invalid_quadrilaterals.items():
            with self.subTest(geometry=name), self.assertRaises(CalibrationError):
                self.make_calibration(corners_px=corners)

    def test_corners_reject_reversed_order(self):
        with self.assertRaises(CalibrationError):
            self.make_calibration(corners_px=tuple(reversed(self.CORNERS)))

    def test_corners_must_be_inside_the_calibrated_image(self):
        for corner in ((-1, 40), (80, -1), (640, 40), (80, 480)):
            with self.subTest(corner=corner), self.assertRaises(CalibrationError):
                self.make_calibration(corners_px=(corner,) + self.CORNERS[1:])

    def test_save_records_named_corners_and_coordinate_convention(self):
        calibration = self.make_calibration()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.json"
            calibration.save(path)
            raw = json.loads(path.read_text(encoding="utf-8"))
            loaded = FieldCalibration.load(path)
        self.assertEqual(1, raw["schema_version"])
        self.assertEqual([640, 480], raw["image_size_px"])
        self.assertEqual([1143.0, 1181.0], raw["field_size_mm"])
        self.assertEqual("bottom_left_x_right_y_up_mm", raw["coordinate_system"])
        self.assertEqual(
            {
                "top_left": [80, 40],
                "top_right": [560, 60],
                "bottom_right": [600, 440],
                "bottom_left": [40, 420],
            },
            raw["corners_px"],
        )
        points = ((170, 120), (480, 320), (310, 240))
        np.testing.assert_allclose(
            loaded.pixel_to_field_mm(points), calibration.pixel_to_field_mm(points), atol=1e-6, rtol=0
        )

    def test_load_rejects_unsupported_or_ambiguous_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.json"
            self.make_calibration().save(path)
            original = json.loads(path.read_text(encoding="utf-8"))
            invalid_metadata = (
                ("schema_version", 2),
                ("schema_version", True),
                ("schema_version", "1"),
                ("coordinate_system", "top_left_x_right_y_down_mm"),
                ("coordinate_system", None),
            )
            for key, value in invalid_metadata:
                raw = dict(original)
                raw[key] = value
                path.write_text(json.dumps(raw), encoding="utf-8")
                with self.subTest(key=key, value=value), self.assertRaises(CalibrationError):
                    FieldCalibration.load(path)

    def test_load_requires_all_four_named_corners(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.json"
            self.make_calibration().save(path)
            raw = json.loads(path.read_text(encoding="utf-8"))
            del raw["corners_px"]["bottom_left"]
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(CalibrationError):
                FieldCalibration.load(path)


if __name__ == "__main__":
    unittest.main()
