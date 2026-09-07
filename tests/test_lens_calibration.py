from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from robo_control.vision.calibration import CalibrationError, FieldCalibration
from robo_control.vision.lens import LensCalibration
from robo_control.vision.lens_calibrate import board_points, fit_lens


def known_lens():
    return LensCalibration((640, 480), ((500., 0., 320.), (0., 510., 240.), (0., 0., 1.)),
                           (-.15, .03, .001, -.001, 0.), "generated-fixture-not-a-physical-camera")


def reference_distortion(ideal, lens):
    """OpenCV 3D projection is independent of our Brown5 forward implementation."""
    points = np.asarray(ideal, dtype=np.float64)
    k = np.asarray(lens.camera_matrix)
    xyz = np.column_stack(((points[:, 0] - k[0, 2]) / k[0, 0],
                           (points[:, 1] - k[1, 2]) / k[1, 1], np.ones(len(points))))
    raw, _ = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), k, np.asarray(lens.distortion_coefficients))
    return raw.reshape(-1, 2)


def board_dataset(count=24, *, render=False):
    lens = known_lens()
    obj = board_points((7, 5), 24.)
    k, d = np.asarray(lens.camera_matrix), np.asarray(lens.distortion_coefficients)
    views, images = [], []
    for i in range(count):
        rotation = np.array((.45 * math.sin(i * .8), .45 * math.cos(i * .65), .12 * math.sin(i)))
        translation = np.array((-72 + (i % 6 - 2.5) * 42, -48 + (i // 6 - 1.5) * 46, 400 + (i % 3) * 25.), dtype=float)
        points, _ = cv2.projectPoints(obj, rotation, translation, k, d)
        views.append(points.reshape(-1, 2))
        if not render:
            continue
        # Rasterise projected little squares at 3x resolution. Polygon edges
        # approximate the curved lens edge within each square; this is a test
        # photograph fixture, not a physical calibration accuracy certificate.
        image = np.full((1440, 1920), 255, np.uint8)
        for row in range(-1, 5):
            for col in range(-1, 7):
                if (row + col) % 2:
                    continue
                square = np.array(((col*24, row*24, 0), ((col+1)*24, row*24, 0),
                                   ((col+1)*24, (row+1)*24, 0), (col*24, (row+1)*24, 0)), dtype=float)
                projected, _ = cv2.projectPoints(square, rotation, translation, k, d)
                cv2.fillConvexPoly(image, np.rint(projected.reshape(-1, 2) * 3).astype(np.int32), 0)
        images.append(cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA))
    return lens, views, images


@unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
class LensModelTests(unittest.TestCase):
    def test_raw_ideal_roundtrip_matches_independent_projection(self):
        lens = known_lens()
        ideal = np.array(((30, 30), (610, 30), (610, 450), (30, 450), (320, 240), (140, 330)), float)
        raw = reference_distortion(ideal, lens)
        np.testing.assert_allclose(lens.distort_points(ideal), raw, atol=1e-10)
        np.testing.assert_allclose(lens.undistort_points(raw), ideal, atol=1e-7)

    def test_profile_roundtrip_is_immutable_and_never_overwrites(self):
        lens = known_lens()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "lens.json"
            lens.save(path)
            restored = LensCalibration.load(path)
            self.assertEqual(lens, restored)
            self.assertEqual(lens.fingerprint, restored.fingerprint)
            with self.assertRaises(FileExistsError):
                lens.save(path)
        data = lens.as_dict()
        data["camera_matrix"][0][0] = 1
        self.assertEqual(500., lens.camera_matrix[0][0])

    def test_invalid_model_schema_coefficients_and_matrix_are_rejected(self):
        base = known_lens().as_dict()
        changes = [{"model": "fisheye"}, {"schema_version": True}, {"unexpected": 1},
                   {"image_size_px": [320, 240]}, {"image_size_px": [True, 480]},
                   {"distortion_coefficients": [0, 0, 0, 0]}, {"distortion_coefficients": [float("nan")] * 5},
                   {"distortion_coefficients": [True, 0, 0, 0, 0]}, {"camera_label": ""},
                   {"camera_matrix": [[500, 1, 320], [0, 510, 240], [0, 0, 1]]},
                   {"camera_matrix": [[-500, 0, 320], [0, 510, 240], [0, 0, 1]]}]
        for change in changes:
            with self.subTest(change=change), self.assertRaises(CalibrationError):
                LensCalibration.from_dict({**base, **change})

    def test_obviously_folded_radial_profile_is_rejected(self):
        data = known_lens().as_dict()
        for coefficients in ([-2., 0, 0, 0, 0], [0, 0, 5, 5, 0]):
            with self.subTest(coefficients=coefficients), self.assertRaises(CalibrationError):
                LensCalibration.from_dict({**data, "distortion_coefficients": coefficients})

    def test_wrong_image_size_and_outside_or_nonfinite_points_are_rejected(self):
        lens = known_lens()
        for value in ([[-1, 0]], [[640, 0]], [[0, 480]], [[math.inf, 0]], [[0]]):
            with self.subTest(value=value), self.assertRaises(CalibrationError):
                lens.undistort_points(value)
        for shape in ((240, 320), (480, 640, 2)):
            with self.subTest(shape=shape), self.assertRaises(CalibrationError):
                lens.undistort_image(np.zeros(shape, np.uint8))


@unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
class LensFieldTests(unittest.TestCase):
    def setUp(self):
        self.lens = known_lens()
        self.ideal_corners = np.array(((60, 40), (580, 40), (580, 440), (60, 440)), float)
        self.raw_corners = reference_distortion(self.ideal_corners, self.lens)
        self.calibration = FieldCalibration((640, 480), self.raw_corners, (1143., 1181.), self.lens)

    def test_corrected_field_coordinates_and_overlay_use_raw_pixels(self):
        expected = np.array(((100, 100), (1000, 100), (1000, 1000), (100, 1000), (500, 500)), float)
        ideal = np.column_stack((60 + expected[:, 0] * 520 / 1143, 440 - expected[:, 1] * 400 / 1181))
        raw = reference_distortion(ideal, self.lens)
        np.testing.assert_allclose(self.calibration.pixel_to_field_mm(raw), expected, atol=2e-4)
        np.testing.assert_allclose(self.calibration.field_mm_to_pixel(expected), raw, atol=2e-4)
        uncorrected = FieldCalibration((640, 480), self.raw_corners, (1143, 1181))
        self.assertGreater(uncorrected.check_points(raw, expected)["max_error_mm"], 5.)
        self.assertLess(self.calibration.check_points(raw, expected)["max_error_mm"], .001)

    def test_direct_raw_warp_preserves_field_corners_outside_ideal_image_canvas(self):
        raw_corners = ((2, 2), (637, 2), (637, 477), (2, 477))
        calibration = FieldCalibration((640, 480), raw_corners, (630, 470), self.lens)
        image = np.zeros((480, 640, 3), np.uint8)
        colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255))
        for point, color in zip(raw_corners, colors):
            cv2.circle(image, point, 6, color, -1)
        warped = calibration.warp(image, 1.)
        for point, color in zip(((0, 0), (630, 0), (630, 470), (0, 470)), colors):
            np.testing.assert_array_equal(warped[point[1], point[0]], color)

    def test_schema_one_remains_compatible_and_cannot_silently_ignore_lens_data(self):
        plain = FieldCalibration((640, 480), self.raw_corners, (1143, 1181))
        data = plain.as_dict()
        self.assertEqual(1, data["schema_version"])
        self.assertEqual(plain, FieldCalibration.from_dict(data))
        data["lens"] = self.lens.as_dict()
        with self.assertRaises(CalibrationError):
            FieldCalibration.from_dict(data)

    def test_schema_two_embeds_profile_and_preserves_raw_geometry(self):
        data = self.calibration.as_dict()
        self.assertEqual(2, data["schema_version"])
        self.assertEqual("raw_camera_pixels", data["pixel_geometry"])
        self.assertEqual(self.calibration, FieldCalibration.from_dict(data))
        for change in ({"lens": None}, {"pixel_geometry": "already_undistorted"}, {"image_size_px": [1280, 960]}):
            with self.subTest(change=change), self.assertRaises(CalibrationError):
                FieldCalibration.from_dict({**data, **change})


@unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
class LensFitTests(unittest.TestCase):
    def test_known_projection_recovers_intrinsics_and_uses_held_out_views(self):
        expected, views, _ = board_dataset()
        lens, report = fit_lens(views, (640, 480), (7, 5), 24., "fixture")
        np.testing.assert_allclose(lens.camera_matrix, expected.camera_matrix, atol=.01)
        np.testing.assert_allclose(lens.distortion_coefficients, expected.distortion_coefficients, atol=.002)
        self.assertEqual([5, 11, 17, 23], report["held_out_view_indices"])
        self.assertLess(max(report["per_view_rms_px"]), .001)
        self.assertFalse(report["physical_accuracy_verified"])

    def test_too_few_or_duplicate_views_cannot_manufacture_a_fit(self):
        _, views, _ = board_dataset()
        for dataset in (views[:11], [views[0]] * 12):
            with self.subTest(count=len(dataset)), self.assertRaises(CalibrationError):
                fit_lens(dataset, (640, 480), (7, 5), 24., "fixture")

    def test_held_out_bad_view_is_not_used_to_refit_or_silently_dropped(self):
        _, views, _ = board_dataset()
        views[5] = views[5].copy()
        views[5][0] += (25, -25)
        with self.assertRaisesRegex(CalibrationError, "threshold"):
            fit_lens(views, (640, 480), (7, 5), 24., "fixture")

    def test_individual_corner_error_cannot_hide_beneath_per_view_rms(self):
        _, views, _ = board_dataset()
        views[5] = views[5].copy()
        views[5][0] += (5, 0)
        with self.assertRaisesRegex(CalibrationError, "threshold"):
            fit_lens(views, (640, 480), (7, 5), 24., "fixture")

    def test_invalid_board_threshold_and_coverage_are_rejected(self):
        _, views, _ = board_dataset()
        for board, square, threshold in (((True, 5), 24, 1), ((2, 5), 24, 1), ((7, 5), math.nan, 1),
                                          ((7, 5), 24, 3), ((7, 5), 24, True)):
            with self.subTest(board=board, square=square, threshold=threshold), self.assertRaises(CalibrationError):
                fit_lens(views, (640, 480), board, square, "fixture", threshold)
        compressed = [(view - (320, 240)) * .2 + (320, 240) for view in views]
        with self.assertRaises(CalibrationError):
            fit_lens(compressed, (640, 480), (7, 5), 24., "fixture")


if __name__ == "__main__":
    unittest.main()
