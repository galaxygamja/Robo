"""Planar image calibration. Public world coordinates are explicitly in mm."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

CORNER_NAMES = ("top_left", "top_right", "bottom_right", "bottom_left")
COORDINATE_SYSTEM = "bottom_left_x_right_y_up_mm"


class CalibrationError(ValueError):
    """The image or calibration cannot yield trustworthy field coordinates."""


def vision_dependencies():
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError('Install vision dependencies: python -m pip install -e ".[vision]"') from exc
    return cv2, np


def _positive_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibrationError(f"{name} must be a finite positive number")
    if not math.isfinite(value) or value <= 0:
        raise CalibrationError(f"{name} must be a finite positive number")
    return float(value)


def _points(values: Any):
    _, np = vision_dependencies()
    try:
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise CalibrationError("points must be a finite N x 2 array") from exc
    if array.ndim != 2 or array.shape[1] != 2 or not np.isfinite(array).all():
        raise CalibrationError("points must be a finite N x 2 array")
    return array


def _project(points: Any, matrix: Any):
    _, np = vision_dependencies()
    points = _points(points)
    homogeneous = np.column_stack((points, np.ones(len(points)))) @ matrix.T
    if (np.abs(homogeneous[:, 2]) < 1e-10).any():
        raise CalibrationError("point is on the homography horizon")
    result = homogeneous[:, :2] / homogeneous[:, 2, None]
    if not np.isfinite(result).all():
        raise CalibrationError("coordinate transform produced a nonfinite point")
    return result


@dataclass(frozen=True, slots=True)
class FieldCalibration:
    """Four physically labelled field corners, TL -> TR -> BR -> BL.

    Pixels use top-left/y-down. The field uses bottom-left/y-up in mm.
    No lens undistortion or elevated robot-tag correction is implied.
    """

    image_size_px: tuple[int, int]
    corners_px: tuple[tuple[float, float], ...]
    field_size_mm: tuple[float, float]

    def __post_init__(self) -> None:
        _, np = vision_dependencies()
        if not isinstance(self.image_size_px, (tuple, list)) or len(self.image_size_px) != 2:
            raise CalibrationError("image_size_px must contain width and height")
        if any(type(value) is not int or value < 2 for value in self.image_size_px):
            raise CalibrationError("image dimensions must be integers >= 2")
        if not isinstance(self.field_size_mm, (tuple, list)) or len(self.field_size_mm) != 2:
            raise CalibrationError("field_size_mm must contain width and height")
        size = tuple(_positive_number(value, "field_size_mm") for value in self.field_size_mm)
        corners = _points(self.corners_px)
        if corners.shape != (4, 2):
            raise CalibrationError("exactly four corners are required: TL TR BR BL")
        width, height = self.image_size_px
        if ((corners < 0).any() or (corners[:, 0] > width - 1).any()
                or (corners[:, 1] > height - 1).any()):
            raise CalibrationError("corners must lie inside the calibration image")
        edges = np.roll(corners, -1, axis=0) - corners
        following = np.roll(edges, -1, axis=0)
        cross = edges[:, 0] * following[:, 1] - edges[:, 1] * following[:, 0]
        # Positive cross products are clockwise in image coordinates. Requiring
        # all four excludes crossed, concave, duplicate and collinear corners.
        if (cross <= 1.0).any() or (np.linalg.norm(edges, axis=1) < 2.0).any():
            raise CalibrationError("corners must form a nondegenerate clockwise TL TR BR BL polygon")
        object.__setattr__(self, "image_size_px", tuple(self.image_size_px))
        object.__setattr__(self, "field_size_mm", size)
        object.__setattr__(self, "corners_px", tuple(tuple(float(v) for v in p) for p in corners))
        matrix = self._matrix()
        if not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) < 3:
            raise CalibrationError("singular field transform")

    def _matrix(self):
        cv2, np = vision_dependencies()
        width, height = self.field_size_mm
        target = np.array(((0, height), (width, height), (width, 0), (0, 0)), dtype=np.float32)
        return cv2.getPerspectiveTransform(np.asarray(self.corners_px, dtype=np.float32), target)

    def pixel_to_field_mm(self, points: Any):
        return _project(points, self._matrix())

    def field_mm_to_pixel(self, points: Any):
        _, np = vision_dependencies()
        return _project(points, np.linalg.inv(self._matrix()))

    def _output_geometry(self, pixels_per_mm: float):
        scale = _positive_number(pixels_per_mm, "pixels_per_mm")
        width, height = self.field_size_mm
        if not math.isfinite(width * scale) or not math.isfinite(height * scale):
            raise CalibrationError("rectified image is too large")
        out_width, out_height = math.ceil(width * scale) + 1, math.ceil(height * scale) + 1
        if min(out_width, out_height) < 2 or out_width * out_height > 40_000_000:
            raise CalibrationError("rectified dimensions must be >= 2 with at most 40 million pixels total")
        return out_width, out_height, scale

    def field_mm_to_rectified_px(self, points: Any, pixels_per_mm: float = 1.0):
        _, np = vision_dependencies()
        _, _, scale = self._output_geometry(pixels_per_mm)
        matrix = np.array(((scale, 0, 0), (0, -scale, self.field_size_mm[1] * scale), (0, 0, 1)))
        return _project(points, matrix)

    def warp(self, image: Any, pixels_per_mm: float = 1.0):
        cv2, np = vision_dependencies()
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8
                or image.ndim not in (2, 3)
                or (image.ndim == 3 and image.shape[2] not in (1, 3, 4))):
            raise CalibrationError("expected a uint8 grayscale, BGR or BGRA image")
        if (image.shape[1], image.shape[0]) != self.image_size_px:
            raise CalibrationError("frame resolution does not match calibration; recalibrate")
        width, height, scale = self._output_geometry(pixels_per_mm)
        display = np.array(((scale, 0, 0), (0, -scale, self.field_size_mm[1] * scale), (0, 0, 1)))
        return cv2.warpPerspective(image, display @ self._matrix(), (width, height))

    def check_points(self, pixel_points: Any, expected_mm: Any) -> dict[str, float | int]:
        """Measure independent landmarks, not the four fitting corners."""
        _, np = vision_dependencies()
        pixels, expected = _points(pixel_points), _points(expected_mm)
        if pixels.shape != expected.shape or len(pixels) == 0:
            raise CalibrationError("validation requires matching nonempty N x 2 point arrays")
        errors = np.linalg.norm(self.pixel_to_field_mm(pixels) - expected, axis=1)
        return {"count": len(errors), "rms_error_mm": float(np.sqrt(np.mean(errors ** 2))),
                "max_error_mm": float(errors.max())}

    def as_dict(self) -> dict[str, Any]:
        return {"schema_version": 1, "coordinate_system": COORDINATE_SYSTEM,
                "image_size_px": list(self.image_size_px),
                "field_size_mm": list(self.field_size_mm),
                "corners_px": dict(zip(CORNER_NAMES, self.corners_px))}

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.as_dict(), indent=2, allow_nan=False) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> FieldCalibration:
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
            if not isinstance(data, dict):
                raise CalibrationError("calibration JSON must be an object")
            if type(data.get("schema_version")) is not int or data["schema_version"] != 1:
                raise CalibrationError("unsupported calibration schema_version")
            if data.get("coordinate_system") != COORDINATE_SYSTEM:
                raise CalibrationError("unsupported calibration coordinate_system")
            corners = data["corners_px"]
            if not isinstance(corners, dict) or set(corners) != set(CORNER_NAMES):
                raise CalibrationError("corners_px must name top_left, top_right, bottom_right, bottom_left")
            return cls(data["image_size_px"], tuple(corners[name] for name in CORNER_NAMES), data["field_size_mm"])
        except (KeyError, TypeError, json.JSONDecodeError) as exc:
            raise CalibrationError(f"invalid calibration JSON: {exc}") from exc
