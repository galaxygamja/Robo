"""Pinned-resolution Brown-Conrady lens calibration, not a fisheye model.

Public pixels are always RAW camera pixels. Ideal pixels are an internal
intermediate with the same camera matrix; no cropping or rescaling is applied.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from .calibration import CalibrationError, _points, vision_dependencies

MODEL = "opencv_pinhole_brown5"


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


@dataclass(frozen=True, slots=True)
class LensCalibration:
    image_size_px: tuple[int, int]
    camera_matrix: tuple[tuple[float, float, float], ...]
    distortion_coefficients: tuple[float, ...]
    camera_label: str

    def __post_init__(self):
        if (not isinstance(self.image_size_px, (tuple, list)) or len(self.image_size_px) != 2
                or any(type(v) is not int or v < 2 for v in self.image_size_px)
                or max(self.image_size_px) >= 32767
                or math.prod(self.image_size_px) > 16_000_000):
            raise CalibrationError("Lens resolution requires integer width/height and at most 16 MP")
        matrix = self.camera_matrix
        if (not isinstance(matrix, (tuple, list)) or len(matrix) != 3
                or any(not isinstance(row, (tuple, list)) or len(row) != 3 for row in matrix)
                or any(not _finite(v) for row in matrix for v in row)):
            raise CalibrationError("camera_matrix must be a finite 3 x 3 matrix")
        width, height = self.image_size_px
        if (not 1 <= matrix[0][0] <= 1e6 or not 1 <= matrix[1][1] <= 1e6
                or not 0 <= matrix[0][2] < width or not 0 <= matrix[1][2] < height
                or matrix[0][1] != 0 or matrix[1][0] != 0 or tuple(matrix[2]) != (0, 0, 1)):
            raise CalibrationError("Require positive focal lengths, zero skew and principal point inside image")
        coefficients = self.distortion_coefficients
        if (not isinstance(coefficients, (tuple, list)) or len(coefficients) != 5
                or any(not _finite(v) or abs(v) > 10 for v in coefficients)):
            raise CalibrationError("Brown5 requires finite [k1,k2,p1,p2,k3] coefficients in [-10,10]")
        if not isinstance(self.camera_label, str) or not self.camera_label.strip() or len(self.camera_label) > 200:
            raise CalibrationError("Identify the physical camera/lens/settings with a nonempty label <= 200 characters")
        object.__setattr__(self, "image_size_px", tuple(self.image_size_px))
        object.__setattr__(self, "camera_matrix", tuple(tuple(float(v) for v in row) for row in matrix))
        object.__setattr__(self, "distortion_coefficients", tuple(float(v) for v in coefficients))
        self._validate_domain()

    @property
    def fingerprint(self):
        payload = json.dumps(self.as_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False)
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def distort_points(self, ideal_pixels):
        """Ideal same-K pixels -> raw pixels; the inverse of undistort_points."""
        _, np = vision_dependencies()
        points = _points(ideal_pixels)
        k = self.camera_matrix
        x, y = (points[:, 0] - k[0][2]) / k[0][0], (points[:, 1] - k[1][2]) / k[1][1]
        k1, k2, p1, p2, k3 = self.distortion_coefficients
        with np.errstate(over="ignore", invalid="ignore"):
            r2 = x * x + y * y
            radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
            xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
            result = np.column_stack((xd * k[0][0] + k[0][2], yd * k[1][1] + k[1][2]))
        if not np.isfinite(result).all():
            raise CalibrationError("Lens projection overflow")
        return result

    def undistort_points(self, raw_pixels):
        """Raw image pixels -> ideal same-K pixels, with inverse residual check."""
        cv2, np = vision_dependencies()
        points = _points(raw_pixels)
        if not len(points):
            return points.copy()
        width, height = self.image_size_px
        if ((points < 0).any() or (points[:, 0] > width - 1).any() or (points[:, 1] > height - 1).any()):
            raise CalibrationError("Lens correction requires raw pixels inside the calibrated image")
        k, d = np.asarray(self.camera_matrix), np.asarray(self.distortion_coefficients)
        criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-10)
        if hasattr(cv2, "undistortPointsIter"):  # OpenCV 4.x
            result = cv2.undistortPointsIter(points.reshape(-1, 1, 2), k, d, None, k, criteria)
        else:  # OpenCV 5 folded the iterative overload into undistortPoints.
            result = cv2.undistortPoints(points.reshape(-1, 1, 2), k, d, R=None, P=k, criteria=criteria)
        result = result.reshape(-1, 2)
        if (not np.isfinite(result).all()
                or (np.linalg.norm(self.distort_points(result) - points, axis=1) > .05).any()):
            raise CalibrationError("Lens inverse did not converge within 0.05 px; profile/domain is invalid")
        return result

    def _validate_domain(self):
        """Reject obvious folded/ill-conditioned fits; not a physical calibration certificate."""
        _, np = vision_dependencies()
        width, height = self.image_size_px
        raw = np.array([(x, y) for y in np.linspace(0, height - 1, 9)
                        for x in np.linspace(0, width - 1, 13)])
        ideal = self.undistort_points(raw)
        k = self.camera_matrix
        x, y = (ideal[:, 0] - k[0][2]) / k[0][0], (ideal[:, 1] - k[1][2]) / k[1][1]
        q = x * x + y * y
        k1, k2, p1, p2, k3 = self.distortion_coefficients
        # Exact extrema of the radial derivative on the sampled radius domain.
        candidates = [0., float(q.max())]
        for root in np.roots([21 * k3, 10 * k2, 3 * k1]):
            if abs(root.imag) < 1e-10 and 0 < root.real < candidates[1]:
                candidates.append(float(root.real))
        if min(1 + 3 * k1 * v + 5 * k2 * v**2 + 7 * k3 * v**3 for v in candidates) <= .05:
            raise CalibrationError("Lens radial mapping folds or is nearly singular")
        radial, slope = 1 + k1 * q + k2 * q**2 + k3 * q**3, k1 + 2 * k2 * q + 3 * k3 * q**2
        jxx = radial + 2 * x*x * slope + 2 * p1 * y + 6 * p2 * x
        jyy = radial + 2 * y*y * slope + 6 * p1 * y + 2 * p2 * x
        jxy = 2 * x*y * slope + 2 * p1 * x + 2 * p2 * y
        if (jxx * jyy - jxy*jxy <= .01).any():
            raise CalibrationError("Lens tangential mapping folds or is nearly singular")

    def undistort_image(self, image):
        cv2, np = vision_dependencies()
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim not in (2, 3)
                or (image.ndim == 3 and image.shape[2] not in (1, 3, 4))
                or (image.shape[1], image.shape[0]) != self.image_size_px):
            raise CalibrationError("Lens image must be uint8 and match the exact calibrated resolution")
        maps = _rectification_maps(self)
        return cv2.remap(image, *maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def as_dict(self):
        return {"schema_version": 1, "model": MODEL, "camera_label": self.camera_label,
                "image_size_px": list(self.image_size_px), "camera_matrix": [list(row) for row in self.camera_matrix],
                "distortion_coefficients": list(self.distortion_coefficients)}

    @classmethod
    def from_dict(cls, data):
        keys = {"schema_version", "model", "camera_label", "image_size_px", "camera_matrix", "distortion_coefficients"}
        if (not isinstance(data, dict) or set(data) != keys or type(data.get("schema_version")) is not int
                or data["schema_version"] != 1 or data.get("model") != MODEL):
            raise CalibrationError("Unsupported lens schema/model/fields; use opencv_pinhole_brown5 schema 1")
        return cls(**{key: data[key] for key in keys - {"schema_version", "model"}})

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8-sig")))

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as handle:
            handle.write(json.dumps(self.as_dict(), indent=2, allow_nan=False) + "\n")


@lru_cache(maxsize=2)
def _rectification_maps(lens):
    cv2, np = vision_dependencies()
    k, d = np.asarray(lens.camera_matrix), np.asarray(lens.distortion_coefficients)
    maps = cv2.initUndistortRectifyMap(k, d, None, k, lens.image_size_px, cv2.CV_32FC1)
    for value in maps:
        value.flags.writeable = False
    return maps
