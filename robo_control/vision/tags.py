"""AprilTag robot observations from real images.

Detection happens in the original camera image to avoid rectification blur. Tag
corners are then projected onto the calibrated field plane in millimetres.
"""

from __future__ import annotations

import json
import math
import re
from collections import Counter
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any

from ..adapters import CameraFrame
from .calibration import (
    COORDINATE_SYSTEM,
    CalibrationError,
    FieldCalibration,
    vision_dependencies,
)

SUPPORTED_DICTIONARIES = (
    "DICT_APRILTAG_16h5",
    "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10",
    "DICT_APRILTAG_36h11",
)
MARKER_FORWARD = "canonical_corner_0_to_1"
_ROBOT_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{0,31}$")
_TAG_ID = re.compile(r"^(0|[1-9][0-9]*)$")


class TagDetectionError(ValueError):
    def __init__(self, reason: str, message: str | None = None) -> None:
        self.reason = reason
        super().__init__(message or reason)


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise TagDetectionError("invalid_config", f"{name} must be a finite number")
    return float(value)


def _normalize_angle(angle: float) -> float:
    normalized = (angle + math.pi) % (2 * math.pi) - math.pi
    return 0.0 if normalized == 0 else normalized


@dataclass(frozen=True, slots=True)
class TagDetectorConfig:
    dictionary_name: str = "DICT_APRILTAG_36h11"
    tag_to_robot: Mapping[int, str] = field(
        default_factory=lambda: {0: "H1", 1: "H2", 2: "B1", 3: "B2"}
    )
    heading_offsets_rad: Mapping[str, float] = field(default_factory=dict)
    robot_center_from_tag_mm: Mapping[str, tuple[float, float]] = field(default_factory=dict)
    allowed_margin_mm: float = 0.0
    tag_size_mm: float | None = None
    tag_size_tolerance_fraction: float = 0.35
    hardware_verified: bool = False

    def __post_init__(self) -> None:
        if self.dictionary_name not in SUPPORTED_DICTIONARIES:
            raise TagDetectionError(
                "invalid_config",
                f"dictionary_name must be one of: {', '.join(SUPPORTED_DICTIONARIES)}",
            )
        if not isinstance(self.tag_to_robot, Mapping) or not self.tag_to_robot:
            raise TagDetectionError("invalid_config", "tag_to_robot must not be empty")
        mapping: dict[int, str] = {}
        for tag_id, robot_id in self.tag_to_robot.items():
            if type(tag_id) is not int or tag_id < 0:
                raise TagDetectionError("invalid_config", "tag IDs must be nonnegative integers")
            if not isinstance(robot_id, str) or not _ROBOT_ID.fullmatch(robot_id):
                raise TagDetectionError("invalid_config", f"invalid robot ID for tag {tag_id}")
            mapping[tag_id] = robot_id
        if len(set(mapping.values())) != len(mapping):
            raise TagDetectionError("invalid_config", "each robot ID must map to exactly one tag")
        if not isinstance(self.heading_offsets_rad, Mapping):
            raise TagDetectionError("invalid_config", "heading_offsets_rad must be an object")
        offsets: dict[str, float] = {}
        for robot_id, value in self.heading_offsets_rad.items():
            if robot_id not in mapping.values():
                raise TagDetectionError("invalid_config", f"heading offset names unknown robot: {robot_id}")
            offsets[robot_id] = _normalize_angle(_finite(value, f"heading offset for {robot_id}"))
        if not isinstance(self.robot_center_from_tag_mm, Mapping):
            raise TagDetectionError("invalid_config", "robot_center_from_tag_mm must be an object")
        center_offsets: dict[str, tuple[float, float]] = {}
        for robot_id, value in self.robot_center_from_tag_mm.items():
            if robot_id not in mapping.values():
                raise TagDetectionError("invalid_config", f"centre offset names unknown robot: {robot_id}")
            if not isinstance(value, (list, tuple)) or len(value) != 2:
                raise TagDetectionError("invalid_config", f"centre offset for {robot_id} must be [forward_mm, left_mm]")
            center_offsets[robot_id] = (
                _finite(value[0], f"forward centre offset for {robot_id}"),
                _finite(value[1], f"left centre offset for {robot_id}"),
            )
        margin = _finite(self.allowed_margin_mm, "allowed_margin_mm")
        if margin < 0:
            raise TagDetectionError("invalid_config", "allowed_margin_mm must be >= 0")
        size = self.tag_size_mm
        if size is not None:
            size = _finite(size, "tag_size_mm")
            if size <= 0:
                raise TagDetectionError("invalid_config", "tag_size_mm must be > 0")
        tolerance = _finite(self.tag_size_tolerance_fraction, "tag_size_tolerance_fraction")
        if not 0 < tolerance <= 1:
            raise TagDetectionError("invalid_config", "tag_size_tolerance_fraction must be in (0, 1]")
        if type(self.hardware_verified) is not bool:
            raise TagDetectionError("invalid_config", "hardware_verified must be a boolean")
        object.__setattr__(self, "tag_to_robot", MappingProxyType(dict(sorted(mapping.items()))))
        object.__setattr__(self, "heading_offsets_rad", MappingProxyType(dict(sorted(offsets.items()))))
        object.__setattr__(self, "robot_center_from_tag_mm", MappingProxyType(dict(sorted(center_offsets.items()))))
        object.__setattr__(self, "allowed_margin_mm", margin)
        object.__setattr__(self, "tag_size_mm", size)
        object.__setattr__(self, "tag_size_tolerance_fraction", tolerance)

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "dictionary_name": self.dictionary_name,
            "tag_to_robot": {str(key): value for key, value in self.tag_to_robot.items()},
            "heading_offsets_rad": dict(self.heading_offsets_rad),
            "robot_center_from_tag_mm": {
                key: {"forward_mm": value[0], "left_mm": value[1]}
                for key, value in self.robot_center_from_tag_mm.items()
            },
            "marker_forward": MARKER_FORWARD,
            "coordinate_system": COORDINATE_SYSTEM,
            "allowed_margin_mm": self.allowed_margin_mm,
            "tag_size_mm": self.tag_size_mm,
            "tag_size_tolerance_fraction": self.tag_size_tolerance_fraction,
            "hardware_verified": self.hardware_verified,
        }

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.as_dict(), indent=2, allow_nan=False) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> TagDetectorConfig:
        try:
            raw = json.loads(Path(path).read_text(encoding="utf-8-sig"))
            if not isinstance(raw, dict):
                raise TagDetectionError("invalid_config", "tag JSON must be an object")
            if type(raw.get("schema_version")) is not int or raw["schema_version"] != 1:
                raise TagDetectionError("invalid_config", "unsupported tag schema_version")
            if raw.get("marker_forward", MARKER_FORWARD) != MARKER_FORWARD:
                raise TagDetectionError("invalid_config", "unsupported marker_forward convention")
            if raw.get("coordinate_system", COORDINATE_SYSTEM) != COORDINATE_SYSTEM:
                raise TagDetectionError("invalid_config", "unsupported coordinate_system")
            encoded_mapping = raw["tag_to_robot"]
            if not isinstance(encoded_mapping, dict):
                raise TagDetectionError("invalid_config", "tag_to_robot must be an object")
            mapping: dict[int, str] = {}
            for key, value in encoded_mapping.items():
                if not isinstance(key, str) or not _TAG_ID.fullmatch(key):
                    raise TagDetectionError("invalid_config", f"invalid JSON tag ID: {key!r}")
                mapping[int(key)] = value
            encoded_centers = raw.get("robot_center_from_tag_mm", {})
            if not isinstance(encoded_centers, dict):
                raise TagDetectionError("invalid_config", "robot_center_from_tag_mm must be an object")
            centers: dict[str, tuple[Any, Any]] = {}
            for robot_id, value in encoded_centers.items():
                if not isinstance(value, dict) or set(value) != {"forward_mm", "left_mm"}:
                    raise TagDetectionError(
                        "invalid_config",
                        f"robot_center_from_tag_mm.{robot_id} requires forward_mm and left_mm",
                    )
                centers[robot_id] = (value["forward_mm"], value["left_mm"])
            return cls(
                dictionary_name=raw["dictionary_name"],
                tag_to_robot=mapping,
                heading_offsets_rad=raw.get("heading_offsets_rad", {}),
                robot_center_from_tag_mm=centers,
                allowed_margin_mm=raw.get("allowed_margin_mm", 0.0),
                tag_size_mm=raw.get("tag_size_mm"),
                tag_size_tolerance_fraction=raw.get("tag_size_tolerance_fraction", 0.35),
                hardware_verified=raw.get("hardware_verified", False),
            )
        except (KeyError, TypeError, json.JSONDecodeError) as exc:
            raise TagDetectionError("invalid_config", f"invalid tag JSON: {exc}") from exc


@dataclass(frozen=True, slots=True)
class RobotTagObservation:
    robot_id: str
    tag_id: int
    tag_center_px: tuple[float, float]
    tag_center_mm: tuple[float, float]
    robot_center_px: tuple[float, float]
    robot_center_mm: tuple[float, float]
    heading_rad: float
    observed_tag_size_mm: float
    corners_px: tuple[tuple[float, float], ...]
    corners_mm: tuple[tuple[float, float], ...]
    captured_at_s: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "tag_id": self.tag_id,
            "tag_center_px": list(self.tag_center_px),
            "tag_center_mm": list(self.tag_center_mm),
            "robot_center_px": list(self.robot_center_px),
            "robot_center_mm": list(self.robot_center_mm),
            "heading_rad": self.heading_rad,
            "observed_tag_size_mm": self.observed_tag_size_mm,
            "corners_px": [list(point) for point in self.corners_px],
            "corners_mm": [list(point) for point in self.corners_mm],
            "captured_at_s": self.captured_at_s,
        }


@dataclass(frozen=True, slots=True)
class TagRejection:
    tag_id: int | None
    reason: str
    corners_px: tuple[tuple[float, float], ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return {"tag_id": self.tag_id, "reason": self.reason,
                "corners_px": [list(point) for point in self.corners_px]}


@dataclass(frozen=True, slots=True)
class TagDetectionBatch:
    frame_sequence: int
    source_name: str
    captured_at_s: float
    observations: tuple[RobotTagObservation, ...]
    unknown_tag_ids: tuple[int, ...]
    duplicate_tag_ids: tuple[int, ...]
    missing_robot_ids: tuple[str, ...]
    rejected: tuple[TagRejection, ...]
    rejected_candidate_count: int
    raw_detection_count: int
    observation_complete: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "frame_sequence": self.frame_sequence,
            "source_name": self.source_name,
            "captured_at_s": self.captured_at_s,
            "robots": [item.as_dict() for item in self.observations],
            "unknown_tag_ids": list(self.unknown_tag_ids),
            "duplicate_tag_ids": list(self.duplicate_tag_ids),
            "missing_robot_ids": list(self.missing_robot_ids),
            "rejected": [item.as_dict() for item in self.rejected],
            "rejected_candidate_count": self.rejected_candidate_count,
            "raw_detection_count": self.raw_detection_count,
            "observation_complete": self.observation_complete,
        }


class AprilTagDetector:
    """Detect configured robot tags without retaining state across frames.

    Marker local +X, canonical corner 0 toward corner 1, is robot forward before
    its configured mounting offset. Tracking and tag-loss timeout are separate.
    """

    def __init__(self, config: TagDetectorConfig, calibration: FieldCalibration) -> None:
        cv2, _ = vision_dependencies()
        self.config = config
        self.calibration = calibration
        dictionary_id = getattr(cv2.aruco, config.dictionary_name, None)
        if dictionary_id is None:
            raise TagDetectionError("unsupported_dictionary", f"OpenCV lacks {config.dictionary_name}")
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        count = int(self.dictionary.bytesList.shape[0])
        if any(tag_id >= count for tag_id in config.tag_to_robot):
            raise TagDetectionError("invalid_config", f"tag ID exceeds {config.dictionary_name} range 0..{count - 1}")
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(self.dictionary, parameters)

    def _image(self, frame: CameraFrame):
        cv2, np = vision_dependencies()
        image = frame.image
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            raise TagDetectionError("invalid_image", "tag input must be a uint8 image")
        if image.ndim == 2:
            gray = image
        elif image.ndim == 3 and image.shape[2] == 1:
            gray = image[:, :, 0]
        elif image.ndim == 3 and image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif image.ndim == 3 and image.shape[2] == 4:
            gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        else:
            raise TagDetectionError("invalid_image", "expected grayscale, BGR or BGRA image")
        if (gray.shape[1], gray.shape[0]) != self.calibration.image_size_px:
            raise TagDetectionError("resolution_mismatch", "frame resolution differs from calibration")
        if type(frame.sequence) is not int or frame.sequence < 1:
            raise TagDetectionError("invalid_frame", "frame sequence must be a positive integer")
        if not isinstance(frame.source_name, str) or not frame.source_name:
            raise TagDetectionError("invalid_frame", "frame source_name must not be empty")
        if (isinstance(frame.captured_at_s, bool) or not isinstance(frame.captured_at_s, (int, float))
                or not math.isfinite(frame.captured_at_s)):
            raise TagDetectionError("invalid_frame", "frame captured_at_s must be finite")
        return gray

    def detect(self, frame: CameraFrame) -> TagDetectionBatch:
        _, np = vision_dependencies()
        gray = self._image(frame)
        try:
            detected_corners, detected_ids, rejected_candidates = self._detector.detectMarkers(gray)
        except Exception as exc:
            raise TagDetectionError("opencv_detection_failed", str(exc)) from exc
        ids = [] if detected_ids is None else [int(value) for value in np.asarray(detected_ids).reshape(-1)]
        if len(ids) != len(detected_corners):
            raise TagDetectionError("opencv_result_mismatch", "OpenCV returned mismatched IDs and corners")
        counts = Counter(ids)
        duplicates = tuple(sorted(tag_id for tag_id, count in counts.items() if count > 1))
        unknown = tuple(sorted({tag_id for tag_id in ids if tag_id not in self.config.tag_to_robot}))
        observations: list[RobotTagObservation] = []
        rejections: list[TagRejection] = []
        width_mm, height_mm = self.calibration.field_size_mm
        margin = self.config.allowed_margin_mm
        for tag_id, raw in zip(ids, detected_corners):
            corners_array = np.asarray(raw, dtype=np.float64).reshape(-1, 2)
            if corners_array.shape != (4, 2) or not np.isfinite(corners_array).all():
                rejections.append(TagRejection(tag_id, "invalid_corners"))
                continue
            corners_px = tuple(tuple(float(value) for value in point) for point in corners_array)
            if tag_id in duplicates:
                rejections.append(TagRejection(tag_id, "duplicate_tag", corners_px))
                continue
            if tag_id not in self.config.tag_to_robot:
                rejections.append(TagRejection(tag_id, "unknown_tag", corners_px))
                continue
            try:
                corners_mm_array = self.calibration.pixel_to_field_mm(corners_array)
                tag_center_mm_array = corners_mm_array.mean(axis=0)
                # In a perspective image the arithmetic mean of four corner
                # pixels is not the projection of the physical square centre.
                tag_center_px_array = self.calibration.field_mm_to_pixel((tag_center_mm_array,))[0]
            except CalibrationError:
                rejections.append(TagRejection(tag_id, "invalid_projection", corners_px))
                continue
            direction = corners_mm_array[1] - corners_mm_array[0]
            if not np.isfinite(direction).all() or float(np.linalg.norm(direction)) < 1e-6:
                rejections.append(TagRejection(tag_id, "degenerate_heading", corners_px))
                continue
            robot_id = self.config.tag_to_robot[tag_id]
            heading = _normalize_angle(
                math.atan2(float(direction[1]), float(direction[0]))
                + self.config.heading_offsets_rad.get(robot_id, 0.0)
            )
            edge_lengths = np.linalg.norm(np.roll(corners_mm_array, -1, axis=0) - corners_mm_array, axis=1)
            observed_size = float(edge_lengths.mean())
            if (not math.isfinite(observed_size) or observed_size <= 0):
                rejections.append(TagRejection(tag_id, "invalid_projection", corners_px))
                continue
            if (self.config.tag_size_mm is not None
                    and (np.abs(edge_lengths - self.config.tag_size_mm)
                         > self.config.tag_size_mm * self.config.tag_size_tolerance_fraction).any()):
                rejections.append(TagRejection(tag_id, "tag_size_mismatch", corners_px))
                continue
            forward_mm, left_mm = self.config.robot_center_from_tag_mm.get(robot_id, (0.0, 0.0))
            center_mm_array = tag_center_mm_array + np.asarray((
                forward_mm * math.cos(heading) - left_mm * math.sin(heading),
                forward_mm * math.sin(heading) + left_mm * math.cos(heading),
            ))
            x_mm, y_mm = (float(center_mm_array[0]), float(center_mm_array[1]))
            if not (-margin <= x_mm <= width_mm + margin and -margin <= y_mm <= height_mm + margin):
                rejections.append(TagRejection(tag_id, "out_of_field", corners_px))
                continue
            try:
                center_px_array = self.calibration.field_mm_to_pixel((center_mm_array,))[0]
            except CalibrationError:
                rejections.append(TagRejection(tag_id, "invalid_projection", corners_px))
                continue
            observations.append(RobotTagObservation(
                robot_id=robot_id,
                tag_id=tag_id,
                tag_center_px=tuple(float(value) for value in tag_center_px_array),
                tag_center_mm=tuple(float(value) for value in tag_center_mm_array),
                robot_center_px=tuple(float(value) for value in center_px_array),
                robot_center_mm=(x_mm, y_mm),
                heading_rad=heading,
                observed_tag_size_mm=observed_size,
                corners_px=corners_px,
                corners_mm=tuple(tuple(float(value) for value in point) for point in corners_mm_array),
                captured_at_s=float(frame.captured_at_s),
            ))
        observations.sort(key=lambda item: item.tag_id)
        rejections.sort(key=lambda item: (-1 if item.tag_id is None else item.tag_id, item.reason))
        observed_robots = {item.robot_id for item in observations}
        missing = tuple(sorted(set(self.config.tag_to_robot.values()) - observed_robots))
        observation_complete = (
            len(observations) == len(self.config.tag_to_robot)
            and not unknown and not duplicates and not rejections and not missing
        )
        return TagDetectionBatch(
            frame_sequence=frame.sequence,
            source_name=frame.source_name,
            captured_at_s=float(frame.captured_at_s),
            observations=tuple(observations),
            unknown_tag_ids=unknown,
            duplicate_tag_ids=duplicates,
            missing_robot_ids=missing,
            rejected=tuple(rejections),
            rejected_candidate_count=len(rejected_candidates),
            raw_detection_count=len(ids),
            observation_complete=observation_complete,
        )

    def annotate(self, image: Any, batch: TagDetectionBatch):
        """Return a copy with accepted tags, headings and rejected tag outlines."""
        cv2, np = vision_dependencies()
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            raise TagDetectionError("invalid_image", "annotation input must be uint8")
        if image.ndim == 2:
            output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif image.ndim == 3 and image.shape[2] == 1:
            output = cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
        elif image.ndim == 3 and image.shape[2] == 3:
            output = image.copy()
        elif image.ndim == 3 and image.shape[2] == 4:
            output = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        else:
            raise TagDetectionError("invalid_image", "annotation expects grayscale, BGR or BGRA")
        for item in batch.rejected:
            if item.corners_px:
                polygon = np.rint(item.corners_px).astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(output, [polygon], True, (0, 0, 255), 2)
        for observation in batch.observations:
            polygon = np.rint(observation.corners_px).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(output, [polygon], True, (0, 200, 0), 2)
            center_mm = np.asarray(observation.robot_center_mm)
            arrow_mm = center_mm + 75.0 * np.asarray((math.cos(observation.heading_rad),
                                                     math.sin(observation.heading_rad)))
            center, target = self.calibration.field_mm_to_pixel((center_mm, arrow_mm))
            center = tuple(np.rint(center).astype(int))
            target = tuple(np.rint(target).astype(int))
            cv2.circle(output, center, 4, (255, 100, 0), -1)
            cv2.arrowedLine(output, center, target, (255, 100, 0), 2, tipLength=0.25)
            cv2.putText(output, f"{observation.robot_id} #{observation.tag_id}",
                        (center[0] + 5, center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 200, 0), 2)
        return output


def default_tag_config_path() -> Path:
    source = Path(__file__).resolve().parents[2] / "config" / "robot_tags.json"
    return source if source.is_file() else Path(__file__).resolve().parents[1] / "data" / "robot_tags.json"
