"""Shared real-image detection path for the inspection CLI and live runtime."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

from ..adapters import CameraFrame
from .calibration import COORDINATE_SYSTEM, FieldCalibration
from .colors import ColorDetector
from .pipeline import FrameProcessor, FrameRejected
from .tags import (
    AprilTagDetector,
    TagDetectionBatch,
    TagDetectionError,
    TagDetectorConfig,
)


@dataclass(frozen=True)
class DetectionResult:
    record: dict
    batch: TagDetectionBatch | None


class DetectionPipeline:
    """Consume one source session, measuring host age before AND after CV work.

    No rendering, tracking, logging, transport or simulated pose updates happen
    here. The caller can therefore run this entire blocking path in a worker.
    """

    def __init__(self, calibration: FieldCalibration, tags: TagDetectorConfig, *,
                 max_age_s: float = 0.2, colors: ColorDetector | None = None,
                 registry_checked: bool = False,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.calibration, self.tags = calibration, tags
        self.detector = AprilTagDetector(tags, calibration)
        self.processor = FrameProcessor(calibration, max_age_s, clock=clock)
        self.colors, self.registry_checked = colors, registry_checked

    def process(self, frame: CameraFrame) -> DetectionResult:
        record = {
            "sequence": frame.sequence, "source_name": frame.source_name,
            "captured_at_s": frame.captured_at_s, "received_at_s": frame.received_at_s,
            "media_time_s": frame.media_time_s, "timestamp_basis": frame.timestamp_basis,
            "is_replay": frame.is_replay, "dictionary_name": self.tags.dictionary_name,
            "tag_size_mm": self.tags.tag_size_mm, "hardware_verified": self.tags.hardware_verified,
            "coordinate_system": COORDINATE_SYSTEM,
            "field_size_mm": list(self.calibration.field_size_mm),
            "registered_robot_ids": sorted(self.tags.tag_to_robot.values()),
            "mission_registry_checked": self.registry_checked, "device_io": False,
            "lens_correction_applied": self.calibration.lens is not None,
            "lens_calibration_id": self.calibration.lens.fingerprint if self.calibration.lens is not None else None,
        }
        try:
            self.processor.begin_frame(frame)
            batch = self.detector.detect(frame)
            objects = self.colors.detect(frame, batch.observations) if self.colors else []
            processed, age = self.processor.finish_frame(frame)
        except (FrameRejected, TagDetectionError) as exc:
            self.processor.abandon_frame()
            record.update(status="rejected_frame", reason=exc.reason)
            return DetectionResult(record, None)
        except Exception:
            self.processor.abandon_frame()
            raise
        record.update(status="detected", processed_at_s=processed, host_age_ms=age * 1000.0,
                      objects=objects, **batch.as_dict())
        return DetectionResult(record, batch)
