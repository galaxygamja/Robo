"""Reject unusable observations before they can become control inputs."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from ..adapters import CameraFrame
from .calibration import CalibrationError, FieldCalibration, _positive_number


class FrameRejected(ValueError):
    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)


@dataclass(frozen=True, slots=True)
class CalibratedFrame:
    image: Any
    original: CameraFrame
    processed_at_s: float
    age_s: float


class FrameProcessor:
    """One source/session per processor; host monotonic time, not match time."""

    def __init__(self, calibration: FieldCalibration, max_age_s: float = 0.2,
                 pixels_per_mm: float = 1.0, clock: Callable[[], float] = time.monotonic) -> None:
        self.calibration = calibration
        self.max_age_s = _positive_number(max_age_s, "max_age_s")
        self.pixels_per_mm = _positive_number(pixels_per_mm, "pixels_per_mm")
        self.clock = clock
        self._source_name: str | None = None
        self._last_sequence = 0
        self._last_timestamp = -math.inf
        self._active: tuple[str, int, float] | None = None

    def _age(self, frame: CameraFrame, now: float) -> float:
        received = frame.received_at_s if frame.received_at_s is not None else frame.captured_at_s
        stamps = (frame.captured_at_s, received, now)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in stamps):
            raise FrameRejected("invalid_timestamp")
        if not frame.captured_at_s <= received <= now:
            raise FrameRejected("timestamp_order")
        age = now - frame.captured_at_s
        if age > self.max_age_s:
            raise FrameRejected("stale_frame")
        return age

    def begin_frame(self, frame: CameraFrame) -> float:
        """Validate and consume frame ordering before downstream image work."""
        if self._active is not None:
            raise RuntimeError("finish or abandon the active frame before beginning another")
        before = self.clock()
        self._age(frame, before)
        if not isinstance(frame.source_name, str) or not frame.source_name:
            raise FrameRejected("invalid_source")
        if self._source_name is not None and frame.source_name != self._source_name:
            raise FrameRejected("source_changed")
        if type(frame.sequence) is not int or frame.sequence <= self._last_sequence:
            raise FrameRejected("out_of_order_sequence")
        if frame.captured_at_s < self._last_timestamp:
            raise FrameRejected("out_of_order_timestamp")
        # Consume even rejected frames, so they cannot be retried as new input.
        self._source_name, self._last_sequence = frame.source_name, frame.sequence
        self._last_timestamp = frame.captured_at_s
        self._active = (frame.source_name, frame.sequence, before)
        return before

    def finish_frame(self, frame: CameraFrame) -> tuple[float, float]:
        """Recheck age after downstream work and return completion time and age."""
        if self._active is None or self._active[:2] != (frame.source_name, frame.sequence):
            raise RuntimeError("frame is not the active consumed frame")
        before = self._active[2]
        try:
            after = self.clock()
            age = self._age(frame, after)
            if after < before:
                raise FrameRejected("clock_moved_backwards")
            return after, age
        finally:
            self._active = None

    def abandon_frame(self) -> None:
        """End downstream work after an error; sequence remains consumed."""
        self._active = None

    def process(self, frame: CameraFrame) -> CalibratedFrame:
        self.begin_frame(frame)
        try:
            image = self.calibration.warp(frame.image, self.pixels_per_mm)
        except CalibrationError as exc:
            self.abandon_frame()
            raise FrameRejected(f"invalid_image: {exc}") from exc
        except Exception:
            self.abandon_frame()
            raise
        after, age = self.finish_frame(frame)
        return CalibratedFrame(image, frame, after, age)
