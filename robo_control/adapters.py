from __future__ import annotations

import json
import math
import socket
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from types import TracebackType
from typing import Any, Protocol, Self

from .models import Point


@dataclass(frozen=True, slots=True)
class CameraFrame:
    image: Any
    captured_at_s: float
    sequence: int
    source_name: str
    received_at_s: float | None = None
    media_time_s: float | None = None
    timestamp_basis: str = "host_read_start"
    is_replay: bool = False


class CameraSource(Protocol):
    def read(self) -> CameraFrame | None: ...

    def close(self) -> None: ...


class SyntheticCameraSource:
    """A hardware-free camera source carrying simulator ground truth."""

    def __init__(self, snapshot_provider: Any) -> None:
        self.snapshot_provider = snapshot_provider
        self.sequence = 0

    def read(self) -> CameraFrame:
        self.sequence += 1
        return CameraFrame(
            image=self.snapshot_provider(),
            captured_at_s=time.monotonic(),
            sequence=self.sequence,
            source_name="synthetic-ground-truth",
            timestamp_basis="synthetic_generation",
        )

    def close(self) -> None:
        return None


class OpenCVCameraSource:
    """Optional synchronous USB/RTSP input, opened only when selected.

    Host timestamps bracket ``read()``; they are not sensor exposure timestamps
    and cannot reveal how long a frame waited in a camera/backend buffer.
    A failed read closes the source. Construct a new source to reconnect.
    """

    def __init__(self, source: int | str = 0) -> None:
        try:
            import cv2  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "OpenCV camera mode requires: pip install -e .[vision]"
            ) from exc
        self._cv2 = cv2
        self._capture = cv2.VideoCapture(source)
        try:
            opened = self._capture.isOpened()
        except Exception:
            self._capture.release()
            raise
        if not opened:
            self._capture.release()
            raise RuntimeError(f"camera source could not be opened: {source!r}")
        self.sequence = 0
        self.source_name = f"webcam:{source}" if isinstance(source, int) else f"opencv:{source}"
        self._closed = False
        self._is_replay = False

    def read(self) -> CameraFrame | None:
        if self._closed:
            return None
        captured_at_s = time.monotonic()
        try:
            ok, image = self._capture.read()
        except self._cv2.error:
            self.close()
            return None
        received_at_s = time.monotonic()
        if not ok or image is None:
            self.close()
            return None
        media_time_s = self._media_time_s() if self._is_replay else None
        self.sequence += 1
        return CameraFrame(
            image=image,
            captured_at_s=captured_at_s,
            sequence=self.sequence,
            source_name=self.source_name,
            received_at_s=received_at_s,
            media_time_s=media_time_s,
            timestamp_basis="host_read_start",
            is_replay=self._is_replay,
        )

    def _media_time_s(self) -> float | None:
        try:
            milliseconds = float(self._capture.get(self._cv2.CAP_PROP_POS_MSEC))
        except (TypeError, ValueError, self._cv2.error):
            return None
        return milliseconds / 1000.0 if math.isfinite(milliseconds) and milliseconds >= 0 else None

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._capture.release()

    def __enter__(self) -> Self:
        if self._closed:
            raise RuntimeError("camera source is closed")
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()


class VideoFileSource(OpenCVCameraSource):
    """Replay one existing local video file without pacing or automatic looping.

    ``media_time_s`` is the backend's playback position, not a monotonic host
    timestamp. Unsupported or invalid playback positions are reported as None.
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser().resolve(strict=True)
        if not self.path.is_file():
            raise ValueError(f"video path must be a local file: {self.path}")
        super().__init__(str(self.path))
        self.source_name = f"video:{self.path}"
        self._is_replay = True


@dataclass(frozen=True, slots=True)
class RobotCommand:
    robot_id: str
    sequence: int
    server_time_s: float
    ttl_s: float
    action: str
    target: Point | None = None
    target_heading_rad: float | None = None
    max_linear_speed_mps: float = 0.0
    max_angular_speed_rps: float = 0.0
    gate: str = "hold"
    emergency_stop: bool = False

    def as_payload(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["target"] = self.target.as_dict() if self.target else None
        return payload


class RobotTransport(Protocol):
    def send(self, command: RobotCommand) -> None: ...

    def close(self) -> None: ...


class DryRunTransport:
    """Safe default: records commands in memory and controls no physical device."""

    def __init__(self) -> None:
        self.commands: list[RobotCommand] = []

    def send(self, command: RobotCommand) -> None:
        self.commands.append(command)
        if len(self.commands) > 1000:
            del self.commands[:500]

    def close(self) -> None:
        return None


class UdpRobotTransport:
    """High-level JSON/UDP transport for a future ESP32 gateway.

    This adapter deliberately does not define motor PWM values. Firmware should
    validate robot_id, monotonically increasing sequence, and TTL before acting.
    """

    def __init__(self, endpoints: dict[str, tuple[str, int]]) -> None:
        self.endpoints = dict(endpoints)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, command: RobotCommand) -> None:
        endpoint = self.endpoints.get(command.robot_id)
        if endpoint is None:
            raise KeyError(f"no UDP endpoint configured for {command.robot_id}")
        encoded = json.dumps(command.as_payload(), separators=(",", ":")).encode("utf-8")
        self.socket.sendto(encoded, endpoint)

    def close(self) -> None:
        self.socket.close()


class DroneController(Protocol):
    def takeoff(self) -> None: ...

    def hold(self) -> None: ...

    def land(self) -> None: ...

    def emergency_stop(self) -> None: ...


class DisabledDroneController:
    """Default fail-closed controller used until the real controller is measured."""

    def _disabled(self) -> None:
        raise RuntimeError("drone output is disabled; controller protocol is unverified")

    takeoff = hold = land = emergency_stop = _disabled


class JsonlLogger:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, record: dict[str, Any]) -> None:
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")))
            handle.write("\n")

