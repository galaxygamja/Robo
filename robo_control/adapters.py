from __future__ import annotations

import json
import socket
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Protocol

from .models import Point


@dataclass(frozen=True, slots=True)
class CameraFrame:
    image: Any
    captured_at_s: float
    sequence: int
    source_name: str


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
        )

    def close(self) -> None:
        return None


class OpenCVCameraSource:
    """Optional USB/RTSP adapter. It is never opened unless explicitly selected."""

    def __init__(self, source: int | str = 0) -> None:
        try:
            import cv2  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "OpenCV camera mode requires: pip install -e .[vision]"
            ) from exc
        self._cv2 = cv2
        self._capture = cv2.VideoCapture(source)
        if not self._capture.isOpened():
            self._capture.release()
            raise RuntimeError(f"camera source could not be opened: {source!r}")
        self.sequence = 0

    def read(self) -> CameraFrame | None:
        ok, image = self._capture.read()
        if not ok:
            return None
        self.sequence += 1
        return CameraFrame(image, time.monotonic(), self.sequence, "opencv")

    def close(self) -> None:
        self._capture.release()


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

