from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import hypot
from typing import Any


class MissionStatus(str, Enum):
    READY = "ready"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    TIMEOUT = "timeout"
    SAFETY_STOP = "safety_stop"


class RobotStatus(str, Enum):
    IDLE = "idle"
    WAITING = "waiting"
    MOVING = "moving"
    ARRIVED = "arrived"
    STOPPED = "stopped"
    LOST = "lost"


@dataclass(frozen=True, slots=True)
class Point:
    x: float
    y: float

    def distance_to(self, other: "Point") -> float:
        return hypot(self.x - other.x, self.y - other.y)

    def as_dict(self) -> dict[str, float]:
        return {"x": round(self.x, 4), "y": round(self.y, 4)}


@dataclass(frozen=True, slots=True)
class Rectangle:
    x: float
    y: float
    width: float
    height: float

    def contains(self, point: Point, padding: float = 0.0) -> bool:
        return (
            self.x - padding <= point.x <= self.x + self.width + padding
            and self.y - padding <= point.y <= self.y + self.height + padding
        )

    def as_dict(self) -> dict[str, float]:
        return {
            "x": self.x,
            "y": self.y,
            "width": self.width,
            "height": self.height,
        }


@dataclass(slots=True)
class Task:
    id: str
    position: Point
    kind: str = "rescue"
    priority: int = 1
    assigned_robot: str | None = None
    completed: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "position": self.position.as_dict(),
            "kind": self.kind,
            "priority": self.priority,
            "assigned_robot": self.assigned_robot,
            "completed": self.completed,
        }


@dataclass(slots=True)
class RobotState:
    id: str
    position: Point
    heading_rad: float = 0.0
    status: RobotStatus = RobotStatus.IDLE
    goal: Point | None = None
    task_id: str | None = None
    path: list[Point] = field(default_factory=list)
    path_tick: int = 0
    last_seen_s: float = 0.0
    battery_percent: float = 100.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "position": self.position.as_dict(),
            "heading_rad": round(self.heading_rad, 4),
            "status": self.status.value,
            "goal": self.goal.as_dict() if self.goal else None,
            "task_id": self.task_id,
            "path": [point.as_dict() for point in self.path],
            "path_tick": self.path_tick,
            "last_seen_s": round(self.last_seen_s, 3),
            "battery_percent": round(self.battery_percent, 1),
        }


@dataclass(frozen=True, slots=True)
class PlannerResult:
    paths: dict[str, list[Point]]
    planning_ms: float
    unresolved: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class SafetyEvent:
    kind: str
    message: str
    robot_ids: tuple[str, ...]
    timestamp_s: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "message": self.message,
            "robot_ids": list(self.robot_ids),
            "timestamp_s": round(self.timestamp_s, 3),
        }

