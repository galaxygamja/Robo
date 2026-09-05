from __future__ import annotations

import math
import json
import threading
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import AppConfig, default_data_path
from .models import (
    MissionStatus,
    Point,
    Rectangle,
    RobotState,
    RobotStatus,
    SafetyEvent,
    Task,
)
from .planner import GridSpec, MultiRobotPlanner, assign_tasks


@dataclass(frozen=True, slots=True)
class Scenario:
    name: str
    start_zone: Rectangle
    robots: tuple[RobotState, ...]
    tasks: tuple[Task, ...]
    obstacles: tuple[Rectangle, ...]


def default_scenario_path() -> Path:
    return default_data_path("scenario_demo.json")


def load_scenario(path: str | Path | None, config: AppConfig) -> Scenario:
    """Load a deterministic scenario. Coordinates are metres in field space."""

    scenario_path = Path(path) if path else default_scenario_path()
    with scenario_path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)
    start_raw = raw["start_zone"]
    start_zone = Rectangle(
        float(start_raw["x"]),
        float(start_raw["y"]),
        float(start_raw["width"]),
        float(start_raw["height"]),
    )
    robot_rows = raw["robots"][: config.mission.robot_count]
    task_rows = raw["tasks"]
    robots = tuple(
        RobotState(
            id=str(row["id"]),
            position=Point(float(row["x"]), float(row["y"])),
            heading_rad=float(row.get("heading_rad", 0.0)),
        )
        for row in robot_rows
    )
    tasks = tuple(
        Task(
            id=str(row["id"]),
            position=Point(float(row["x"]), float(row["y"])),
            kind=str(row.get("kind", "rescue")),
            priority=int(row.get("priority", 1)),
        )
        for row in task_rows
    )
    obstacles = tuple(
        Rectangle(
            float(row["x"]),
            float(row["y"]),
            float(row["width"]),
            float(row["height"]),
        )
        for row in raw.get("obstacles", [])
    )

    if len(robots) != config.mission.robot_count:
        raise ValueError(
            f"scenario has {len(robots)} robots but config requests {config.mission.robot_count}"
        )
    if len({robot.id for robot in robots}) != len(robots):
        raise ValueError("scenario robot IDs must be unique")
    if len({task.id for task in tasks}) != len(tasks):
        raise ValueError("scenario task IDs must be unique")
    for robot in robots:
        if not start_zone.contains(robot.position):
            raise ValueError(f"{robot.id} is outside the configured start zone")
        radius = config.robot.radius_m
        if not (
            radius <= robot.position.x <= config.field.width_m - radius
            and radius <= robot.position.y <= config.field.height_m - radius
        ):
            raise ValueError(f"{robot.id} body crosses the field boundary")
    for index, first in enumerate(robots):
        for second in robots[index + 1 :]:
            if (
                first.position.distance_to(second.position) + 1e-9
                < config.safety.minimum_separation_m
            ):
                raise ValueError(
                    f"{first.id} and {second.id} violate startup minimum separation"
                )
    for label, point in [
        *((robot.id, robot.position) for robot in robots),
        *((task.id, task.position) for task in tasks),
    ]:
        if not (0.0 <= point.x <= config.field.width_m and 0.0 <= point.y <= config.field.height_m):
            raise ValueError(f"{label} is outside the field")

    return Scenario(
        name=str(raw.get("name", scenario_path.stem)),
        start_zone=start_zone,
        robots=robots,
        tasks=tasks,
        obstacles=obstacles,
    )


class SimulationEngine:
    def __init__(
        self,
        config: AppConfig,
        scenario: Scenario | None = None,
        scenario_path: str | Path | None = None,
    ) -> None:
        self.config = config
        self.scenario = scenario or load_scenario(scenario_path, config)
        clearance = config.robot.radius_m + config.field.boundary_margin_m
        self.grid = GridSpec(
            config.field.width_m,
            config.field.height_m,
            config.field.grid_cell_m,
            clearance,
        )
        self.planner = MultiRobotPlanner(
            self.grid,
            self.scenario.obstacles,
            config.safety.minimum_separation_m,
        )
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._shutdown = threading.Event()
        self._last_wall_time = time.monotonic()
        self._events: list[SafetyEvent] = []
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self.robots = {robot.id: deepcopy(robot) for robot in self.scenario.robots}
            self.tasks = {task.id: deepcopy(task) for task in self.scenario.tasks}
            self.status = MissionStatus.READY
            self.elapsed_s = 0.0
            self._plan_started_at_s = 0.0
            self.planning_ms = 0.0
            self.unresolved: tuple[str, ...] = ()
            self.collision_count = 0
            self.replan_count = 0
            self._events.clear()
            self._assign_and_plan()

    def _assign_and_plan(self) -> None:
        for task in self.tasks.values():
            task.assigned_robot = None
        assignments = assign_tasks(self.robots.values(), self.tasks.values())
        goals: dict[str, Point] = {}
        for robot in self.robots.values():
            robot.goal = None
            robot.task_id = None
            robot.path = [robot.position]
            robot.path_tick = 0
            robot.status = RobotStatus.IDLE
        for robot_id, task in assignments.items():
            robot = self.robots[robot_id]
            robot.goal = task.position
            robot.task_id = task.id
            task.assigned_robot = robot_id
            goals[robot_id] = task.position

        step_s = self.config.field.grid_cell_m / self.config.robot.linear_speed_mps
        max_ticks = min(800, max(80, math.ceil(self.config.mission.duration_s / step_s)))
        result = self.planner.plan_all(self.robots.values(), goals, max_ticks=max_ticks)
        self.planning_ms = result.planning_ms
        self.unresolved = result.unresolved
        self.replan_count += 1
        self._plan_started_at_s = self.elapsed_s
        for robot_id, path in result.paths.items():
            robot = self.robots[robot_id]
            robot.path = path
            if robot_id in result.unresolved:
                robot.status = RobotStatus.STOPPED
            else:
                robot.status = RobotStatus.WAITING

    def replan(self) -> None:
        with self._lock:
            if self.status in (
                MissionStatus.COMPLETED,
                MissionStatus.TIMEOUT,
                MissionStatus.SAFETY_STOP,
            ):
                return
            self._assign_and_plan()

    def start(self) -> None:
        with self._lock:
            if self.status in (MissionStatus.COMPLETED, MissionStatus.TIMEOUT):
                self.reset()
            if self.status == MissionStatus.SAFETY_STOP:
                return
            self.status = MissionStatus.RUNNING
            for robot in self.robots.values():
                if robot.path and robot.id not in self.unresolved:
                    robot.status = RobotStatus.MOVING
            self._last_wall_time = time.monotonic()

    def pause(self) -> None:
        with self._lock:
            if self.status == MissionStatus.RUNNING:
                self.status = MissionStatus.PAUSED
                for robot in self.robots.values():
                    if robot.status == RobotStatus.MOVING:
                        robot.status = RobotStatus.WAITING

    def emergency_stop(self, reason: str = "operator emergency stop") -> None:
        with self._lock:
            self.status = MissionStatus.SAFETY_STOP
            for robot in self.robots.values():
                robot.status = RobotStatus.STOPPED
            self._events.append(
                SafetyEvent("emergency_stop", reason, tuple(self.robots), self.elapsed_s)
            )

    def _interpolated_position(self, robot: RobotState, local_time_s: float) -> tuple[Point, int]:
        if len(robot.path) <= 1:
            return robot.path[0] if robot.path else robot.position, 0
        step_s = self.config.field.grid_cell_m / self.config.robot.linear_speed_mps
        progress = max(0.0, local_time_s / step_s)
        tick = min(int(progress), len(robot.path) - 1)
        if tick >= len(robot.path) - 1:
            return robot.path[-1], len(robot.path) - 1
        fraction = progress - tick
        start = robot.path[tick]
        end = robot.path[tick + 1]
        return (
            Point(start.x + (end.x - start.x) * fraction, start.y + (end.y - start.y) * fraction),
            tick,
        )

    def _validate_safety(self) -> bool:
        robots = tuple(self.robots.values())
        for index, first in enumerate(robots):
            if not (
                self.config.robot.radius_m <= first.position.x <= self.config.field.width_m - self.config.robot.radius_m
                and self.config.robot.radius_m <= first.position.y <= self.config.field.height_m - self.config.robot.radius_m
            ):
                self.emergency_stop(f"{first.id} crossed the field boundary")
                return False
            for second in robots[index + 1 :]:
                distance = first.position.distance_to(second.position)
                if distance + 1e-6 < self.config.safety.minimum_separation_m:
                    self.collision_count += 1
                    self._events.append(
                        SafetyEvent(
                            "collision",
                            f"{first.id} and {second.id} separation={distance:.3f}m",
                            (first.id, second.id),
                            self.elapsed_s,
                        )
                    )
                    self.emergency_stop("configured minimum separation violated")
                    return False
        return True

    def step(self, dt_s: float) -> None:
        with self._lock:
            if self.status != MissionStatus.RUNNING:
                return
            scaled_dt = min(0.25, max(0.0, dt_s)) * self.config.mission.simulation_speed
            self.elapsed_s += scaled_dt
            if self.elapsed_s >= self.config.mission.duration_s:
                self.elapsed_s = self.config.mission.duration_s
                self.status = MissionStatus.TIMEOUT
                for robot in self.robots.values():
                    robot.status = RobotStatus.STOPPED
                return

            local_time = self.elapsed_s - self._plan_started_at_s
            for robot in self.robots.values():
                if robot.id in self.unresolved or robot.status == RobotStatus.ARRIVED:
                    continue
                previous = robot.position
                robot.position, robot.path_tick = self._interpolated_position(robot, local_time)
                dx = robot.position.x - previous.x
                dy = robot.position.y - previous.y
                if abs(dx) + abs(dy) > 1e-9:
                    robot.heading_rad = math.atan2(dy, dx)
                robot.last_seen_s = self.elapsed_s
                robot.battery_percent = max(0.0, robot.battery_percent - scaled_dt * 0.015)
                if (
                    robot.path_tick >= len(robot.path) - 1
                    and (
                        robot.goal is None
                        or robot.position.distance_to(robot.goal)
                        <= self.config.robot.goal_tolerance_m
                    )
                ):
                    robot.status = RobotStatus.ARRIVED
                    if robot.task_id:
                        self.tasks[robot.task_id].completed = True
                else:
                    robot.status = RobotStatus.MOVING

            if not self._validate_safety():
                return
            if self.tasks and all(task.completed for task in self.tasks.values()):
                self.status = MissionStatus.COMPLETED
            elif all(
                robot.status in (RobotStatus.ARRIVED, RobotStatus.STOPPED)
                for robot in self.robots.values()
            ):
                self._assign_and_plan()
                assigned = [robot for robot in self.robots.values() if robot.task_id]
                if assigned and all(robot.id in self.unresolved for robot in assigned):
                    self.status = MissionStatus.PAUSED
                    self._events.append(
                        SafetyEvent(
                            "planner_stalled",
                            "remaining tasks are unreachable; automatic replanning paused",
                            tuple(robot.id for robot in assigned),
                            self.elapsed_s,
                        )
                    )
                else:
                    for robot in assigned:
                        if robot.path and robot.id not in self.unresolved:
                            robot.status = RobotStatus.MOVING

    def run_background(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._shutdown.clear()
        self._thread = threading.Thread(target=self._loop, name="simulation", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        period = 1.0 / self.config.mission.control_hz
        self._last_wall_time = time.monotonic()
        while not self._shutdown.is_set():
            now = time.monotonic()
            dt = now - self._last_wall_time
            self._last_wall_time = now
            self.step(dt)
            self._shutdown.wait(period)

    def close(self) -> None:
        self._shutdown.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "version": 1,
                "scenario": self.scenario.name,
                "status": self.status.value,
                "elapsed_s": round(self.elapsed_s, 3),
                "remaining_s": round(max(0.0, self.config.mission.duration_s - self.elapsed_s), 3),
                "duration_s": self.config.mission.duration_s,
                "field": {
                    "width_m": self.config.field.width_m,
                    "height_m": self.config.field.height_m,
                    "start_zone": self.scenario.start_zone.as_dict(),
                },
                "robots": [robot.as_dict() for robot in self.robots.values()],
                "tasks": [task.as_dict() for task in self.tasks.values()],
                "obstacles": [obstacle.as_dict() for obstacle in self.scenario.obstacles],
                "metrics": {
                    "planning_ms": round(self.planning_ms, 3),
                    "replan_count": self.replan_count,
                    "collision_count": self.collision_count,
                    "completed_tasks": sum(task.completed for task in self.tasks.values()),
                    "total_tasks": len(self.tasks),
                    "unresolved_robots": list(self.unresolved),
                },
                "events": [event.as_dict() for event in self._events[-20:]],
                "warnings": [
                    "DEMO 좌표이며 공식 경기 배치가 아닙니다.",
                    "실제 모터와 드론 출력은 기본적으로 비활성화되어 있습니다.",
                ],
            }
