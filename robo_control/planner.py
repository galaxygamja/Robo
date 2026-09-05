from __future__ import annotations

import heapq
import itertools
import math
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Iterable

from .models import PlannerResult, Point, Rectangle, RobotState, Task

Cell = tuple[int, int]


@dataclass(frozen=True, slots=True)
class GridSpec:
    width_m: float
    height_m: float
    cell_m: float
    clearance_m: float

    @property
    def columns(self) -> int:
        return max(1, int(self.width_m // self.cell_m))

    @property
    def rows(self) -> int:
        return max(1, int(self.height_m // self.cell_m))

    def world_to_cell(self, point: Point) -> Cell:
        column = min(self.columns - 1, max(0, int(point.x / self.cell_m)))
        row = min(self.rows - 1, max(0, int(point.y / self.cell_m)))
        return column, row

    def cell_to_world(self, cell: Cell) -> Point:
        return Point((cell[0] + 0.5) * self.cell_m, (cell[1] + 0.5) * self.cell_m)

    def in_bounds(self, cell: Cell) -> bool:
        if not (0 <= cell[0] < self.columns and 0 <= cell[1] < self.rows):
            return False
        point = self.cell_to_world(cell)
        return (
            self.clearance_m <= point.x <= self.width_m - self.clearance_m
            and self.clearance_m <= point.y <= self.height_m - self.clearance_m
        )


class ReservationTable:
    """Time-indexed occupied cells and transitions for prioritized planning."""

    def __init__(self, minimum_separation_cells: float = 0.0) -> None:
        self.minimum_separation_cells = minimum_separation_cells
        self.vertices: dict[int, list[Cell]] = defaultdict(list)
        self.edges: set[tuple[Cell, Cell, int]] = set()
        self.transitions: dict[int, list[tuple[Cell, Cell]]] = defaultdict(list)

    def vertex_is_reserved(self, cell: Cell, tick: int) -> bool:
        limit_sq = self.minimum_separation_cells**2
        for occupied in self.vertices.get(tick, ()):
            dx = cell[0] - occupied[0]
            dy = cell[1] - occupied[1]
            if dx * dx + dy * dy < limit_sq - 1e-9:
                return True
        return False

    def transition_is_reserved(self, current: Cell, nxt: Cell, next_tick: int) -> bool:
        if (nxt, current, next_tick) in self.edges:
            return True
        limit_sq = self.minimum_separation_cells**2
        for other_start, other_end in self.transitions.get(next_tick, ()):
            relative_start = (
                float(current[0] - other_start[0]),
                float(current[1] - other_start[1]),
            )
            relative_velocity = (
                float((nxt[0] - current[0]) - (other_end[0] - other_start[0])),
                float((nxt[1] - current[1]) - (other_end[1] - other_start[1])),
            )
            velocity_sq = relative_velocity[0] ** 2 + relative_velocity[1] ** 2
            if velocity_sq <= 1e-12:
                closest_u = 0.0
            else:
                closest_u = -(
                    relative_start[0] * relative_velocity[0]
                    + relative_start[1] * relative_velocity[1]
                ) / velocity_sq
                closest_u = min(1.0, max(0.0, closest_u))
            dx = relative_start[0] + relative_velocity[0] * closest_u
            dy = relative_start[1] + relative_velocity[1] * closest_u
            if dx * dx + dy * dy < limit_sq - 1e-9:
                return True
        return False

    def can_hold(self, cell: Cell, from_tick: int, through_tick: int) -> bool:
        """Return whether a robot may remain at cell for the rest of the horizon."""

        return all(
            not self.vertex_is_reserved(cell, tick)
            for tick in range(from_tick, through_tick + 1)
        )

    def reserve_path(self, path: list[Cell], horizon: int) -> None:
        if not path:
            return
        for tick, cell in enumerate(path):
            self.vertices[tick].append(cell)
            if tick:
                self.edges.add((path[tick - 1], cell, tick))
                self.transitions[tick].append((path[tick - 1], cell))
        goal = path[-1]
        for tick in range(len(path), horizon + 1):
            self.vertices[tick].append(goal)
            self.edges.add((goal, goal, tick))
            self.transitions[tick].append((goal, goal))


class SpaceTimePlanner:
    """Deterministic 4-connected space-time A* with wait actions."""

    _MOVES: tuple[Cell, ...] = ((0, 0), (0, -1), (-1, 0), (1, 0), (0, 1))

    def __init__(self, grid: GridSpec, obstacles: Iterable[Rectangle] = ()) -> None:
        self.grid = grid
        self.obstacles = tuple(obstacles)
        self._blocked = self._build_blocked_cells()
        self._component_by_cell = self._build_connected_components()

    def _build_blocked_cells(self) -> set[Cell]:
        blocked: set[Cell] = set()
        for column in range(self.grid.columns):
            for row in range(self.grid.rows):
                cell = (column, row)
                point = self.grid.cell_to_world(cell)
                if not self.grid.in_bounds(cell) or any(
                    obstacle.contains(point, padding=self.grid.clearance_m)
                    for obstacle in self.obstacles
                ):
                    blocked.add(cell)
        return blocked

    def _build_connected_components(self) -> dict[Cell, int]:
        """Index fixed-grid connectivity once, before adding time reservations."""

        components: dict[Cell, int] = {}
        component_id = 0
        for column in range(self.grid.columns):
            for row in range(self.grid.rows):
                start = (column, row)
                if start in self._blocked or start in components:
                    continue
                components[start] = component_id
                frontier = deque([start])
                while frontier:
                    current = frontier.popleft()
                    for dx, dy in self._MOVES:
                        if dx == 0 and dy == 0:
                            continue
                        nxt = (current[0] + dx, current[1] + dy)
                        if (
                            nxt in components
                            or nxt in self._blocked
                            or not self.grid.in_bounds(nxt)
                        ):
                            continue
                        components[nxt] = component_id
                        frontier.append(nxt)
                component_id += 1
        return components

    def _reachable_with_blocked(
        self,
        start: Cell,
        goal: Cell,
        blocked: set[Cell],
    ) -> bool:
        """Check connectivity when call-specific static obstacles are supplied."""

        frontier = deque([start])
        visited = {start}
        while frontier:
            current = frontier.popleft()
            if current == goal:
                return True
            for dx, dy in self._MOVES:
                if dx == 0 and dy == 0:
                    continue
                nxt = (current[0] + dx, current[1] + dy)
                if nxt in visited or nxt in blocked or not self.grid.in_bounds(nxt):
                    continue
                visited.add(nxt)
                frontier.append(nxt)
        return False

    def blocked_cells_around(self, points: Iterable[Point], radius_m: float) -> set[Cell]:
        cells = set(self._blocked)
        radius_sq = radius_m * radius_m
        centers = tuple(points)
        for column in range(self.grid.columns):
            for row in range(self.grid.rows):
                cell = (column, row)
                point = self.grid.cell_to_world(cell)
                if any(
                    (point.x - center.x) ** 2 + (point.y - center.y) ** 2 < radius_sq
                    for center in centers
                ):
                    cells.add(cell)
        return cells

    def plan(
        self,
        start: Point,
        goal: Point,
        reservations: ReservationTable,
        max_ticks: int,
        extra_blocked: set[Cell] | None = None,
    ) -> list[Cell] | None:
        start_cell = self.grid.world_to_cell(start)
        goal_cell = self.grid.world_to_cell(goal)
        blocked = self._blocked | (extra_blocked or set())

        if not self.grid.in_bounds(start_cell) or not self.grid.in_bounds(goal_cell):
            return None
        if start_cell in blocked or goal_cell in blocked:
            return None
        if self._component_by_cell.get(start_cell) != self._component_by_cell.get(goal_cell):
            return None
        if extra_blocked and not self._reachable_with_blocked(start_cell, goal_cell, blocked):
            return None

        def heuristic(cell: Cell) -> int:
            return abs(cell[0] - goal_cell[0]) + abs(cell[1] - goal_cell[1])

        counter = itertools.count()
        frontier: list[tuple[int, int, int, Cell, int]] = []
        heapq.heappush(frontier, (heuristic(start_cell), 0, next(counter), start_cell, 0))
        came_from: dict[tuple[Cell, int], tuple[Cell, int] | None] = {(start_cell, 0): None}
        best_cost: dict[tuple[Cell, int], int] = {(start_cell, 0): 0}

        final_state: tuple[Cell, int] | None = None
        while frontier:
            _, cost, _, current, tick = heapq.heappop(frontier)
            state = (current, tick)
            if cost != best_cost.get(state):
                continue
            if current == goal_cell and reservations.can_hold(current, tick, max_ticks):
                final_state = state
                break
            if tick >= max_ticks:
                continue

            for dx, dy in self._MOVES:
                nxt = (current[0] + dx, current[1] + dy)
                next_tick = tick + 1
                if not self.grid.in_bounds(nxt) or nxt in blocked:
                    continue
                if reservations.vertex_is_reserved(nxt, next_tick):
                    continue
                if reservations.transition_is_reserved(current, nxt, next_tick):
                    continue
                next_state = (nxt, next_tick)
                next_cost = cost + 1
                if next_cost >= best_cost.get(next_state, math.inf):
                    continue
                best_cost[next_state] = next_cost
                came_from[next_state] = state
                heapq.heappush(
                    frontier,
                    (next_cost + heuristic(nxt), next_cost, next(counter), nxt, next_tick),
                )

        if final_state is None:
            return None

        path: list[Cell] = []
        state: tuple[Cell, int] | None = final_state
        while state is not None:
            path.append(state[0])
            state = came_from[state]
        path.reverse()
        return path


def assign_tasks(robots: Iterable[RobotState], tasks: Iterable[Task]) -> dict[str, Task]:
    """Assign unique tasks; exact through eight items, deterministic greedy above eight."""

    available_robots = tuple(sorted(robots, key=lambda item: item.id))
    available_tasks = tuple(
        sorted((task for task in tasks if not task.completed), key=lambda item: item.id)
    )
    if not available_robots or not available_tasks:
        return {}
    def cost(robot: RobotState, task: Task) -> float:
        priority_weight = max(1, task.priority)
        return robot.position.distance_to(task.position) / priority_weight

    result: dict[str, Task] = {}
    if max(len(available_robots), len(available_tasks)) > 8:
        candidates = sorted(
            (
                (cost(robot, task), robot.id, task.id, robot, task)
                for robot in available_robots
                for task in available_tasks
            ),
            key=lambda item: item[:3],
        )
        used_robots: set[str] = set()
        used_tasks: set[str] = set()
        for _, robot_id, task_id, robot, task in candidates:
            if robot_id in used_robots or task_id in used_tasks:
                continue
            result[robot.id] = task
            used_robots.add(robot_id)
            used_tasks.add(task_id)
            if len(result) == min(len(available_robots), len(available_tasks)):
                break
        return result

    if len(available_robots) <= len(available_tasks):
        best = min(
            itertools.permutations(available_tasks, len(available_robots)),
            key=lambda candidate: (
                sum(cost(robot, task) for robot, task in zip(available_robots, candidate)),
                tuple(task.id for task in candidate),
            ),
        )
        result.update(zip((robot.id for robot in available_robots), best))
    else:
        best_robots = min(
            itertools.permutations(available_robots, len(available_tasks)),
            key=lambda candidate: (
                sum(cost(robot, task) for robot, task in zip(candidate, available_tasks)),
                tuple(robot.id for robot in candidate),
            ),
        )
        result.update((robot.id, task) for robot, task in zip(best_robots, available_tasks))
    return result


class MultiRobotPlanner:
    def __init__(
        self,
        grid: GridSpec,
        obstacles: Iterable[Rectangle],
        minimum_separation_m: float,
    ) -> None:
        self.grid = grid
        self.minimum_separation_m = minimum_separation_m
        self.single = SpaceTimePlanner(grid, obstacles)

    def plan_all(
        self,
        robots: Iterable[RobotState],
        goals: dict[str, Point],
        max_ticks: int = 400,
    ) -> PlannerResult:
        started = time.perf_counter()
        all_robots = tuple(robots)
        ordered = sorted(
            (robot for robot in all_robots if robot.id in goals),
            key=lambda robot: (robot.position.distance_to(goals[robot.id]), robot.id),
        )
        stationary = {robot.id for robot in all_robots if robot.id not in goals}
        unresolved: set[str] = set()
        paths: dict[str, list[Point]] = {}

        # If a robot cannot be planned it becomes a stationary obstacle. Rebuild
        # earlier paths so none of them can pass through that newly stopped body.
        for _ in range(len(ordered) + 1):
            reservations = ReservationTable(self.minimum_separation_m / self.grid.cell_m)
            for robot in all_robots:
                if robot.id in stationary or robot.id in unresolved:
                    reservations.reserve_path(
                        [self.grid.world_to_cell(robot.position)], max_ticks
                    )

            candidate_paths: dict[str, list[Point]] = {
                robot.id: [robot.position] for robot in ordered if robot.id in unresolved
            }
            newly_unresolved: set[str] = set()
            for robot in ordered:
                if robot.id in unresolved:
                    continue
                path_cells = self.single.plan(
                    robot.position,
                    goals[robot.id],
                    reservations,
                    max_ticks=max_ticks,
                )
                if path_cells is None:
                    newly_unresolved.add(robot.id)
                    candidate_paths[robot.id] = [robot.position]
                    continue
                reservations.reserve_path(path_cells, max_ticks)
                candidate_paths[robot.id] = [
                    self.grid.cell_to_world(cell) for cell in path_cells
                ]

            paths = candidate_paths
            if not newly_unresolved:
                break
            unresolved.update(newly_unresolved)

        return PlannerResult(
            paths=paths,
            planning_ms=(time.perf_counter() - started) * 1000.0,
            unresolved=tuple(sorted(unresolved)),
        )
