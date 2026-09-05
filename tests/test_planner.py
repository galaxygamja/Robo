from __future__ import annotations

import unittest

from robo_control.models import Point, Rectangle, RobotState, Task
from robo_control.planner import GridSpec, MultiRobotPlanner, ReservationTable, SpaceTimePlanner, assign_tasks


class AssignmentTests(unittest.TestCase):
    def test_minimum_cost_assignment_is_deterministic(self) -> None:
        robots = [RobotState("R2", Point(9, 0)), RobotState("R1", Point(0, 0))]
        tasks = [Task("T2", Point(10, 0)), Task("T1", Point(1, 0))]

        assignment = assign_tasks(robots, tasks)

        self.assertEqual("T1", assignment["R1"].id)
        self.assertEqual("T2", assignment["R2"].id)

    def test_completed_task_is_not_assigned(self) -> None:
        robot = RobotState("R1", Point(0, 0))
        completed = Task("T1", Point(1, 0), completed=True)
        self.assertEqual({}, assign_tasks([robot], [completed]))

    def test_more_than_eight_tasks_uses_unique_deterministic_fallback(self) -> None:
        robots = [RobotState(f"R{i}", Point(i / 10, 0.0)) for i in range(6)]
        tasks = [Task(f"T{i}", Point(i / 10, 1.0)) for i in range(9)]

        first = assign_tasks(robots, tasks)
        second = assign_tasks(robots, tasks)

        self.assertEqual(6, len(first))
        self.assertEqual(6, len({task.id for task in first.values()}))
        self.assertEqual(
            {robot_id: task.id for robot_id, task in first.items()},
            {robot_id: task.id for robot_id, task in second.items()},
        )


class SpaceTimePlannerTests(unittest.TestCase):
    def test_unreachable_goal_returns_none(self) -> None:
        grid = GridSpec(1.0, 1.0, 0.1, 0.0)
        wall = Rectangle(0.45, 0.0, 0.1, 1.0)
        planner = SpaceTimePlanner(grid, [wall])

        path = planner.plan(Point(0.15, 0.55), Point(0.85, 0.55), ReservationTable(), 100)

        self.assertIsNone(path)

    def test_static_disconnect_returns_before_space_time_expansion(self) -> None:
        class CountingReservations(ReservationTable):
            def __init__(self) -> None:
                super().__init__()
                self.space_time_checks = 0

            def vertex_is_reserved(self, cell, tick):  # type: ignore[no-untyped-def]
                self.space_time_checks += 1
                return super().vertex_is_reserved(cell, tick)

            def transition_is_reserved(self, current, nxt, next_tick):  # type: ignore[no-untyped-def]
                self.space_time_checks += 1
                return super().transition_is_reserved(current, nxt, next_tick)

        grid = GridSpec(1.0, 1.0, 0.02, 0.0)
        wall = Rectangle(0.49, 0.0, 0.02, 1.0)
        planner = SpaceTimePlanner(grid, [wall])
        reservations = CountingReservations()

        path = planner.plan(
            Point(0.25, 0.5),
            Point(0.75, 0.5),
            reservations,
            max_ticks=800,
        )

        self.assertIsNone(path)
        self.assertEqual(0, reservations.space_time_checks)

    def test_goal_inside_inflated_obstacle_is_rejected(self) -> None:
        grid = GridSpec(1.0, 1.0, 0.1, 0.05)
        obstacle = Rectangle(0.4, 0.4, 0.2, 0.2)
        planner = SpaceTimePlanner(grid, [obstacle])

        path = planner.plan(Point(0.15, 0.15), Point(0.55, 0.55), ReservationTable(), 100)

        self.assertIsNone(path)

    def test_two_robots_have_no_vertex_or_swap_collision(self) -> None:
        grid = GridSpec(5.0, 3.0, 1.0, 0.0)
        planner = MultiRobotPlanner(grid, [], minimum_separation_m=0.1)
        robots = [
            RobotState("R1", Point(0.5, 1.5)),
            RobotState("R2", Point(4.5, 1.5)),
        ]
        result = planner.plan_all(
            robots,
            {"R1": Point(4.5, 1.5), "R2": Point(0.5, 1.5)},
            max_ticks=30,
        )
        self.assertEqual((), result.unresolved)

        first = [grid.world_to_cell(point) for point in result.paths["R1"]]
        second = [grid.world_to_cell(point) for point in result.paths["R2"]]
        horizon = max(len(first), len(second))
        first += [first[-1]] * (horizon - len(first))
        second += [second[-1]] * (horizon - len(second))
        for tick in range(horizon):
            self.assertNotEqual(first[tick], second[tick])
            if tick:
                self.assertFalse(first[tick - 1] == second[tick] and second[tick - 1] == first[tick])

    def test_goal_is_safe_for_the_entire_remaining_horizon(self) -> None:
        grid = GridSpec(6.0, 2.0, 1.0, 0.0)
        planner = SpaceTimePlanner(grid)
        reservations = ReservationTable(minimum_separation_cells=0.1)
        crossing_path = [(5, 0), (4, 0), (3, 0), (2, 0), (1, 0), (0, 0)]
        reservations.reserve_path(crossing_path, horizon=12)

        path = planner.plan(
            Point(0.5, 1.5),
            Point(2.5, 0.5),
            reservations,
            max_ticks=12,
        )

        self.assertIsNotNone(path)
        assert path is not None
        goal = path[-1]
        arrival_tick = len(path) - 1
        self.assertTrue(reservations.can_hold(goal, arrival_tick, 12))

    def test_swept_transition_respects_continuous_minimum_distance(self) -> None:
        reservations = ReservationTable(minimum_separation_cells=2.1)
        reservations.reserve_path([(0, 1), (0, 2)], horizon=1)

        self.assertTrue(reservations.transition_is_reserved((2, 2), (2, 1), 1))

    def test_idle_robot_is_reserved_as_a_stationary_obstacle(self) -> None:
        grid = GridSpec(5.0, 3.0, 1.0, 0.0)
        planner = MultiRobotPlanner(grid, [], minimum_separation_m=0.1)
        active = RobotState("R1", Point(0.5, 1.5))
        idle = RobotState("R2", Point(2.5, 1.5))

        result = planner.plan_all([active, idle], {"R1": Point(4.5, 1.5)}, max_ticks=20)

        cells = [grid.world_to_cell(point) for point in result.paths["R1"]]
        self.assertNotIn((2, 1), cells)

    def test_unresolved_robot_forces_earlier_paths_to_be_rebuilt(self) -> None:
        grid = GridSpec(2.0, 1.0, 1.0, 0.0)
        planner = MultiRobotPlanner(grid, [], minimum_separation_m=0.1)
        robots = [
            RobotState("R1", Point(0.5, 0.5)),
            RobotState("R2", Point(1.5, 0.5)),
        ]

        result = planner.plan_all(
            robots,
            {"R1": Point(1.5, 0.5), "R2": Point(0.5, 0.5)},
            max_ticks=10,
        )

        self.assertEqual(("R1", "R2"), result.unresolved)
        self.assertEqual([Point(0.5, 0.5)], result.paths["R1"])
        self.assertEqual([Point(1.5, 0.5)], result.paths["R2"])


if __name__ == "__main__":
    unittest.main()
