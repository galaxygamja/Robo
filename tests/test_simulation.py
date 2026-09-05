from __future__ import annotations

import unittest

from robo_control.config import load_config
from robo_control.models import MissionStatus, Point, Rectangle, RobotState, RobotStatus, Task
from robo_control.simulation import Scenario, SimulationEngine


class SimulationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.engine = SimulationEngine(load_config())

    def test_default_six_robot_mission_completes_without_collision(self) -> None:
        self.assertEqual(6, len(self.engine.robots))
        self.assertEqual((), self.engine.unresolved)
        self.engine.start()

        minimum_seen = float("inf")
        for _ in range(2500):
            self.engine.step(0.05)
            robots = list(self.engine.robots.values())
            for index, first in enumerate(robots):
                for second in robots[index + 1 :]:
                    minimum_seen = min(minimum_seen, first.position.distance_to(second.position))
            if self.engine.status != MissionStatus.RUNNING:
                break

        snapshot = self.engine.snapshot()
        self.assertEqual(MissionStatus.COMPLETED, self.engine.status)
        self.assertEqual(0, snapshot["metrics"]["collision_count"])
        self.assertEqual(6, snapshot["metrics"]["completed_tasks"])
        self.assertGreaterEqual(
            minimum_seen + 1e-6,
            self.engine.config.safety.minimum_separation_m,
        )

    def test_emergency_stop_is_fail_closed(self) -> None:
        self.engine.start()
        self.engine.step(0.1)
        self.engine.emergency_stop("test")
        before = [robot.position for robot in self.engine.robots.values()]
        self.engine.step(5.0)

        self.assertEqual(MissionStatus.SAFETY_STOP, self.engine.status)
        self.assertEqual(before, [robot.position for robot in self.engine.robots.values()])
        self.assertTrue(all(robot.status == RobotStatus.STOPPED for robot in self.engine.robots.values()))

    def test_replan_does_not_clear_emergency_stop(self) -> None:
        self.engine.start()
        self.engine.emergency_stop("test")
        self.engine.replan()
        self.assertEqual(MissionStatus.SAFETY_STOP, self.engine.status)
        self.assertTrue(all(robot.status == RobotStatus.STOPPED for robot in self.engine.robots.values()))

    def test_reset_is_deterministic(self) -> None:
        paths_before = {
            robot.id: [point.as_dict() for point in robot.path]
            for robot in self.engine.robots.values()
        }
        self.engine.start()
        self.engine.step(1.0)
        self.engine.reset()
        paths_after = {
            robot.id: [point.as_dict() for point in robot.path]
            for robot in self.engine.robots.values()
        }
        self.assertEqual(paths_before, paths_after)
        self.assertEqual(MissionStatus.READY, self.engine.status)

    def test_one_robot_processes_more_than_one_task_in_rounds(self) -> None:
        scenario = Scenario(
            name="multi-round",
            start_zone=Rectangle(0.0, 0.8, 0.5, 0.381),
            robots=(RobotState("R1", Point(0.15, 0.95)),),
            tasks=(
                Task("T1", Point(0.15, 0.20)),
                Task("T2", Point(0.85, 0.20)),
            ),
            obstacles=(),
        )
        engine = SimulationEngine(load_config(), scenario=scenario)
        engine.start()
        for _ in range(2500):
            engine.step(0.05)
            if engine.status != MissionStatus.RUNNING:
                break

        self.assertEqual(MissionStatus.COMPLETED, engine.status)
        self.assertEqual(2, engine.snapshot()["metrics"]["completed_tasks"])
        self.assertGreaterEqual(engine.replan_count, 2)

    def test_unreachable_task_pauses_instead_of_replanning_every_tick(self) -> None:
        scenario = Scenario(
            name="unreachable",
            start_zone=Rectangle(0.0, 0.8, 0.5, 0.381),
            robots=(RobotState("R1", Point(0.15, 0.95)),),
            tasks=(Task("T1", Point(0.85, 0.20)),),
            obstacles=(Rectangle(0.45, 0.0, 0.10, 1.181),),
        )
        engine = SimulationEngine(load_config(), scenario=scenario)
        self.assertEqual(("R1",), engine.unresolved)
        engine.start()
        for _ in range(20):
            engine.step(0.05)

        self.assertEqual(MissionStatus.PAUSED, engine.status)
        self.assertLessEqual(engine.replan_count, 2)
        self.assertEqual("planner_stalled", engine.snapshot()["events"][-1]["kind"])


if __name__ == "__main__":
    unittest.main()
