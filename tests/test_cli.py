from __future__ import annotations

import json
import unittest
from contextlib import redirect_stdout
from dataclasses import replace
from io import BytesIO, StringIO, TextIOWrapper

from robo_control.__main__ import run_headless
from robo_control.config import load_config
from robo_control.models import Point, Rectangle, RobotState, Task
from robo_control.simulation import Scenario, SimulationEngine


class HeadlessTests(unittest.TestCase):
    def run_silently(self, engine: SimulationEngine, step: float):
        output = StringIO()
        with redirect_stdout(output):
            exit_code = run_headless(engine, step)
        return exit_code, json.loads(output.getvalue())

    def test_requested_step_above_clamp_still_finishes_mission(self) -> None:
        code, state = self.run_silently(SimulationEngine(load_config()), 5.0)
        self.assertEqual(0, code)
        self.assertEqual("completed", state["status"])
        self.assertEqual(6, state["metrics"]["completed_tasks"])
        self.assertEqual(0, state["metrics"]["collision_count"])

    def test_slow_simulation_runs_until_completion(self) -> None:
        config = load_config()
        config = replace(config, mission=replace(config.mission, simulation_speed=0.01))
        code, state = self.run_silently(SimulationEngine(config), 0.05)
        self.assertEqual(0, code)
        self.assertEqual("completed", state["status"])
        self.assertEqual(6, state["metrics"]["completed_tasks"])

    def test_unfinished_mission_reaches_exact_timeout(self) -> None:
        config = load_config()
        config = replace(config, mission=replace(config.mission, duration_s=0.3))
        code, state = self.run_silently(SimulationEngine(config), 5.0)
        self.assertEqual(2, code)
        self.assertEqual("timeout", state["status"])
        self.assertEqual(0.3, state["elapsed_s"])
        self.assertTrue(all(robot["status"] == "stopped" for robot in state["robots"]))

    def test_paused_unreachable_mission_returns_without_spinning(self) -> None:
        scenario = Scenario(
            name="unreachable-headless",
            start_zone=Rectangle(0.0, 0.8, 0.5, 0.381),
            robots=(RobotState("R1", Point(0.15, 0.95)),),
            tasks=(Task("T1", Point(0.85, 0.20)),),
            obstacles=(Rectangle(0.45, 0.0, 0.10, 1.181),),
        )
        code, state = self.run_silently(SimulationEngine(load_config(), scenario=scenario), 0.05)
        self.assertEqual(2, code)
        self.assertEqual("paused", state["status"])
        self.assertEqual(0.05, state["elapsed_s"])
        self.assertEqual("planner_stalled", state["events"][-1]["kind"])

    def test_invalid_headless_steps_are_rejected_before_start(self) -> None:
        engine = SimulationEngine(load_config())
        for invalid in (0, -0.1, float("nan"), float("inf"), float("-inf")):
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(ValueError, "finite and positive"):
                    self.run_silently(engine, invalid)
                self.assertEqual("ready", engine.status.value)

    def test_headless_json_preserves_korean_on_legacy_stdout(self) -> None:
        for encoding in ("ascii", "cp1252"):
            with self.subTest(encoding=encoding):
                engine = SimulationEngine(load_config())
                expected_warnings = engine.snapshot()["warnings"]
                buffer = BytesIO()
                with TextIOWrapper(buffer, encoding=encoding) as output:
                    with redirect_stdout(output):
                        code = run_headless(engine, 0.05)
                    output.flush()
                    state = json.loads(buffer.getvalue().decode(encoding))
                self.assertEqual(0, code)
                self.assertEqual("completed", state["status"])
                self.assertEqual(expected_warnings, state["warnings"])
                self.assertTrue(any("실제" in warning for warning in state["warnings"]))


if __name__ == "__main__":
    unittest.main()
