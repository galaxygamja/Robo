from __future__ import annotations

import json
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

import robo_control.config as config_module
from robo_control.config import default_config_path, load_config
from robo_control.simulation import default_scenario_path, load_scenario


class ConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        with default_config_path().open("r", encoding="utf-8") as handle:
            self.raw = json.load(handle)

    def load_modified(self, *path_and_value: object):
        raw = deepcopy(self.raw)
        *path, value = path_and_value
        target = raw
        for key in path[:-1]:
            target = target[str(key)]
        target[str(path[-1])] = value
        with tempfile.TemporaryDirectory() as directory:
            path_obj = Path(directory) / "config.json"
            path_obj.write_text(json.dumps(raw), encoding="utf-8")
            return load_config(path_obj)

    def test_default_config_loads(self) -> None:
        config = load_config()
        self.assertEqual(6, config.mission.robot_count)
        self.assertGreaterEqual(
            config.safety.minimum_separation_m,
            2 * config.robot.radius_m,
        )

    def test_packaged_runtime_data_matches_source_defaults(self) -> None:
        package_data = Path(config_module.__file__).resolve().parent / "data"
        pairs = (
            (default_config_path(), package_data / "default.json"),
            (default_scenario_path(), package_data / "scenario_demo.json"),
        )
        for source, packaged in pairs:
            with self.subTest(filename=source.name):
                self.assertTrue(packaged.is_file())
                self.assertEqual(
                    json.loads(source.read_text(encoding="utf-8")),
                    json.loads(packaged.read_text(encoding="utf-8")),
                )

        config = load_config()
        scenario = load_scenario(None, config)
        self.assertEqual(config.mission.robot_count, len(scenario.robots))

    def test_robot_count_must_be_integer_in_supported_range(self) -> None:
        for invalid in (0, 9, 2.5, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                self.load_modified("mission", "robot_count", invalid)

    def test_minimum_separation_cannot_be_less_than_body_diameter(self) -> None:
        with self.assertRaises(ValueError):
            self.load_modified("safety", "minimum_separation_m", 0.1)

    def test_margin_and_ports_are_validated(self) -> None:
        with self.assertRaises(ValueError):
            self.load_modified("field", "boundary_margin_m", -0.1)
        with self.assertRaises(ValueError):
            self.load_modified("network", "dashboard_port", 65536)
        with self.assertRaises(ValueError):
            self.load_modified("network", "robot_udp_port", 9100.5)


if __name__ == "__main__":
    unittest.main()
