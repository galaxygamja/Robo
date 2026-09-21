"""No robot connection: Arduino IDE export and safe public settings contract."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ArduinoSketchTests(unittest.TestCase):
    def test_export_matches_canonical_firmware(self):
        spec = importlib.util.spec_from_file_location("sync_sketch", ROOT / "tools/sync_arduino_sketch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        expected = module.expected_files()
        actual = {p.name for p in module.DEST.iterdir() if p.is_file()}
        self.assertEqual(set(expected), actual)
        for name, text in expected.items():
            with self.subTest(name=name):
                self.assertEqual(text, (module.DEST / name).read_text(encoding="utf-8"))

    def test_entry_point_is_arduino_sketch(self):
        text = (ROOT / "arduino/RoboRobot/RoboRobot.ino").read_text(encoding="utf-8")
        self.assertIn("void setup()", text)
        self.assertIn("void loop()", text)
        self.assertIn("roboSetup();", text)
        self.assertIn("roboLoop();", text)

    def test_public_defaults_are_safe(self):
        text = (ROOT / "arduino/RoboRobot/RobotSettings.h").read_text(encoding="utf-8")
        self.assertIn("#define HARDWARE_OUTPUT_ENABLED 0", text)
        self.assertIn("#define ACTUATOR_CALIBRATION_CONFIRMED 0", text)
        example = (ROOT / "arduino/RoboRobot/Secrets.example.h").read_text(encoding="utf-8")
        self.assertIn('#define ROBOT_SHARED_TOKEN "CHANGE_ME"', example)
        self.assertNotIn("WiFi.begin", text)


if __name__ == "__main__":
    unittest.main()
