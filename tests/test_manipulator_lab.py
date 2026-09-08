"""Integration with existing mission intent and the unchanged real-feedback gate."""
import json
import subprocess
import sys
import unittest

from test_mission_runtime import MissionHarness

from robo_control.fake_manipulator import FakeManipulatorReceiver
from robo_control.manipulator_lab import run_demo
from robo_control.manipulator_wire import ManipulatorSender, operation_from_intent
from robo_control.wire_codec import encode_frame


class ManipulatorLabTests(unittest.TestCase):
    def test_four_robot_fault_lab(self):
        result = run_demo()
        self.assertTrue(result["passed"], result)
        self.assertEqual({"H1", "H2", "B1", "B2"}, result["receivers"].keys())
        self.assertEqual(13, len(result["checks"]))
        self.assertFalse(result["device_io"])
        self.assertFalse(result["mission_completed"])
        self.assertTrue(all(v["requested_action"] is None for v in result["receivers"].values()))

    def test_cli_json_and_success_exit(self):
        completed = subprocess.run([sys.executable, "-m", "robo_control.manipulator_lab", "--compact"],
                                   capture_output=True, text=True, check=True, timeout=20)
        self.assertTrue(json.loads(completed.stdout)["passed"])

    def test_real_mission_rejects_fake_sample_even_with_matching_identity(self):
        h = MissionHarness()
        h.advance_to_sensor()
        mission = h.session.mission
        original_phase = mission.active.phase
        intent = mission.snapshot()["manipulator_intent"]
        operation = operation_from_intent(intent, timeout_ms=1000)
        sender = ManipulatorSender(intent["robot_id"], session_id=intent["session_id"])
        receiver = FakeManipulatorReceiver(intent["robot_id"], supported_actions={operation["action"]})

        def exchange(message, at):
            self.assertTrue(sender.accept_frame(receiver.receive_frame(encode_frame(message), 1000. + at), 10. + at))

        exchange(sender.begin(operation, 10.), .01)
        exchange(sender.execute(10.02), .03)
        receiver.inject_test_sample(operation["command_id"], {"servo_closed": True}, 1000.04)
        exchange(sender.sample(10.05), .06)
        evidence = sender.snapshot()["evidence"]
        # Even adding a plausible host timestamp cannot launder the synthetic flag.
        evidence["observed_at_s"] = h.now + .02
        h.step(feedback=evidence)
        self.assertEqual(original_phase, mission.active.phase)
        self.assertEqual("wrong_feedback_identity_or_synthetic", mission.feedback_reason)
        self.assertEqual([], mission.completed)
        self.assertFalse(mission.snapshot()["manipulator_intent"]["dispatch_enabled"])


if __name__ == "__main__":
    unittest.main()
