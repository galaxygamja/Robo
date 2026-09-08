from __future__ import annotations

import copy
import unittest

from robo_control.fake_manipulator import FakeManipulatorReceiver
from robo_control.manipulator_wire import (
    MAX_OPERATIONS,
    MAX_SEQUENCE,
    PHASE_ACTIONS,
    SIGNALS,
    ManipulatorSender,
    operation_from_intent,
    validate_operation,
)
from robo_control.wire_codec import FrameDecoder, WireError, decode_frame, encode_frame


def operation(command_id="mission:0:3", action="gripper_close", phase="close_servo", timeout_ms=1000):
    return {"command_id": command_id, "action": action, "phase": phase, "timeout_ms": timeout_ms}


class Harness:
    def __init__(self, robot_id="B1", host_origin=10., device_origin=1000.):
        self.host_origin, self.device_origin = host_origin, device_origin
        self.sender = ManipulatorSender(robot_id, session_id="mission")
        self.receiver = FakeManipulatorReceiver(robot_id, supported_actions=set().union(*PHASE_ACTIONS.values()))

    def exchange(self, message, t, *, delay=0.):
        response = self.receiver.receive_frame(encode_frame(message), self.device_origin + t)
        accepted = self.sender.accept_frame(response, self.host_origin + t + delay)
        return decode_frame(response), accepted

    def ready(self, op=None):
        request = self.sender.begin(op or operation(), self.host_origin)
        response, accepted = self.exchange(request, .01)
        assert accepted, response
        return self

    def active(self, op=None):
        self.ready(op)
        request = self.sender.execute(self.host_origin + .02)
        response, accepted = self.exchange(request, .03)
        assert accepted, response
        return self

    def inject(self, t=.04, signals=None):
        self.receiver.inject_test_sample("mission:0:3", signals or {"servo_closed": True}, self.device_origin + t)

    def sample(self, sent=.05, arrived=.06, delay=0.):
        request = self.sender.sample(self.host_origin + sent)
        return self.exchange(request, arrived, delay=delay)


class ManipulatorContractTests(unittest.TestCase):
    def test_all_phase_action_pairs_and_boundaries(self):
        for phase, actions in PHASE_ACTIONS.items():
            for action in actions:
                for timeout in (1, 4000):
                    self.assertEqual(action, validate_operation(operation(action=action, phase=phase, timeout_ms=timeout))["action"])

    def test_operation_rejects_unknown_types_fields_and_navigation(self):
        mutations = [{"phase": "approach"}, {"phase": []}, {"action": "PWM"},
                     {"action": []}, {"timeout_ms": True}, {"timeout_ms": 0},
                     {"timeout_ms": 4001}, {"timeout_ms": 1.0}, {"command_id": ""},
                     {"command_id": "x" * 129}, {"command_id": "a\n"}, {"pin": 4}]
        for changes in mutations:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_operation(operation() | changes)

    def test_mission_intent_copy_is_not_dispatch(self):
        intent = {"robot_id": "B1", "session_id": "mission", "command_id": "mission:0:3", "phase": "close_servo",
                  "servo_intent": "gripper_close", "device_io": False, "dispatch_enabled": False, "fault": None}
        result = operation_from_intent(intent, timeout_ms=1000)
        self.assertEqual(operation(), result)
        result["action"] = "changed"
        self.assertEqual("gripper_close", intent["servo_intent"])
        for changes in ({"dispatch_enabled": True}, {"device_io": True}, {"fault": "stop"},
                        {"phase": "approach"}, {"session_id": "bad\n"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                operation_from_intent(intent | changes, timeout_ms=1000)

    def test_wire_v1_does_not_accept_draft(self):
        from robo_control.fake_receiver import FakeRobotReceiver
        h = Harness()
        request = h.sender.begin(operation(), 10.)
        response = FakeRobotReceiver("B1").receive(request, 1000.)
        self.assertEqual("rejected", response["result"])

    def test_invalid_configuration(self):
        for robot_id in (None, "", "bad id"):
            with self.assertRaises(ValueError):
                ManipulatorSender(robot_id, session_id="s")
        with self.assertRaises(ValueError):
            ManipulatorSender("B1", session_id="")
        for actions in (set(), {"PWM"}, ["hold"]):
            with self.assertRaises(ValueError):
                FakeManipulatorReceiver("B1", supported_actions=actions)


class ManipulatorReceiverTests(unittest.TestCase):
    def test_boot_and_offer_have_no_action(self):
        h = Harness()
        self.assertIsNone(h.receiver.snapshot()["requested_action"])
        h.ready()
        self.assertIsNone(h.receiver.snapshot()["requested_action"])
        self.assertEqual(0, h.receiver.accepted_operations)

    def test_explicit_capability_required(self):
        h = Harness()
        h.receiver = FakeManipulatorReceiver("B1", supported_actions={"hold"})
        response, accepted = h.exchange(h.sender.begin(operation(), 10.), .01)
        self.assertFalse(accepted)
        self.assertEqual("unsupported_action", response["reason"])

    def test_four_robot_addresses_and_independent_clock_origins(self):
        for i, robot_id in enumerate(("H1", "H2", "B1", "B2")):
            for host, device in ((10., 10000. + i), (90000., float(i))):
                h = Harness(robot_id, host, device).active()
                h.inject()
                response, accepted = h.sample()
                self.assertTrue(accepted)
                self.assertEqual(robot_id, response["robot_id"])
                self.assertEqual(1, h.receiver.accepted_operations)

    def test_wrong_robot_does_not_cancel_other_endpoint(self):
        h = Harness().active()
        request = h.sender.sample(10.04)
        request["robot_id"] = "B2"
        response = h.receiver.receive(request, 1000.05)
        self.assertEqual("wrong_robot", response["reason"])
        self.assertEqual("active", h.receiver.state)

    def test_delay_does_not_refresh_execution_permit(self):
        h = Harness().ready()
        request = h.sender.execute(10.02)
        response = h.receiver.receive(request, 1000.31)
        self.assertEqual("rejected", response["result"])
        self.assertEqual(0, h.receiver.accepted_operations)

    def test_short_timeout_expires_before_execution(self):
        h = Harness().ready(operation(timeout_ms=15))
        request = h.sender.execute(10.011)
        response = h.receiver.receive(request, 1000.03)
        self.assertEqual("operation_expired_before_execute", response["reason"])

    def test_duplicate_execute_is_never_applied_twice(self):
        h = Harness().ready()
        request = h.sender.execute(10.02)
        h.exchange(request, .03)
        response = h.receiver.receive(request, 1000.04)
        self.assertEqual("rejected", response["result"])
        self.assertEqual(1, h.receiver.accepted_operations)
        self.assertIsNone(h.receiver.snapshot()["requested_action"])

    def test_new_sequence_cannot_reuse_consumed_permit(self):
        h = Harness().ready()
        request = h.sender.execute(10.02)
        h.exchange(request, .03)
        request["seq"] += 1
        self.assertEqual("rejected", h.receiver.receive(request, 1000.04)["result"])
        self.assertEqual(1, h.receiver.accepted_operations)

    def test_reconnect_same_operation_rejected_even_new_host_instance(self):
        h = Harness().active()
        another = ManipulatorSender("B1", session_id="mission")
        response = h.receiver.receive(another.begin(operation(), 11.), 1000.1)
        self.assertEqual("already_executed_or_history_full", response["reason"])
        self.assertEqual(1, h.receiver.accepted_operations)

    def test_old_context_cannot_authorize_new_operation(self):
        h = Harness().ready()
        old = h.sender.execute(10.02)
        another = ManipulatorSender("B1", session_id="mission2")
        h.receiver.receive(another.begin(operation(command_id="other"), 0.), 1000.03)
        self.assertEqual("rejected", h.receiver.receive(old, 1000.04)["result"])
        self.assertEqual(0, h.receiver.accepted_operations)

    def test_wrong_operation_or_boot_or_link_fails_closed(self):
        for field in ("host_session_id", "boot_id", "link_id", "hello_id", "operation"):
            h = Harness().ready()
            request = h.sender.execute(10.02)
            request[field] = operation(command_id="other") if field == "operation" else "wrong"
            self.assertEqual("rejected", h.receiver.receive(request, 1000.03)["result"])
            self.assertEqual("closed", h.receiver.state)

    def test_sample_requests_never_extend_watchdog(self):
        h = Harness().active()
        deadline = h.receiver.snapshot()["deadline_receiver_s"]
        h.sample()
        self.assertEqual(deadline, h.receiver.snapshot()["deadline_receiver_s"])
        h.receiver.tick(deadline)
        self.assertEqual("closed", h.receiver.state)
        self.assertEqual("operation_watchdog", h.receiver.reason)

    def test_cancel_stop_estop_preempt_pending_execute(self):
        for kind in ("cancel", "stop", "estop"):
            h = Harness().ready()
            delayed = h.sender.execute(10.02)
            stop = h.sender.stop(10.03, kind=kind)
            response, accepted = h.exchange(stop, .04)
            self.assertTrue(accepted)
            self.assertEqual("closed", response["state"])
            self.assertEqual("rejected", h.receiver.receive(delayed, 1000.05)["result"])
            self.assertEqual(0, h.receiver.accepted_operations)

    def test_old_sequence_stop_is_idempotent(self):
        h = Harness().active()
        request = h.sender.stop(10.04)
        request["seq"] = 1
        for now in (1000.05, 1000.06):
            self.assertEqual("accepted", h.receiver.receive(request, now)["result"])
        self.assertIsNone(h.receiver.snapshot()["requested_action"])

    def test_estop_not_reset_by_hello_stop_or_local_reset_alone(self):
        h = Harness().active()
        h.exchange(h.sender.stop(10.04, kind="estop"), .05)
        hello = h.sender.begin(operation(command_id="new"), 10.06)
        self.assertEqual("rejected", h.receiver.receive(hello, 1000.07)["result"])
        h.receiver.reset_emergency_stop(1000.08)
        self.assertIsNone(h.receiver.snapshot()["requested_action"])
        self.assertEqual("closed", h.receiver.state)

    def test_disconnect_invalidates_action(self):
        h = Harness().active()
        h.receiver.disconnect(1000.04)
        self.assertIsNone(h.receiver.snapshot()["requested_action"])

    def test_reboot_changes_boot_id_and_rejects_old_execute(self):
        h = Harness().ready()
        request = h.sender.execute(10.02)
        other = FakeManipulatorReceiver("B1", supported_actions={"gripper_close"})
        self.assertNotEqual(h.receiver.boot_id, other.boot_id)
        self.assertEqual("rejected", other.receive(request, 0.)["result"])

    def test_schema_faults_clear_active_action(self):
        for changes in ({"version": True}, {"seq": True}, {"seq": MAX_SEQUENCE + 1},
                        {"type": []}, {"ttl_ms": 999}, {"protocol": "robo-wire"}):
            h = Harness().active()
            request = h.sender.sample(10.04) | changes
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                h.receiver.receive(request, 1000.05)
            self.assertEqual("closed", h.receiver.state)

    def test_malformed_frame_latches_closed(self):
        for frame in (b'{}', b'{"x":1,"x":2}\n', b'{"x":NaN}\n', b'\xff\n'):
            h = Harness().active()
            with self.assertRaises(WireError):
                h.receiver.receive_frame(frame, 1000.04)
            self.assertEqual("closed", h.receiver.state)

    def test_fragmented_frame_does_not_execute_before_lf(self):
        h = Harness().ready()
        raw = encode_frame(h.sender.execute(10.02))
        decoder = FrameDecoder()
        for byte in raw[:-1]:
            self.assertEqual([], decoder.feed(bytes([byte])))
        self.assertEqual(0, h.receiver.accepted_operations)
        messages = decoder.feed(raw[-1:])
        self.assertEqual(1, len(messages))
        h.receiver.receive(messages[0], 1000.03)
        self.assertEqual(1, h.receiver.accepted_operations)

    def test_receiver_history_capacity_does_not_evict_replay_protection(self):
        h = Harness()
        h.receiver._consumed = {("mission", str(i)) for i in range(MAX_OPERATIONS)}
        response = h.receiver.receive(h.sender.begin(operation(), 10.), 1000.)
        self.assertEqual("rejected", response["result"])
        self.assertEqual(MAX_OPERATIONS, len(h.receiver._consumed))


class ManipulatorSenderTests(unittest.TestCase):
    def test_ack_is_not_physical_success_or_sensor_reading(self):
        h = Harness().active()
        self.assertIsNone(h.sender.snapshot()["evidence"])
        self.assertIsNone(h.sender.snapshot()["physical_success"])
        response, accepted = h.sample()
        self.assertTrue(accepted)
        self.assertIsNone(response["signals"])
        self.assertIsNone(h.sender.snapshot()["evidence"])

    def test_no_execute_before_handshake_or_after_offer_expiry(self):
        h = Harness()
        with self.assertRaises(ValueError):
            h.sender.execute(10.)
        h.ready()
        with self.assertRaises(ValueError):
            h.sender.execute(10.311)
        self.assertEqual(0, h.receiver.accepted_operations)

    def test_no_automatic_retry_after_lost_execute_ack(self):
        h = Harness().ready()
        request = h.sender.execute(10.02)
        late = h.receiver.receive(request, 1000.03)
        h.sender.poll(10.32)
        self.assertEqual("closed", h.sender.state)
        self.assertFalse(h.sender.accept_response(late, 10.33))
        with self.assertRaises(ValueError):
            h.sender.begin(operation(), 10.34)
        self.assertEqual(1, h.receiver.accepted_operations)
        h.receiver.tick(1001.01)
        self.assertIsNone(h.receiver.snapshot()["requested_action"])

    def test_only_one_pending_request_and_explicit_stop_before_next(self):
        h = Harness().active()
        h.sender.sample(10.04)
        with self.assertRaises(ValueError):
            h.sender.sample(10.05)
        with self.assertRaises(ValueError):
            h.sender.begin(operation(command_id="new"), 10.05)

    def test_cancel_before_hello_ack_authorizes_nothing(self):
        h = Harness()
        hello = h.sender.begin(operation(), 10.)
        self.assertIsNone(h.sender.stop(10.01, kind="cancel"))
        response = h.receiver.receive(hello, 1000.02)
        self.assertFalse(h.sender.accept_response(response, 10.03))
        h.receiver.tick(1000.32)
        self.assertEqual(0, h.receiver.accepted_operations)
        self.assertEqual("closed", h.receiver.state)

    def test_stop_ack_timeout_cannot_claim_remote_stop(self):
        h = Harness().active()
        stop = h.sender.stop(10.04)
        response = h.receiver.receive(stop, 1000.05)
        self.assertFalse(h.sender.accept_response(response, 10.34))
        self.assertIsNone(h.sender.snapshot()["physical_success"])

    def test_wrong_response_identity_schema_or_synthetic_flag_closes(self):
        for changes in ({"robot_id": "B2"}, {"host_session_id": "other"}, {"boot_id": "other"},
                        {"link_id": "other"}, {"hello_id": "other"}, {"seq": 100},
                        {"version": True}, {"seq": True}, {"synthetic": False},
                        {"device_io": True}, {"hardware_ready": True}, {"physical_success": True},
                        {"execution": "success"}, {"request_type": "sample"}, {"extra": 0},
                        {"operation": operation(command_id="other")}):
            h = Harness().ready()
            request = h.sender.execute(10.02)
            response = h.receiver.receive(request, 1000.03) | changes
            with self.subTest(changes=changes):
                self.assertFalse(h.sender.accept_response(response, 10.04))
                self.assertEqual("closed", h.sender.state)

    def test_ack_cannot_smuggle_sensor_success(self):
        h = Harness().ready()
        response = h.receiver.receive(h.sender.execute(10.02), 1000.03)
        response["signals"] = {"servo_closed": True}
        self.assertFalse(h.sender.accept_response(response, 10.04))

    def test_duplicate_ack_closes_instead_of_reauthorizing(self):
        h = Harness().ready()
        request = h.sender.execute(10.02)
        response, _ = h.exchange(request, .03)
        self.assertFalse(h.sender.accept_response(response, 10.04))
        self.assertEqual("closed", h.sender.state)

    def test_explicit_next_command_is_allowed_after_stop(self):
        h = Harness().active()
        h.exchange(h.sender.stop(10.04), .05)
        response, accepted = h.exchange(h.sender.begin(operation(command_id="mission:0:4"), 10.06), .07)
        self.assertTrue(accepted, response)
        self.assertEqual("ready", h.sender.state)
        self.assertEqual(1, h.receiver.accepted_operations)

    def test_sender_attempt_history_is_bounded_without_eviction(self):
        h = Harness()
        h.sender._attempted = {str(i) for i in range(MAX_OPERATIONS)}
        with self.assertRaises(ValueError):
            h.sender.begin(operation(), 10.)

    def test_sequence_exhaustion_closes(self):
        h = Harness().ready()
        h.sender._seq = MAX_SEQUENCE
        with self.assertRaises(ValueError):
            h.sender.execute(10.02)
        self.assertEqual("closed", h.sender.state)

    def test_bad_clocks_latch_and_cannot_resume(self):
        for value in (-1, True, float("nan"), float("inf"), 0., 10**1000):
            for who in ("sender", "receiver"):
                h = Harness().active()
                obj = getattr(h, who)
                tick = obj.poll if who == "sender" else obj.tick
                with self.subTest(value=str(value)[:20], who=who), self.assertRaises(ValueError):
                    tick(value)
                with self.assertRaises(ValueError):
                    tick(10000.)
                self.assertEqual("closed", obj.state)

    def test_malformed_response_clears_evidence(self):
        h = Harness().active()
        h.inject()
        h.sample()
        self.assertFalse(h.sender.accept_frame(b'{}', 10.07))
        self.assertIsNone(h.sender.snapshot()["evidence"])


class ManipulatorSensorTests(unittest.TestCase):
    def test_fresh_explicit_sample_is_only_synthetic_diagnostic(self):
        h = Harness().active()
        h.inject(signals={"servo_closed": True, "gripper_present": False})
        response, accepted = h.sample()
        self.assertTrue(accepted)
        self.assertEqual(SIGNALS, response["signals"].keys())
        self.assertNotIn("object_settled", response["signals"])
        evidence = h.sender.snapshot()["evidence"]
        self.assertTrue(evidence["signals"]["servo_closed"])
        self.assertFalse(evidence["signals"]["gripper_present"])
        self.assertIsNone(evidence["signals"]["hopper_loaded"])
        self.assertTrue(evidence["synthetic"])
        self.assertFalse(evidence["mission_feedback_eligible"])
        self.assertNotIn("observed_at_s", evidence)

    def test_sample_is_detached_from_input_and_snapshots(self):
        h = Harness().active()
        signals = {"servo_closed": True}
        h.inject(signals=signals)
        signals["servo_closed"] = False
        h.sample()
        snapshot = h.sender.snapshot()
        snapshot["evidence"]["signals"]["servo_closed"] = False
        self.assertTrue(h.sender.snapshot()["evidence"]["signals"]["servo_closed"])

    def test_sensor_capture_requires_post_execute_operation_identity(self):
        for command_id, now, values in (("other", 1000.04, {"servo_closed": True}),
            ("mission:0:3", 1000.03, {"servo_closed": True}),
            ("mission:0:3", 1000.04, {"object_settled": True}),
            ("mission:0:3", 1000.04, {"servo_closed": 1}), ("mission:0:3", 1000.04, {})):
            h = Harness().active()
            with self.assertRaises(ValueError):
                h.receiver.inject_test_sample(command_id, values, now)
            self.assertEqual("closed", h.receiver.state)

    def test_duplicate_sample_does_not_get_new_timestamp(self):
        h = Harness().active()
        h.inject()
        self.assertTrue(h.sample()[1])
        response, accepted = h.sample(.07, .08)
        self.assertFalse(accepted)
        self.assertEqual(1, response["sample_sequence"])
        self.assertGreaterEqual(response["sample_age_ms"], 40)
        self.assertIsNone(h.sender.snapshot()["evidence"])

    def test_new_sample_sequence_is_accepted(self):
        h = Harness().active()
        h.inject()
        h.sample()
        h.inject(.07, {"servo_closed": False})
        self.assertTrue(h.sample(.08, .09)[1])
        self.assertFalse(h.sender.snapshot()["evidence"]["signals"]["servo_closed"])

    def test_old_sample_or_delayed_ack_rejected(self):
        for sent, arrived, delay in ((.24, .25, 0), (.05, .06, .19)):
            h = Harness().active()
            h.inject()
            self.assertFalse(h.sample(sent, arrived, delay)[1])
            self.assertIsNone(h.sender.snapshot()["evidence"])

    def test_evidence_expires_without_new_requests(self):
        h = Harness().active()
        h.inject()
        h.sample()
        deadline = h.sender.snapshot()["evidence"]["valid_until_host_s"]
        self.assertIsNone(h.sender.poll(deadline)["evidence"])

    def test_cancel_invalidates_previously_fresh_evidence(self):
        h = Harness().active()
        h.inject()
        h.sample()
        h.sender.stop(10.07, kind="cancel")
        self.assertIsNone(h.sender.snapshot()["evidence"])

    def test_invalid_sample_responses_fail_closed(self):
        for changes in ({"sample_age_ms": -1}, {"sample_age_ms": True}, {"sample_age_ms": .5},
                        {"sample_sequence": True}, {"sample_sequence": 0}, {"signals": {}},
                        {"signals": {k: 1 for k in SIGNALS}}, {"signals": None}):
            h = Harness().active()
            h.inject()
            response = h.receiver.receive(h.sender.sample(10.05), 1000.06)
            with self.subTest(changes=changes):
                self.assertFalse(h.sender.accept_response(response | changes, 10.07))

    def test_sensor_counter_exhaustion_stops(self):
        h = Harness().active()
        h.receiver._sample_seq = MAX_SEQUENCE
        with self.assertRaises(ValueError):
            h.inject()

    def test_response_copy_cannot_mutate_device_sample(self):
        h = Harness().active()
        h.inject()
        response, _ = h.sample()
        response["signals"]["servo_closed"] = False
        request = h.sender.sample(10.07)
        later = h.receiver.receive(copy.deepcopy(request), 1000.08)
        self.assertTrue(later["signals"]["servo_closed"])


if __name__ == "__main__":
    unittest.main()
