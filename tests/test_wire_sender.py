from __future__ import annotations

import copy
import math
import unittest

from robo_control.control_loop import ClosedLoopController
from robo_control.fake_receiver import FakeRobotReceiver
from robo_control.fleet import DEFAULT_ROLES
from robo_control.wire_sender import WireCommandSender


def packet(*, issued=10.04, sequence=1, session="host-1", pause=False, emergency=False):
    positions = {rid: (0., i * 1000.) for i, rid in enumerate(DEFAULT_ROLES)}
    controller = ClosedLoopController(
        roles=DEFAULT_ROLES, session_id=session, drive_model="differential_body",
        goals={rid: {"x_mm": 600., "y_mm": position[1]} for rid, position in positions.items()})
    controller.sequence = sequence - 1
    controller.set_paused(pause)
    if emergency:
        controller.emergency_stop()
    observation = {"source_name": "camera-test", "sequence": sequence, "captured_at_s": issued,
                   "observation_usable": True, "stop_required": False,
                   "tracks": [{"robot_id": rid, "robot_center_mm": list(position),
                               "heading_rad": 0., "observed_at_s": issued, "velocity_mm_s": [0., 0.],
                               "angular_velocity_rad_s": 0., "state": "observed", "valid_for_control": True}
                              for rid, position in positions.items()]}
    return controller.tick(observation, issued)


def armed(robot="H1", *, host=10., receiver=1000.):
    sender = WireCommandSender(robot, session_id="host-1")
    device = FakeRobotReceiver(robot, boot_id="boot-1")
    challenge = device.receive(sender.hello(host), receiver)
    sender.accept_response(challenge, host + .01)
    reply = device.receive(sender.arm(host + .02), receiver + .02)
    sender.accept_response(reply, host + .03)
    return sender, device


class WireSenderTests(unittest.TestCase):
    def test_real_differential_controller_and_independent_clock_receiver(self):
        sender, receiver = armed()
        source = packet()
        message = sender.drive_from_packet(source, 10.04)
        self.assertEqual(message["type"], "drive")
        self.assertEqual(message["v_mm_s"], source["robots"][0]["forward_velocity_mm_s"])
        self.assertEqual(message["ttl_ms"], 300)
        response = receiver.receive(message, 1000.04)
        state = sender.accept_response(response, 10.05)
        self.assertTrue(state["permit_available"])
        self.assertEqual(state["state"], "armed")
        self.assertEqual(state["execution"], "unknown")
        for flag in ("device_io", "hardware_ready", "motion_permitted"):
            self.assertIs(state[flag], False)

    def test_constructor_rejects_invalid_fleet_tokens_timeout(self):
        cases = ({"session_id": "space is invalid"}, {"session_id": "한글"},
                 {"session_id": True}, {"robot_ids": ("H1", "H1")}, {"robot_ids": []},
                 {"robot_ids": {"H1": "beaver"}}, {"robot_ids": ["bad robot"]},
                 {"ack_timeout_s": True}, {"ack_timeout_s": .301}, {"ack_timeout_s": 0})
        for kwargs in cases:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                WireCommandSender("H1", **{"session_id": "host-1", **kwargs})

    def test_initial_state_never_claims_execution_or_permit(self):
        sender = WireCommandSender("H1", session_id="host-1")
        self.assertFalse(sender.snapshot()["permit_available"])
        self.assertEqual(sender.snapshot()["execution"], "unknown")
        with self.assertRaises(ValueError):
            sender.arm(0.)

    def test_old_controller_packet_cannot_be_wrapped_in_a_new_permit(self):
        sender, _ = armed()
        with self.assertRaisesRegex(ValueError, "predates"):
            sender.drive_from_packet(packet(issued=10.02), 10.04)
        self.assertEqual(sender.snapshot()["state"], "fault")
        self.assertFalse(sender.snapshot()["permit_available"])

    def test_each_drive_ack_requires_newer_causally_created_controller_output(self):
        sender, receiver = armed()
        reply = receiver.receive(sender.drive_from_packet(packet(), 10.04), 1000.04)
        sender.accept_response(reply, 10.06)
        with self.assertRaisesRegex(ValueError, "predates"):
            sender.drive_from_packet(packet(issued=10.05, sequence=2), 10.07)

    def test_permit_is_consumed_on_send_and_cannot_retry_in_flight(self):
        sender, _ = armed()
        sender.drive_from_packet(packet(), 10.04)
        self.assertFalse(sender.snapshot()["permit_available"])
        with self.assertRaisesRegex(ValueError, "in_flight"):
            sender.drive_from_packet(packet(issued=10.05, sequence=2), 10.05)
        self.assertEqual(sender.snapshot()["state"], "fault")

    def test_ack_timeout_equality_latches_and_late_ack_cannot_reopen(self):
        sender, receiver = armed()
        reply = receiver.receive(sender.drive_from_packet(packet(), 10.04), 1000.04)
        self.assertEqual(sender.poll(10.34)["state"], "fault")
        self.assertEqual(sender.accept_response(reply, 10.35)["state"], "fault")
        self.assertFalse(sender.snapshot()["permit_available"])
        self.assertEqual(receiver.tick(1000.32)["v_mm_s"], 0.)

    def test_no_response_no_retry_poll_only_diagnostics(self):
        sender = WireCommandSender("H1", session_id="host-1")
        sender.hello(0.)
        self.assertEqual(sender.poll(.299)["pending_request"], "hello")
        state = sender.poll(.3)
        self.assertEqual(state["state"], "fault")
        self.assertIsNone(state["pending_request"])
        self.assertEqual(state["wire_sequence"], 0)

    def test_explicit_hello_recovers_but_cannot_reuse_consumed_controller_sequence(self):
        sender, receiver = armed()
        sender.drive_from_packet(packet(), 10.04)
        sender.poll(10.34)
        response = receiver.receive(sender.hello(10.4), 1000.4)
        sender.accept_response(response, 10.41)
        sender.accept_response(receiver.receive(sender.arm(10.42), 1000.42), 10.43)
        with self.assertRaisesRegex(ValueError, "sequence"):
            sender.drive_from_packet(packet(issued=10.44, sequence=1), 10.44)

    def test_duplicate_ack_cannot_grant_second_permit(self):
        sender, receiver = armed()
        reply = receiver.receive(sender.drive_from_packet(packet(), 10.04), 1000.04)
        sender.accept_response(reply, 10.05)
        self.assertEqual(sender.accept_response(reply, 10.06)["state"], "fault")

    def test_foreign_robot_session_boot_link_sequence_and_hello_ack_rejected(self):
        for key, value in (("robot_id", "H2"), ("host_session_id", "other"), ("boot_id", "other"),
                           ("link_id", "other"), ("seq", 100), ("hello_id", "other")):
            sender, receiver = armed()
            reply = receiver.receive(sender.drive_from_packet(packet(), 10.04), 1000.04)
            reply[key] = value
            with self.subTest(key=key):
                self.assertEqual(sender.accept_response(reply, 10.05)["state"], "fault")
                self.assertFalse(sender.snapshot()["permit_available"])

    def test_old_hello_reply_does_not_complete_new_handshake(self):
        sender = WireCommandSender("H1", session_id="host-1")
        receiver = FakeRobotReceiver("H1")
        old = receiver.receive(sender.hello(1.), 900.)
        sender.hello(1.02)
        self.assertEqual(sender.accept_response(old, 1.03)["state"], "fault")

    def test_malformed_and_hardware_claiming_responses_fail_closed(self):
        for mutate in (lambda r: r.update(version=True), lambda r: r.update(extra="x"),
                       lambda r: r.pop("synthetic"), lambda r: r.update(device_io=True),
                       lambda r: r.update(execution="completed"), lambda r: r.update(lease_remaining_ms=True),
                       lambda r: r.update(omega_rad_s=float("nan")), lambda r: r.update(v_mm_s=10**400)):
            sender, receiver = armed()
            reply = receiver.receive(sender.drive_from_packet(packet(), 10.04), 1000.04)
            mutate(reply)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                sender.accept_response(reply, 10.05)
            self.assertEqual(sender.snapshot()["state"], "fault")

    def test_acceptance_must_match_requested_setpoint_and_state(self):
        for mutate in (lambda r: r.update(v_mm_s=0.), lambda r: r.update(state="disarmed"),
                       lambda r: r.update(permit_id=None), lambda r: r.update(challenge_id="strange")):
            sender, receiver = armed()
            reply = receiver.receive(sender.drive_from_packet(packet(), 10.04), 1000.04)
            mutate(reply)
            with self.subTest(mutate=mutate):
                self.assertEqual(sender.accept_response(reply, 10.05)["state"], "fault")

    def test_receiver_rejection_does_not_count_as_execution(self):
        sender, receiver = armed()
        message = sender.drive_from_packet(packet(), 10.04)
        receiver.disconnect(1000.035)
        state = sender.accept_response(receiver.receive(message, 1000.04), 10.05)
        self.assertEqual(state["state"], "fault")
        self.assertEqual(state["execution"], "unknown")

    def test_stop_can_supersede_pending_drive_without_a_permit(self):
        sender, receiver = armed()
        old_reply = receiver.receive(sender.drive_from_packet(packet(), 10.04), 1000.04)
        message = sender.stop(10.05)
        self.assertEqual(message["type"], "stop")
        stopped = receiver.receive(message, 1000.05)
        sender.accept_response(stopped, 10.06)
        self.assertEqual(receiver.snapshot()["v_mm_s"], 0.)
        self.assertFalse(sender.snapshot()["permit_available"])
        self.assertEqual(sender.accept_response(old_reply, 10.07)["state"], "fault")

    def test_controller_hold_and_estop_are_stop_requests(self):
        for source, kind in ((packet(pause=True), "stop"), (packet(emergency=True), "estop")):
            sender, receiver = armed()
            message = sender.drive_from_packet(source, 10.04)
            with self.subTest(kind=kind):
                self.assertEqual(message["type"], kind)
                state = sender.accept_response(receiver.receive(message, 1000.04), 10.05)
                self.assertFalse(state["permit_available"])
                self.assertEqual(state["state"], "estop" if kind == "estop" else "disarmed")

    def test_normal_stop_after_estop_does_not_clear_the_receiver_latch(self):
        sender, receiver = armed()
        sender.accept_response(receiver.receive(sender.stop(10.04, emergency=True), 1000.04), 10.05)
        state = sender.accept_response(receiver.receive(sender.stop(10.06), 1000.06), 10.07)
        self.assertEqual(state["state"], "estop")
        self.assertFalse(state["permit_available"])
        self.assertEqual(receiver.snapshot()["state"], "estop")

    def test_all_fleet_commands_validated_even_other_robot_last_row(self):
        for mutate in (lambda p: p["robots"].pop(), lambda p: p["robots"][-1].update(robot_id="H1"),
                       lambda p: p["robots"][-1].update(robot_id="stranger"),
                       lambda p: p["robots"][-1].update(forward_velocity_mm_s=181.),
                       lambda p: p["robots"][-1].update(angular_velocity_rad_s=1.51),
                       lambda p: p["robots"][-1].update(wheel_velocity_rad_s=[0., 0., 0., 0.]),
                       lambda p: p["robots"][-1].update(velocity_world_mm_s=[0., 100.]),
                       lambda p: p["robots"][-1].update(pose_heading_rad=float("nan"))):
            sender, _ = armed()
            source = packet()
            mutate(source)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                sender.drive_from_packet(source, 10.04)
            self.assertFalse(sender.snapshot()["permit_available"])
            self.assertIsNone(sender.snapshot()["pending_request"])
            self.assertEqual(sender.snapshot()["controller_sequence"], 1)

    def test_wrong_source_envelopes_flags_times_and_nonfinite_values(self):
        for mutate in (lambda p: p.update(session_id="other"), lambda p: p.update(drive_model="mecanum"),
                       lambda p: p.update(schema_version=True), lambda p: p.update(sequence=True),
                       lambda p: p.update(sequence=2**53), lambda p: p.update(ttl_s=.301),
                       lambda p: p.update(ttl_s=.0009), lambda p: p.update(issued_at_s=10.05),
                       lambda p: p.update(issued_at_s=9.), lambda p: p.update(issued_at_s=10**400),
                       lambda p: p.update(hardware_ready=True), lambda p: p.update(motion_permitted=True),
                       lambda p: p.update(device_io=True), lambda p: p.update(emergency_stop="false"),
                       lambda p: p.update(mock_motion_permitted=1), lambda p: p.update(conflicts={})):
            sender, _ = armed()
            source = packet()
            mutate(source)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                sender.drive_from_packet(source, 10.04)
            self.assertEqual(sender.snapshot()["state"], "fault")

    def test_invalid_clock_fail_closes_and_no_implicit_resume(self):
        for bad in (True, -1, float("nan"), float("inf"), 9., 10**400):
            sender, _ = armed()
            with self.subTest(now=bad), self.assertRaises(ValueError):
                sender.poll(bad)
            self.assertFalse(sender.snapshot()["permit_available"])
            with self.assertRaises(ValueError):
                sender.drive_from_packet(packet(), 10.04)

    def test_ttl_floors_milliseconds_without_extending_source_command(self):
        sender, receiver = armed()
        source = packet()
        source["ttl_s"] = .0409
        message = sender.drive_from_packet(source, 10.04)
        self.assertEqual(message["ttl_ms"], 40)
        receiver.receive(message, 1000.04)
        self.assertEqual(receiver.tick(1000.06)["v_mm_s"], 0.)

    def test_ttl_exact_host_deadline_rejects_even_permit_still_young(self):
        sender, _ = armed()
        source = packet()
        source["ttl_s"] = .01
        with self.assertRaisesRegex(ValueError, "ttl"):
            sender.drive_from_packet(source, 10.05)

    def test_status_does_not_replace_or_renew_permit(self):
        sender, receiver = armed()
        first = sender.snapshot()["last_response"]["permit_id"]
        response = receiver.receive(sender.status(10.1), 1000.1)
        response["permit_id"] = "forged-new-permit"
        sender.accept_response(response, 10.11)
        request = sender.drive_from_packet(packet(issued=10.12), 10.12)
        self.assertEqual(request["permit_id"], first)

    def test_status_does_not_extend_authorization_age(self):
        sender, receiver = armed()
        sender.accept_response(receiver.receive(sender.status(10.1), 1000.1), 10.11)
        self.assertEqual(sender.poll(10.33)["state"], "fault")

    def test_response_and_snapshot_aliases_do_not_mutate_pending_or_permit(self):
        sender, receiver = armed()
        source = packet()
        original = copy.deepcopy(source)
        request = sender.drive_from_packet(source, 10.04)
        reply = receiver.receive(request, 1000.04)
        request["v_mm_s"] = 0.
        snapshot = sender.accept_response(reply, 10.05)
        snapshot["last_response"]["permit_id"] = "bad"
        reply["permit_id"] = "also-bad"
        self.assertNotEqual(sender.snapshot()["last_response"]["permit_id"], "bad")
        self.assertNotEqual(sender.snapshot()["last_response"]["permit_id"], "also-bad")
        self.assertEqual(source, original)

    def test_four_senders_are_independent_and_wrong_ack_does_not_change_others(self):
        links = {rid: armed(rid) for rid in DEFAULT_ROLES}
        replies = {}
        for rid, (sender, receiver) in links.items():
            replies[rid] = receiver.receive(sender.drive_from_packet(packet(), 10.04), 1000.04)
        self.assertEqual(links["H1"][0].accept_response(replies["H2"], 10.05)["state"], "fault")
        for rid in ("H2", "B1", "B2"):
            self.assertEqual(links[rid][0].accept_response(replies[rid], 10.05)["state"], "armed")

    def test_differential_turn_and_reverse_body_velocity_are_not_lateral(self):
        sender, receiver = armed()
        source = packet()
        row = source["robots"][0]
        row.update(forward_velocity_mm_s=-50., pose_heading_rad=math.pi / 2,
                   velocity_world_mm_s=[0., -50.], angular_velocity_rad_s=1.)
        message = sender.drive_from_packet(source, 10.04)
        self.assertEqual((message["v_mm_s"], message["omega_rad_s"]), (-50., 1.))
        self.assertEqual(receiver.receive(message, 1000.04)["result"], "accepted")


if __name__ == "__main__":
    unittest.main()
