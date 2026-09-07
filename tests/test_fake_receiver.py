from __future__ import annotations

import copy
import math
import unittest

from robo_control.control_loop import ControlLimits
from robo_control.fake_receiver import FakeRobotReceiver
from robo_control.wire_codec import decode_frame, encode_frame


class ReceiverHarness:
    def __init__(self, robot_id="H1", *, start=100.):
        self.receiver = FakeRobotReceiver(robot_id)
        self.robot_id = robot_id
        self.now = start
        self.seq = 0
        self.reply = None

    def send(self, message, now=None):
        self.now = self.now if now is None else now
        self.reply = decode_frame(self.receiver.receive_frame(encode_frame(message), self.now))
        return self.reply

    def hello(self, *, session="host-session", now=None):
        self.seq = 0
        return self.send({"protocol": "robo-wire", "version": 1, "type": "hello",
            "robot_id": self.robot_id, "host_session_id": session, "hello_id": "hello-id"}, now)

    def message(self, kind, **fields):
        self.seq += 1
        return {"protocol": "robo-wire", "version": 1, "type": kind, "robot_id": self.robot_id,
            "host_session_id": self.reply["host_session_id"], "boot_id": self.reply["boot_id"],
            "link_id": self.reply["link_id"], "seq": self.seq, **fields}

    def arm(self, *, now=None):
        return self.send(self.message("arm", challenge_id=self.reply["challenge_id"]), now)

    def ready(self):
        self.hello()
        self.arm(now=self.now+.01)
        return self

    def drive(self, **fields):
        return self.message("drive", **{"permit_id": self.reply["permit_id"], "ttl_ms": 300,
            "v_mm_s": 50., "omega_rad_s": .25, **fields})


class FakeReceiverTests(unittest.TestCase):
    def test_boot_hello_arm_are_zero_and_not_physical_completion(self):
        h = ReceiverHarness()
        self.assertEqual("disarmed", h.receiver.snapshot()["state"])
        self.assertEqual("challenge", h.hello()["result"])
        self.assertEqual("accepted", h.arm()["result"])
        self.assertEqual(0, h.reply["v_mm_s"])
        self.assertTrue(h.reply["synthetic"])
        self.assertEqual("unknown", h.reply["execution"])
        for field in ("device_io", "hardware_ready", "motion_permitted"):
            self.assertIs(False, h.reply[field])
        self.assertIsNone(h.receiver.snapshot()["measured_velocity"])

    def test_delayed_command_does_not_receive_a_new_lifetime(self):
        h = ReceiverHarness().ready()
        permit_time = h.now
        self.assertEqual("accepted", h.send(h.drive(), permit_time+.28)["result"])
        self.assertAlmostEqual(permit_time+.3, h.receiver.snapshot()["command_deadline_receiver_s"])
        self.assertEqual("armed", h.receiver.tick(permit_time+.299)["state"])
        self.assertEqual("disarmed", h.receiver.tick(permit_time+.3)["state"])
        self.assertEqual(0, h.receiver.snapshot()["v_mm_s"])

    def test_short_ttl_is_also_measured_from_permit_not_arrival(self):
        h = ReceiverHarness().ready()
        response = h.send(h.drive(ttl_ms=100), h.now+.1)
        self.assertEqual("rejected", response["result"])
        self.assertEqual("expired_permit", response["reason"])

    def test_arm_challenge_expires_at_exact_deadline(self):
        h = ReceiverHarness()
        h.hello()
        self.assertEqual("rejected", h.arm(now=h.now+.3)["result"])
        self.assertEqual("disarmed", h.receiver.snapshot()["state"])

    def test_duplicate_drive_never_reapplies_or_renews(self):
        h = ReceiverHarness().ready()
        message = h.drive()
        self.assertEqual("accepted", h.send(message)["result"])
        self.assertEqual("rejected", h.send(message, h.now+.01)["result"])
        state = h.receiver.snapshot()
        self.assertEqual(1, state["accepted_drive_count"])
        self.assertEqual(0, state["v_mm_s"])
        self.assertIsNone(state["command_deadline_receiver_s"])

    def test_reordered_command_and_reused_permit_reject(self):
        for reuse in (False, True):
            with self.subTest(reuse=reuse):
                h = ReceiverHarness().ready()
                first = h.drive()
                h.send(first)
                second = h.drive()
                second["permit_id" if reuse else "seq"] = first["permit_id" if reuse else "seq"]
                self.assertEqual("rejected", h.send(second)["result"])
                self.assertEqual(1, h.receiver.snapshot()["accepted_drive_count"])

    def test_rejected_payload_cannot_be_repaired_and_replayed(self):
        h = ReceiverHarness().ready()
        bad = h.drive(v_mm_s=181.)
        self.assertEqual("rejected", h.send(bad)["result"])
        bad["v_mm_s"] = 10.
        self.assertEqual("rejected", h.send(bad)["result"])
        self.assertEqual(0, h.receiver.snapshot()["accepted_drive_count"])

    def test_four_receivers_do_not_accept_another_robots_frame(self):
        fleet = [ReceiverHarness(rid).ready() for rid in ("H1", "H2", "B1", "B2")]
        message = fleet[0].drive()
        for h in fleet:
            before = h.receiver.snapshot()
            reply = h.send(message)
            if h.robot_id == "H1":
                self.assertEqual("accepted", reply["result"])
            else:
                self.assertEqual("wrong_robot", reply["reason"])
                self.assertEqual(before, h.receiver.snapshot())

    def test_wrong_session_link_and_boot_disarm_without_applying(self):
        for field in ("host_session_id", "link_id", "boot_id"):
            with self.subTest(field=field):
                h = ReceiverHarness().ready()
                h.send(h.drive())
                packet = h.drive()
                packet[field] = "foreign"
                self.assertEqual("rejected", h.send(packet)["result"])
                self.assertEqual(0., h.receiver.snapshot()["v_mm_s"])

    def test_unknown_fields_types_and_nonfinite_values_reject(self):
        mutations = [{"version": True}, {"seq": True}, {"seq": 2**53}, {"ttl_ms": True},
            {"ttl_ms": 301}, {"ttl_ms": 0}, {"v_mm_s": True}, {"v_mm_s": math.nan},
            {"v_mm_s": 10**500}, {"omega_rad_s": math.inf}, {"omega_rad_s": 1.51},
            {"lateral_velocity_mm_s": 10.}, {"type": "servo"}, {"protocol": "other"}]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                h = ReceiverHarness().ready()
                packet = h.drive()
                packet.update(mutation)
                # Direct method must also reject invalid numerics, before encoding.
                reply = h.receiver.receive(packet, h.now)
                self.assertEqual("rejected", reply["result"])
                self.assertEqual(0., reply["v_mm_s"])

    def test_stop_takes_priority_even_with_old_sequence(self):
        h = ReceiverHarness().ready()
        h.send(h.drive())
        pending = h.drive()
        stop = h.message("stop", reason="operator")
        stop["seq"] = 1
        self.assertEqual("accepted", h.send(stop)["result"])
        self.assertEqual("accepted", h.send(stop)["result"])
        self.assertEqual("rejected", h.send(pending)["result"])
        self.assertEqual(0, h.receiver.snapshot()["v_mm_s"])

    def test_estop_cannot_be_reset_over_wire_or_by_reconnect(self):
        h = ReceiverHarness().ready()
        h.send(h.drive())
        self.assertEqual("estop", h.send(h.message("estop", reason="test"))["state"])
        self.assertEqual("rejected", h.hello()["result"])
        self.assertEqual("estop", h.receiver.snapshot()["state"])
        h.receiver.reset_emergency_stop(h.now)
        self.assertEqual("disarmed", h.receiver.snapshot()["state"])
        h.hello()
        self.assertEqual("accepted", h.arm()["result"])
        self.assertEqual(0, h.receiver.snapshot()["v_mm_s"])

    def test_status_and_missing_packets_cannot_keep_motion_alive(self):
        h = ReceiverHarness().ready()
        issued = h.now
        h.send(h.drive())
        for i in range(1, 6):
            self.assertEqual("status", h.send(h.message("status"), issued+i*.05)["result"])
        state = h.receiver.tick(issued+.3)
        self.assertEqual("command_watchdog", state["reason"])
        self.assertEqual("disarmed", state["state"])

    def test_timeout_requires_new_handshake_not_just_next_command(self):
        h = ReceiverHarness().ready()
        h.send(h.drive())
        pending = h.drive()
        h.receiver.tick(h.now+.3)
        self.assertEqual("rejected", h.send(pending, h.now+.31)["result"])
        h.hello(now=h.now+.01)
        h.arm()
        self.assertEqual(0, h.reply["v_mm_s"])
        self.assertEqual("accepted", h.send(h.drive())["result"])

    def test_disconnect_and_reconnect_invalidate_inflight_frames(self):
        h = ReceiverHarness().ready()
        h.send(h.drive())
        pending = h.drive()
        old_link = h.reply["link_id"]
        h.receiver.disconnect(h.now)
        h.hello()
        h.arm()
        self.assertNotEqual(old_link, h.reply["link_id"])
        self.assertEqual("rejected", h.send(pending)["result"])
        self.assertEqual(0, h.receiver.snapshot()["v_mm_s"])

    def test_reboot_has_new_identity_and_cannot_replay_old_arm(self):
        h = ReceiverHarness()
        h.hello()
        old = h.message("arm", challenge_id=h.reply["challenge_id"])
        new_receiver = FakeRobotReceiver("H1")
        self.assertNotEqual(h.receiver.boot_id, new_receiver.boot_id)
        self.assertEqual("rejected", new_receiver.receive(old, 0.)["result"])

    def test_replayed_hello_stops_and_mints_new_challenge(self):
        h = ReceiverHarness().ready()
        h.send(h.drive())
        old_link = h.reply["link_id"]
        response = h.hello()
        self.assertNotEqual(old_link, response["link_id"])
        self.assertEqual(0, response["v_mm_s"])

    def test_truncated_and_corrupt_wire_stop_without_partial_command(self):
        for data in (b'{"type":"drive"', b'{}\n{}\n', b'\xff\n',
                     b'{"robot_id":"H1","robot_id":"H2"}\n', b'x'*4097):
            with self.subTest(data=data[:40]):
                h = ReceiverHarness().ready()
                h.send(h.drive())
                response = decode_frame(h.receiver.receive_frame(data, h.now))
                self.assertEqual("malformed_wire", response["reason"])
                self.assertEqual(0, response["v_mm_s"])

    def test_invalid_clock_latches_zero_until_new_receiver_instance(self):
        for stamp in (-1, True, math.inf, math.nan, 99.):
            with self.subTest(stamp=stamp):
                h = ReceiverHarness().ready()
                h.send(h.drive())
                with self.assertRaises(ValueError):
                    h.receiver.tick(stamp)
                self.assertEqual(0, h.receiver.snapshot()["v_mm_s"])
                with self.assertRaises(ValueError):
                    h.receiver.tick(101.)

    def test_snapshot_is_independent_and_does_not_self_expire(self):
        h = ReceiverHarness().ready()
        h.send(h.drive())
        before = h.receiver.snapshot()
        changed = copy.deepcopy(before)
        changed["v_mm_s"] = 999.
        self.assertEqual(before, h.receiver.snapshot())
        h.receiver.tick(h.now+.3)
        self.assertEqual(50, before["v_mm_s"])
        self.assertEqual(0, h.receiver.snapshot()["v_mm_s"])

    def test_custom_short_permit_and_constructor_validation(self):
        for kwargs in ({"robot_id": ""}, {"robot_id": "H1", "lease_ms": 301},
                       {"robot_id": "H1", "lease_ms": True}, {"robot_id": "H1", "boot_id": "bad token"}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                FakeRobotReceiver(**kwargs)
        h = ReceiverHarness()
        h.receiver = FakeRobotReceiver("H1", lease_ms=100)
        h.ready()
        self.assertEqual("rejected", h.send(h.drive(ttl_ms=101))["result"])

    def test_receiver_limits_cannot_silently_exceed_wire_v1_contract(self):
        for limits in (ControlLimits(max_speed_mm_s=181), ControlLimits(max_turn_rad_s=1.51)):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                FakeRobotReceiver("H1", limits=limits)
        h = ReceiverHarness()
        h.receiver = FakeRobotReceiver("H1", limits=ControlLimits(max_speed_mm_s=10))
        h.ready()
        self.assertEqual("rejected", h.send(h.drive(v_mm_s=11))["result"])


if __name__ == "__main__":
    unittest.main()
