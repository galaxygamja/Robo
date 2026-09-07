from __future__ import annotations

import copy
import unittest

from robo_control.wire_codec import WireError, encode_frame
from robo_control.wire_lab import WireLab, run_demo
from robo_control.wire_sender import WireCommandSender


class WireIntegrationTests(unittest.TestCase):
    def ready(self, **kwargs):
        lab = WireLab(**kwargs)
        lab.connect()
        lab.advance(.02)
        return lab

    def test_full_fault_demo_and_all_four_final_zero(self):
        result = run_demo()
        self.assertTrue(result["passed"])
        self.assertEqual(7, len(result["checks"]))
        self.assertTrue(result["final_all_zero"])
        self.assertFalse(result["device_io"])
        self.assertEqual("unknown", result["physical_execution"])

    def test_independent_clock_origins_do_not_change_accepted_commands(self):
        for host, devices in ((1., (10000., 5., 20., 90000.)), (90000., (1., 2., 3., 4.))):
            with self.subTest(host=host):
                lab = self.ready(host_origin_s=host, device_origins_s=devices)
                replies = lab.send_controller_packet(lab.controller_packet())
                self.assertTrue(all(r["result"] == "accepted" for r in replies.values()))
                lab.advance(.3)
                self.assertTrue(lab.stopped())

    def test_one_lost_robot_request_does_not_renew_it_or_touch_other_robots(self):
        lab = self.ready()
        packet = lab.controller_packet()
        messages = {rid: sender.drive_from_packet(packet, lab.host_now) for rid, sender in lab.senders.items()}
        for rid in lab.ids:
            lab.exchange(rid, messages[rid], drop_request=rid == "H1")
        self.assertEqual(0, lab.receivers["H1"].snapshot()["accepted_drive_count"])
        self.assertTrue(all(lab.receivers[rid].snapshot()["v_mm_s"] > 0 for rid in lab.ids if rid != "H1"))
        lab.advance(.3)
        self.assertTrue(lab.stopped())
        self.assertEqual("fault", lab.senders["H1"].snapshot()["state"])

    def test_late_ack_cannot_reopen_timed_out_sender(self):
        lab = self.ready()
        msg = lab.senders["H1"].drive_from_packet(lab.controller_packet(), lab.host_now)
        response = lab.exchange("H1", msg, ack_delay_s=.31)
        self.assertEqual("accepted", response["result"])  # A historical acceptance, not current execution.
        self.assertEqual("fault", lab.senders["H1"].snapshot()["state"])
        self.assertTrue(lab.stopped())

    def test_source_command_made_before_new_permit_cannot_be_rewrapped(self):
        lab = self.ready()
        old = lab.controller_packet()
        lab.advance(.01)
        lab.connect()
        with self.assertRaisesRegex(ValueError, "predates"):
            lab.senders["H1"].drive_from_packet(old, lab.host_now)
        self.assertTrue(lab.stopped())

    def test_malformed_complete_fleet_is_not_partially_delivered(self):
        lab = self.ready()
        packet = lab.controller_packet()
        packet["robots"][-1]["robot_id"] = "H1"
        with self.assertRaises(ValueError):
            lab.send_controller_packet(packet)
        self.assertTrue(lab.stopped())
        self.assertTrue(all(r.snapshot()["accepted_drive_count"] == 0 for r in lab.receivers.values()))

    def test_corrupt_stream_closes_only_its_addressed_link_and_latches_decoder(self):
        lab = self.ready()
        lab.send_controller_packet(lab.controller_packet())
        with self.assertRaises(WireError):
            lab.deliver("H1", b'{"version":1,"version":2}\n')
        self.assertEqual(0, lab.receivers["H1"].snapshot()["v_mm_s"])
        self.assertGreater(lab.receivers["B1"].snapshot()["v_mm_s"], 0)
        with self.assertRaises(WireError):
            lab.deliver("H1", b'{}\n')
        lab.advance(.3)
        self.assertTrue(lab.stopped())

    def test_fragment_without_newline_never_applies_command(self):
        lab = self.ready()
        msg = lab.senders["H1"].drive_from_packet(lab.controller_packet(), lab.host_now)
        raw = encode_frame(msg)
        self.assertEqual([], lab.deliver("H1", raw[:-1]))
        self.assertEqual(0, lab.receivers["H1"].snapshot()["accepted_drive_count"])
        lab.advance(.3)
        response = lab.deliver("H1", raw[-1:])[0]
        self.assertEqual("rejected", response["result"])
        self.assertTrue(lab.stopped())

    def test_observation_loss_controller_hold_is_transmitted_as_stop(self):
        lab = self.ready()
        lab.send_controller_packet(lab.controller_packet())
        lab.advance(.02)
        packet = lab.controller.tick(None, lab.host_now)
        replies = lab.send_controller_packet(packet)
        self.assertTrue(all(r["request_type"] == "stop" for r in replies.values()))
        self.assertTrue(lab.stopped())

    def test_current_mission_runtime_packet_reaches_protocol_without_fabricated_feedback(self):
        from test_mission_runtime import MissionHarness

        mission = MissionHarness(plan_change=lambda p: p["tasks"][0]["pickup"].update(x_mm=450.))
        lab = WireLab(host_origin_s=10.)
        lab.senders = {rid: WireCommandSender(rid, session_id="mission-test") for rid in lab.ids}
        lab.connect()
        for _ in range(4):
            lab.advance(.02)
            event = mission.step()
        packet = event["command"]
        original = copy.deepcopy(packet)
        self.assertEqual("differential_body", packet["drive_model"])
        message = lab.senders["H1"].drive_from_packet(packet, lab.host_now)
        reply = lab.exchange("H1", message)
        self.assertEqual("accepted", reply["result"])
        self.assertEqual("drive", reply["request_type"])
        self.assertEqual(original, packet)
        self.assertTrue(reply["synthetic"])
        self.assertEqual("unknown", reply["execution"])
        self.assertNotIn("signals", reply)
        self.assertEqual([], mission.session.mission.snapshot()["completed_tasks"])


if __name__ == "__main__":
    unittest.main()
