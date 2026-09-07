from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

from test_wire_sender import packet

from robo_control.fake_receiver import FakeRobotReceiver
from robo_control.fleet import DEFAULT_ROLES
from robo_control.runtime_report import inspect_report
from robo_control.wire_sender import WireCommandSender


def report_rows(*, stop_delivery="acknowledged"):
    """Actual sender/receiver snapshots, but no runtime/device/clock execution."""
    session = "host-1"
    links = {}
    for index, rid in enumerate(DEFAULT_ROLES):
        sender = WireCommandSender(rid, session_id=session)
        receiver = FakeRobotReceiver(rid, boot_id=f"boot-{rid}")
        clock = 1000. + index * 100.
        sender.accept_response(receiver.receive(sender.hello(10.), clock), 10.01)
        sender.accept_response(receiver.receive(sender.arm(10.02), clock + .02), 10.03)
        sender.accept_response(receiver.receive(sender.drive_from_packet(packet(), 10.04), clock + .04), 10.05)
        links[rid] = (sender, receiver, clock)

    def snapshot(*, closed):
        robots = [{"robot_id": rid, "connected": True, "sender": sender.snapshot(),
                   "receiver": receiver.snapshot(), "stop_requested": closed,
                   "stop_acknowledged": closed and stop_delivery == "acknowledged"}
                  for rid, (sender, receiver, _) in links.items()]
        return {"mode": "fake_wire", "state": "closed" if closed else "ready",
                "session_id": session, "fault": None, "started": True, "closed": closed,
                "ready": not closed, "synthetic": True, "execution": "unknown",
                "physical_execution": "unknown", "device_io": False, "hardware_ready": False,
                "motion_permitted": False, "receiver_watchdog_model": "same_process_polled",
                "host_at_s": 10.08 if closed else 10.06, "pending_event_count": 0,
                "counters": {"requests_sent": 16 if closed else 12,
                             "responses_accepted": 16 if closed and stop_delivery == "acknowledged" else 12},
                "robots": robots,
                "unconfirmed_stop_robot_ids": [r["robot_id"] for r in robots
                                               if r["stop_requested"] and not r["stop_acknowledged"]]}

    def tick(*, closed, wire):
        actuator = {"session_id": session, "drive_model": "differential_body",
                    "device_io": False, "hardware_ready": False, "motion_permitted": False,
                    "robots": [{"robot_id": rid, "velocity_world_mm_s": [0., 0.],
                                "angular_velocity_rad_s": 0., "wheel_velocity_rad_s": [],
                                "forward_velocity_mm_s": 0.} for rid in DEFAULT_ROLES]}
        return {"schema_version": 1, "event": "runtime_tick", "session_id": session,
                "input_mode": "live_camera", "output_mode": "dry_run_commands",
                "transport_mode": "fake_wire", "device_io": False, "motion_permitted": False,
                "at_s": 10.08 if closed else 10.06, "tick_sequence": 2 if closed else 1,
                "status": "closed" if closed else "tracking_goal",
                "closed_reason": "operator_stop" if closed else None,
                "actuator": actuator, "wire": wire, "observation": None, "command": None}

    rows = [{"schema_version": 1, "event": "session_started", "session_id": session,
             "roles": dict(DEFAULT_ROLES), "input_mode": "live_camera", "output_mode": "dry_run_commands",
             "transport_mode": "fake_wire", "drive_model": "differential_body",
             "device_io": False, "motion_permitted": False}, tick(closed=False, wire=snapshot(closed=False))]
    for sender, receiver, clock in links.values():
        stop = sender.stop(10.07)
        if stop_delivery != "dropped_request":
            response = receiver.receive(stop, clock + .07)
            if stop_delivery == "acknowledged":
                sender.accept_response(response, 10.075)
    rows.append(tick(closed=True, wire=snapshot(closed=True)))
    return rows


class RuntimeWireReportTests(unittest.TestCase):
    def inspect(self, rows):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "runtime.jsonl"
            path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            return inspect_report(path)

    def test_acknowledged_stops_report_synthetic_setpoints_not_physical_stop(self):
        result = self.inspect(report_rows())
        self.assertEqual("fake_wire", result["transport_mode"])
        self.assertTrue(result["log_complete"])
        self.assertTrue(result["final_wire_zero"])
        self.assertTrue(result["final_wire_stop_acknowledged"])
        self.assertEqual([], result["unconfirmed_stop_robot_ids"])
        self.assertEqual("synthetic_receiver_setpoints", result["wire_zero_evidence"])
        self.assertFalse(result["physical_stop_verified"])
        self.assertEqual(16, result["wire_counters"]["requests_sent"])

    def test_dropped_stop_request_does_not_make_host_zero_prove_receiver_zero(self):
        result = self.inspect(report_rows(stop_delivery="dropped_request"))
        self.assertTrue(result["log_complete"])
        self.assertTrue(result["final_stop_recorded"])
        self.assertFalse(result["final_wire_zero"])
        self.assertFalse(result["final_wire_stop_acknowledged"])
        self.assertEqual(sorted(DEFAULT_ROLES), result["unconfirmed_stop_robot_ids"])
        self.assertFalse(result["physical_stop_verified"])

    def test_dropped_stop_ack_keeps_zero_and_ack_evidence_separate(self):
        result = self.inspect(report_rows(stop_delivery="dropped_ack"))
        self.assertTrue(result["final_wire_zero"])
        self.assertFalse(result["final_wire_stop_acknowledged"])
        self.assertEqual(sorted(DEFAULT_ROLES), result["unconfirmed_stop_robot_ids"])

    def test_fault_tick_count_and_final_reason_are_reported(self):
        rows = report_rows(stop_delivery="dropped_request")
        rows[1]["wire"].update(state="fault", ready=False, fault="H1:ack_timeout")
        rows[-1]["wire"]["fault"] = "H1:ack_timeout"
        result = self.inspect(rows)
        self.assertEqual(1, result["wire_fault_ticks"])
        self.assertEqual({"H1:ack_timeout": 2}, result["wire_fault_reasons"])

    def test_mock_and_legacy_reports_remain_readable_without_wire_claims(self):
        for explicit in (False, True):
            rows = report_rows()
            for row in rows:
                row.pop("wire", None)
                if explicit:
                    row["transport_mode"] = "mock"
                else:
                    row.pop("transport_mode")
            result = self.inspect(rows)
            self.assertEqual("mock", result["transport_mode"])
            self.assertIsNone(result["final_wire_zero"])
            self.assertIsNone(result["final_wire_stop_acknowledged"])
            self.assertIsNone(result["wire_counters"])

    def test_transport_mode_is_pinned_by_header(self):
        for index, value in ((0, "udp"), (0, "mock"), (1, "mock"), (2, "mock")):
            rows = report_rows()
            rows[index]["transport_mode"] = value
            with self.subTest(index=index, value=value), self.assertRaises(ValueError):
                self.inspect(rows)
        rows = report_rows()
        rows[1].pop("transport_mode")
        with self.assertRaises(ValueError):
            self.inspect(rows)

    def test_fake_wire_missing_or_wrong_drive_model_is_rejected(self):
        for mutate in (lambda rows: rows[1].pop("wire"),
                       lambda rows: rows[0].update(drive_model="mecanum"),
                       lambda rows: rows[1].update(wire=None)):
            rows = report_rows()
            mutate(rows)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_missing_duplicate_and_unknown_wire_robot_are_rejected(self):
        for mutate in (lambda robots: robots.pop(),
                       lambda robots: robots.append(copy.deepcopy(robots[0])),
                       lambda robots: robots[1].update(robot_id=robots[0]["robot_id"]),
                       lambda robots: robots[0].update(robot_id="foreign")):
            rows = report_rows()
            mutate(rows[-1]["wire"]["robots"])
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_mixed_wire_sender_receiver_response_sessions_are_rejected(self):
        for target in ("wire", "sender", "receiver", "response"):
            rows = report_rows()
            wire = rows[-1]["wire"]
            envelope = (wire if target == "wire" else wire["robots"][0]["sender"]["last_response"]
                        if target == "response" else wire["robots"][0][target])
            envelope["session_id" if target == "wire" else "host_session_id"] = "different-session"
            with self.subTest(target=target), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_false_device_flags_and_synthetic_execution_are_required(self):
        for index in (0, 1, 2):
            rows = report_rows()
            rows[index]["hardware_ready"] = True
            with self.subTest(row=index), self.assertRaises(ValueError):
                self.inspect(rows)
        for target in ("wire", "sender", "receiver", "response"):
            for key, value in (("device_io", True), ("hardware_ready", 0), ("motion_permitted", None),
                               ("synthetic", 1), ("execution", "completed")):
                rows = report_rows()
                wire = rows[-1]["wire"]
                envelope = (wire if target == "wire" else wire["robots"][0]["sender"]["last_response"]
                            if target == "response" else wire["robots"][0][target])
                envelope[key] = value
                with self.subTest(target=target, key=key), self.assertRaises(ValueError):
                    self.inspect(rows)

    def test_invalid_sender_receiver_states_and_finite_limits_are_rejected(self):
        for target, changes in (("sender", {"state": "teleporting"}),
                                ("sender", {"pending_request": "drive"}),
                                ("sender", {"wire_sequence": True}),
                                ("sender", {"controller_sequence": -1}),
                                ("sender", {"permit_available": 1}),
                                ("receiver", {"v_mm_s": 180.01}),
                                ("receiver", {"omega_rad_s": 1.501}),
                                ("receiver", {"v_mm_s": float("nan")}),
                                ("receiver", {"v_mm_s": 10**400}),
                                ("receiver", {"measured_velocity": [0., 0.]}),
                                ("receiver", {"accepted_drive_count": False}),
                                ("receiver", {"receiver_at_s": -1}),
                                ("receiver", {"command_deadline_receiver_s": None})):
            rows = report_rows()
            rows[1]["wire"]["robots"][0][target].update(changes)
            with self.subTest(target=target, changes=changes), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_ready_closed_and_fault_consistency_is_checked(self):
        for changes in ({"ready": False}, {"state": "closed"}, {"closed": True},
                        {"started": False}, {"state": "fault", "ready": False},
                        {"fault": "not-latched"}, {"pending_event_count": -1},
                        {"counters": {"requests": True}}, {"host_at_s": 11.}):
            rows = report_rows()
            rows[1]["wire"].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.inspect(rows)
        for changes in ({"connected": False}, {"sender": None}):
            rows = report_rows()
            rows[1]["wire"]["robots"][0].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_final_close_requires_wire_closed_and_every_stop_request(self):
        for mutate in (lambda wire: wire.update(state="pending", closed=False),
                       lambda wire: wire["robots"][0].update(stop_requested=False)):
            rows = report_rows(stop_delivery="dropped_request")
            mutate(rows[-1]["wire"])
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_stop_ack_claim_requires_matching_accepted_stop_response(self):
        for changes in ({"request_type": "drive"}, {"result": "status"}, {"state": "armed"},
                        {"permit_id": "forged-permit"}, {"challenge_id": "forged-challenge"},
                        {"lease_remaining_ms": 100},
                        {"v_mm_s": 1.}, {"seq": 2}, {"boot_id": "other-boot"}, {"link_id": "other-link"}):
            rows = report_rows()
            rows[-1]["wire"]["robots"][0]["sender"]["last_response"].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.inspect(rows)
        rows = report_rows(stop_delivery="dropped_request")
        rows[-1]["wire"]["robots"][0]["stop_acknowledged"] = True
        rows[-1]["wire"]["unconfirmed_stop_robot_ids"].remove("H1")
        with self.assertRaises(ValueError):
            self.inspect(rows)

    def test_declared_unconfirmed_stops_are_not_trusted(self):
        for declared in ([], ["H1"] * 4, ["H1", "H2", "B1", "foreign"], None):
            rows = report_rows(stop_delivery="dropped_ack")
            rows[-1]["wire"]["unconfirmed_stop_robot_ids"] = declared
            with self.subTest(declared=declared), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_inspection_does_not_advance_expired_receiver_or_mutate_fixture(self):
        rows = report_rows(stop_delivery="dropped_request")
        before = copy.deepcopy(rows)
        first = self.inspect(rows)
        second = self.inspect(rows)
        self.assertEqual(before, rows)
        self.assertEqual(first, second)
        self.assertFalse(second["final_wire_zero"])

    def test_invalid_state_types_are_validation_errors_not_parser_crashes(self):
        for target, key in (("wire", "state"), ("sender", "state"), ("sender", "pending_request"),
                            ("receiver", "state")):
            rows = report_rows()
            envelope = rows[1]["wire"] if target == "wire" else rows[1]["wire"]["robots"][0][target]
            envelope[key] = []
            with self.subTest(target=target, key=key), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_actual_supervisor_idle_and_early_close_snapshots_are_readable(self):
        from robo_control.runtime_wire import RuntimeWireSupervisor

        supervisor = RuntimeWireSupervisor(tuple(DEFAULT_ROLES), session_id="host-1")
        rows = report_rows()
        rows[1]["wire"] = supervisor.poll(10.06)
        rows[-1]["wire"] = supervisor.close(10.08)
        result = self.inspect(rows)
        self.assertTrue(result["final_wire_zero"])
        self.assertFalse(result["final_wire_stop_acknowledged"])
        self.assertEqual(sorted(DEFAULT_ROLES), result["unconfirmed_stop_robot_ids"])

    def test_actual_supervisor_drive_and_close_snapshots_are_readable(self):
        from robo_control.runtime_wire import RuntimeWireSupervisor

        for dropped in (False, True):
            supervisor = RuntimeWireSupervisor(tuple(DEFAULT_ROLES), session_id="host-1")
            supervisor.start(10.)
            rows = report_rows()
            rows[1]["wire"] = supervisor.submit(packet(), 10.04)
            if dropped:
                supervisor.configure_link("H1", 10.06, drop_requests=1)
            rows[-1]["wire"] = supervisor.close(10.08)
            result = self.inspect(rows)
            self.assertEqual(not dropped, result["final_wire_zero"])
            self.assertEqual(not dropped, result["final_wire_stop_acknowledged"])
            self.assertEqual(["H1"] if dropped else [], result["unconfirmed_stop_robot_ids"])


if __name__ == "__main__":
    unittest.main()
