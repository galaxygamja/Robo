from __future__ import annotations

import copy
import json
import math
import unittest

from robo_control.control_loop import ClosedLoopController, ControlLimits
from robo_control.fleet import DEFAULT_ROLES
from robo_control.runtime_wire import MAX_QUEUED_EVENTS, RuntimeWireSupervisor
from robo_control.wire_codec import encode_frame


def packet(issued=.01, sequence=1, *, paused=False, idle_robot=None, all_idle=False, emergency=False):
    positions = {rid: (0., index * 1000.) for index, rid in enumerate(DEFAULT_ROLES)}
    goals = {} if all_idle else {rid: {"x_mm": 600., "y_mm": position[1]}
                               for rid, position in positions.items() if rid != idle_robot}
    controller = ClosedLoopController(roles=DEFAULT_ROLES, session_id="runtime-1",
                                     drive_model="differential_body", goals=goals)
    controller.sequence = sequence - 1
    controller.set_paused(paused)
    if emergency:
        controller.emergency_stop()
    observation = {"source_name": "test-camera", "sequence": sequence, "captured_at_s": issued,
        "observation_usable": True, "stop_required": False,
        "tracks": [{"robot_id": rid, "robot_center_mm": list(position), "heading_rad": 0.,
            "observed_at_s": issued, "velocity_mm_s": [0., 0.], "angular_velocity_rad_s": 0.,
            "state": "observed", "valid_for_control": True} for rid, position in positions.items()]}
    return controller.tick(observation, issued)


def supervisor(**kwargs):
    return RuntimeWireSupervisor(tuple(DEFAULT_ROLES), session_id="runtime-1", **kwargs)


def robots(state):
    return {row["robot_id"]: row for row in state["robots"]}


class RuntimeWireTests(unittest.TestCase):
    def test_initial_explicit_handshake_is_zero_and_clock_origins_are_distinct(self):
        wire = supervisor()
        self.assertEqual("idle", wire.snapshot()["state"])
        self.assertFalse(wire.ready)
        state = wire.start(10.)
        self.assertTrue(wire.started)
        self.assertTrue(wire.ready)
        self.assertEqual("ready", state["state"])
        stamps = []
        for row in state["robots"]:
            self.assertEqual("armed", row["receiver"]["state"])
            self.assertEqual(0, row["receiver"]["v_mm_s"])
            self.assertIsNone(row["sender"]["pending_request"])
            stamps.append(row["receiver"]["receiver_at_s"])
        self.assertEqual(4, len(set(stamps)))
        self.assertNotIn(10., stamps)
        self.assertEqual(16, state["counters"]["events_processed"])

    def test_controller_packet_crosses_frames_and_four_receivers(self):
        wire = supervisor()
        wire.start(0.)
        source = packet()
        state = wire.submit(source, .01)
        self.assertTrue(state["ready"])
        for row, command in zip(state["robots"], source["robots"]):
            self.assertEqual(command["robot_id"], row["robot_id"])
            self.assertEqual(command["forward_velocity_mm_s"], row["receiver"]["v_mm_s"])
            self.assertEqual(1, row["receiver"]["accepted_drive_count"])
        self.assertEqual(1, state["counters"]["submitted_packets"])
        self.assertEqual(0, state["pending_event_count"])

    def test_delayed_handshake_and_ack_are_nonblocking_without_replay(self):
        wire = supervisor(link_options={rid: {"response_delay_s": .01} for rid in DEFAULT_ROLES})
        self.assertEqual("connecting", wire.start(0.)["state"])
        self.assertFalse(wire.poll(.01)["ready"])
        self.assertTrue(wire.poll(.02)["ready"])
        pending = wire.submit(packet(.03), .03)
        self.assertEqual("pending", pending["state"])
        self.assertIsNone(pending["fault"])
        self.assertTrue(wire.poll(.04)["ready"])
        self.assertTrue(wire.poll(.05)["ready"])
        for row in wire.snapshot()["robots"]:
            self.assertEqual(1, row["receiver"]["accepted_drive_count"])

    def test_one_dropped_ack_faults_and_requests_stop_for_whole_fleet(self):
        wire = supervisor()
        wire.start(0.)
        wire.configure_link("B2", .01, drop_responses=1)
        self.assertFalse(wire.submit(packet(.02), .02)["ready"])
        state = wire.poll(.32)
        self.assertIsNotNone(state["fault"])
        self.assertFalse(state["ready"])
        self.assertTrue(all(row["stop_requested"] for row in state["robots"]))
        self.assertTrue(all(row["receiver"]["v_mm_s"] == 0 for row in state["robots"]))
        self.assertEqual(1, state["counters"]["responses_dropped"])

    def test_dropped_request_never_applies_or_retries_and_eventually_inhibits_fleet(self):
        wire = supervisor()
        wire.start(0.)
        wire.configure_link("B1", .01, drop_requests=1)
        state = wire.submit(packet(.02), .02)
        self.assertEqual(0, robots(state)["B1"]["receiver"]["accepted_drive_count"])
        self.assertEqual(1, robots(state)["B2"]["receiver"]["accepted_drive_count"])
        state = wire.poll(.31)
        self.assertIsNotNone(state["fault"])
        self.assertEqual(0, robots(state)["B1"]["receiver"]["accepted_drive_count"])
        self.assertEqual(1, state["counters"]["requests_dropped"])

    def test_stalled_poll_never_executes_queued_motion_retroactively(self):
        wire = supervisor()
        wire.start(0.)
        for rid in DEFAULT_ROLES:
            wire.configure_link(rid, .01, request_delay_s=.05)
        self.assertFalse(wire.submit(packet(.02), .02)["ready"])
        # No modeled endpoint process ran at the earlier due times. The actual
        # poll time first expires watchdogs and retires the stale drive bytes.
        state = wire.poll(.4)
        self.assertIsNotNone(state["fault"])
        self.assertTrue(all(row["receiver"]["accepted_drive_count"] == 0 for row in state["robots"]))
        self.assertGreaterEqual(state["counters"]["retired_events"], 4)

    def test_no_new_observation_ticks_independent_modeled_watchdogs(self):
        wire = supervisor()
        wire.start(0.)
        wire.submit(packet(), .01)
        count = wire.snapshot()["counters"]["submitted_packets"]
        state = wire.poll(.3)
        self.assertIn("watchdog", state["fault"])
        self.assertEqual(count, state["counters"]["submitted_packets"])
        self.assertTrue(all(row["receiver"]["v_mm_s"] == 0 for row in state["robots"]))
        self.assertFalse(wire.poll(.4)["ready"])

    def test_atomic_preparation_rejects_bad_last_robot_before_any_drive(self):
        wire = supervisor()
        wire.start(0.)
        source = packet()
        source["robots"][-1]["forward_velocity_mm_s"] = 999
        state = wire.submit(source, .01)
        self.assertEqual("invalid_controller_packet", state["fault"])
        self.assertEqual(0, state["counters"]["submitted_packets"])
        self.assertTrue(all(row["receiver"]["accepted_drive_count"] == 0 for row in state["robots"]))

    def test_tight_receiver_rejection_faults_whole_fleet(self):
        wire = supervisor(limits=ControlLimits(max_speed_mm_s=1))
        wire.start(0.)
        state = wire.submit(packet(), .01)
        self.assertIsNotNone(state["fault"])
        self.assertTrue(all(row["stop_requested"] for row in state["robots"]))
        self.assertTrue(all(row["receiver"]["v_mm_s"] == 0 for row in state["robots"]))

    def test_delayed_stop_does_not_fabricate_zero_or_ack(self):
        wire = supervisor()
        wire.start(0.)
        wire.submit(packet(), .01)
        wire.configure_link("H1", .02, request_delay_s=.1)
        state = wire.stop(.02, "operator_test")
        first = robots(state)["H1"]
        self.assertGreater(first["receiver"]["v_mm_s"], 0)
        self.assertTrue(first["stop_requested"])
        self.assertFalse(first["stop_acknowledged"])
        self.assertIn("H1", state["unconfirmed_stop_robot_ids"])
        state = wire.poll(.13)
        self.assertEqual(0, robots(state)["H1"]["receiver"]["v_mm_s"])
        self.assertTrue(robots(state)["H1"]["stop_acknowledged"])

    def test_disconnection_drops_stop_until_receiver_watchdog_without_fake_ack(self):
        wire = supervisor()
        wire.start(0.)
        wire.submit(packet(), .01)
        state = wire.configure_link("B1", .02, connected=False)
        self.assertIn("link_disconnected:B1", state["fault"])
        self.assertGreater(robots(state)["B1"]["receiver"]["v_mm_s"], 0)
        self.assertFalse(robots(state)["B1"]["stop_acknowledged"])
        state = wire.poll(.3)
        self.assertEqual(0, robots(state)["B1"]["receiver"]["v_mm_s"])
        self.assertIn("B1", state["unconfirmed_stop_robot_ids"])

    def test_lost_stop_ack_is_unconfirmed_even_when_receiver_is_zero(self):
        wire = supervisor()
        wire.start(0.)
        wire.submit(packet(), .01)
        wire.configure_link("H2", .02, drop_responses=1)
        state = wire.close(.02)
        self.assertIsNone(state["fault"])
        self.assertEqual(0, robots(state)["H2"]["receiver"]["v_mm_s"])
        self.assertFalse(robots(state)["H2"]["stop_acknowledged"])
        self.assertIn("H2", wire.poll(.4)["unconfirmed_stop_robot_ids"])

    def test_close_is_nonblocking_and_later_poll_can_settle_stop_ack(self):
        wire = supervisor()
        wire.start(0.)
        wire.submit(packet(), .01)
        wire.configure_link("B2", .02, response_delay_s=.05)
        state = wire.close(.02, "normal_runtime_end")
        self.assertEqual("closed", state["state"])
        self.assertTrue(wire.closed)
        self.assertIsNone(wire.fault)
        self.assertIn("B2", state["unconfirmed_stop_robot_ids"])
        state = wire.poll(.08)
        self.assertEqual([], state["unconfirmed_stop_robot_ids"])
        self.assertIsNone(state["fault"])
        self.assertFalse(wire.ready)

    def test_close_after_fault_preserves_original_reason_without_resending_stops(self):
        wire = supervisor()
        wire.start(0.)
        state = wire.stop(.01, "first_fault")
        count = state["counters"]["requests_delivered"]
        self.assertEqual("first_fault", wire.close(.02, "normal_end")["fault"])
        self.assertEqual(count, wire.close(.03)["counters"]["requests_delivered"])

    def test_late_drive_ack_after_stop_cannot_restore_permit_or_hide_pending_stop(self):
        wire = supervisor()
        wire.start(0.)
        wire.submit(packet(), .01)
        old = robots(wire.snapshot())["H1"]["sender"]["last_response"]
        wire.configure_link("H1", .02, response_delay_s=.1)
        wire.stop(.02, "stop_before_late_ack")
        state = wire.inject_response("H1", encode_frame(old), .03)
        self.assertFalse(state["ready"])
        self.assertFalse(robots(state)["H1"]["sender"]["permit_available"])
        self.assertFalse(robots(state)["H1"]["stop_acknowledged"])
        self.assertEqual(1, state["counters"]["ignored_late_responses"])
        self.assertTrue(robots(wire.poll(.13))["H1"]["stop_acknowledged"])

    def test_duplicate_ack_latches_global_fault_without_reapplication(self):
        wire = supervisor()
        wire.start(0.)
        wire.configure_link("H1", .01, duplicate_responses=1)
        state = wire.submit(packet(.02), .02)
        self.assertIsNotNone(state["fault"])
        self.assertFalse(wire.ready)
        self.assertTrue(all(row["receiver"]["accepted_drive_count"] == 1 for row in state["robots"]))
        self.assertTrue(all(row["receiver"]["v_mm_s"] == 0 for row in state["robots"]))

    def test_wrong_robot_ack_and_malformed_utf8_cause_fleet_inhibit(self):
        for malformed in (False, True):
            with self.subTest(malformed=malformed):
                wire = supervisor()
                wire.start(0.)
                wire.submit(packet(), .01)
                other = robots(wire.snapshot())["B1"]["sender"]["last_response"]
                frame = b"\xff\n" if malformed else encode_frame(other)
                state = wire.inject_response("H1", frame, .02)
                self.assertIsNotNone(state["fault"])
                self.assertTrue(all(row["stop_requested"] for row in state["robots"]))
                self.assertTrue(all(row["receiver"]["v_mm_s"] == 0 for row in state["robots"]))
                if malformed:
                    self.assertIn("H1", state["unconfirmed_stop_robot_ids"])

    def test_reconnect_requires_confirmation_and_retires_context_not_source_sequence(self):
        wire = supervisor()
        wire.start(0.)
        wire.submit(packet(), .01)
        old = robots(wire.snapshot())["H1"]["sender"]
        wire.stop(.02, "test_fault")
        with self.assertRaises(ValueError):
            wire.reconnect(.03, operator_confirmed=False)
        self.assertIsNotNone(wire.fault)
        state = wire.reconnect(.03, operator_confirmed=True)
        self.assertTrue(state["ready"])
        new = robots(state)["H1"]["sender"]
        self.assertNotEqual(old["link_id"], new["link_id"])
        self.assertEqual(old["controller_sequence"], new["controller_sequence"])
        self.assertEqual(0, robots(state)["H1"]["receiver"]["v_mm_s"])
        self.assertIsNotNone(wire.submit(packet(.04, sequence=1), .04)["fault"])

    def test_reconnect_after_disconnected_watchdog_requires_restored_link(self):
        wire = supervisor()
        wire.start(0.)
        wire.submit(packet(), .01)
        wire.configure_link("H1", .02, connected=False)
        wire.poll(.3)
        with self.assertRaises(ValueError):
            wire.reconnect(.31, operator_confirmed=True)
        wire.configure_link("H1", .32, connected=True)
        self.assertFalse(wire.ready)
        self.assertTrue(wire.reconnect(.33, operator_confirmed=True)["ready"])
        self.assertTrue(wire.submit(packet(.34, sequence=2), .34)["ready"])

    def test_reconnect_does_not_replay_old_queued_drive(self):
        wire = supervisor()
        wire.start(0.)
        wire.configure_link("B2", .01, request_delay_s=.2)
        wire.submit(packet(.02), .02)
        wire.stop(.03, "cancel_queued_drive")
        wire.configure_link("B2", .04, request_delay_s=0.)
        self.assertTrue(wire.reconnect(.04, operator_confirmed=True)["ready"])
        state = wire.poll(.23)
        self.assertEqual(0, robots(state)["B2"]["receiver"]["accepted_drive_count"])
        self.assertTrue(all(row["receiver"]["v_mm_s"] == 0 for row in state["robots"]))

    def test_idle_robot_and_all_at_goal_keep_zero_permits_without_rearm(self):
        wire = supervisor()
        wire.start(0.)
        state = wire.submit(packet(idle_robot="B2"), .01)
        self.assertTrue(state["ready"])
        self.assertEqual(0, robots(state)["B2"]["receiver"]["v_mm_s"])
        self.assertTrue(robots(state)["B2"]["sender"]["permit_available"])
        state = wire.submit(packet(.02, sequence=2, all_idle=True), .02)
        self.assertTrue(state["ready"])
        self.assertTrue(all(row["receiver"]["v_mm_s"] == 0 for row in state["robots"]))

    def test_actual_hold_and_emergency_stop_latch_without_auto_rearm(self):
        for emergency in (False, True):
            with self.subTest(emergency=emergency):
                wire = supervisor()
                wire.start(0.)
                wire.submit(packet(), .01)
                state = wire.submit(packet(.02, 2, paused=not emergency, emergency=emergency), .02)
                self.assertIsNotNone(state["fault"])
                self.assertTrue(all(row["receiver"]["state"] == ("estop" if emergency else "disarmed")
                                    for row in state["robots"]))
                state = wire.submit(packet(.03, 3), .03)
                self.assertFalse(state["ready"])

    def test_queued_bytes_and_counters_are_bounded(self):
        wire = supervisor()
        wire.start(0.)
        for _ in range(MAX_QUEUED_EVENTS + 2):
            state = wire.inject_response("H1", b"{}\n", .01, delay_s=5.)
            self.assertLessEqual(state["pending_event_count"], MAX_QUEUED_EVENTS)
        self.assertIsNotNone(wire.fault)
        self.assertTrue(all(type(count) is int and 0 <= count <= 2**53 - 1
                            for count in state["counters"].values()))

    def test_invalid_clock_latches_and_cannot_reconnect_without_new_supervisor(self):
        for stamp in (-1, True, math.nan, math.inf, .005):
            with self.subTest(stamp=stamp):
                wire = supervisor()
                wire.start(0.)
                wire.submit(packet(), .01)
                with self.assertRaises(ValueError):
                    wire.poll(stamp)
                self.assertEqual("invalid_supervisor_clock", wire.fault)
                self.assertFalse(wire.ready)
                with self.assertRaises(ValueError):
                    wire.reconnect(.02, operator_confirmed=True)

    def test_close_after_broken_clock_returns_honest_state_without_advancing_or_raising(self):
        wire = supervisor()
        wire.start(0.)
        before = wire.submit(packet(), .01)
        with self.assertRaises(ValueError):
            wire.poll(-1)
        state = wire.close(999., "runtime_failed")
        self.assertTrue(state["closed"])
        self.assertEqual("closed", state["state"])
        self.assertEqual("invalid_supervisor_clock", state["fault"])
        self.assertEqual(.01, state["host_at_s"])
        self.assertFalse(state["ready"])
        for old, current in zip(before["robots"], state["robots"]):
            self.assertEqual(old["receiver"], current["receiver"])
            self.assertGreater(current["receiver"]["v_mm_s"], 0)
            self.assertTrue(current["stop_requested"])
            self.assertFalse(current["stop_acknowledged"])
        self.assertEqual(list(DEFAULT_ROLES), state["unconfirmed_stop_robot_ids"])
        self.assertEqual(state, wire.close(math.nan, "repeat_cleanup"))
        with self.assertRaises(ValueError):
            wire.poll(.02)
        with self.assertRaises(ValueError):
            wire.reconnect(.02, operator_confirmed=True)

    def test_broken_clock_close_does_not_deliver_already_queued_motion(self):
        wire = supervisor()
        wire.start(0.)
        wire.configure_link("H1", .01, request_delay_s=.1)
        before = wire.submit(packet(.02), .02)
        self.assertEqual(0, robots(before)["H1"]["receiver"]["accepted_drive_count"])
        with self.assertRaises(ValueError):
            wire.poll(.01)
        state = wire.close(100.)
        self.assertEqual(0, robots(state)["H1"]["receiver"]["accepted_drive_count"])
        self.assertEqual(before["counters"]["requests_delivered"], state["counters"]["requests_delivered"])
        self.assertTrue(all(not row["stop_acknowledged"] for row in state["robots"]))

    def test_broken_clock_before_first_valid_tick_still_allows_honest_close(self):
        wire = supervisor()
        with self.assertRaises(ValueError):
            wire.poll(math.nan)
        state = wire.close(0.)
        self.assertTrue(state["closed"])
        self.assertFalse(state["started"])
        self.assertIsNone(state["host_at_s"])
        self.assertEqual("invalid_supervisor_clock", state["fault"])
        self.assertEqual(list(DEFAULT_ROLES), state["unconfirmed_stop_robot_ids"])
        self.assertTrue(all(row["receiver"]["receiver_at_s"] is None for row in state["robots"]))

    def test_snapshots_are_json_independent_and_never_claim_physical_execution(self):
        wire = supervisor()
        wire.start(0.)
        state = wire.submit(packet(), .01)
        before = copy.deepcopy(state)
        state["robots"][0]["receiver"]["v_mm_s"] = 999
        state["counters"]["faults"] = 99
        self.assertEqual(before, wire.snapshot())
        self.assertEqual(before, json.loads(json.dumps(before, allow_nan=False)))
        self.assertEqual("same_process_polled", before["receiver_watchdog_model"])
        self.assertEqual("unknown", before["physical_execution"])
        for flag in ("device_io", "hardware_ready", "motion_permitted"):
            self.assertIs(False, before[flag])

    def test_start_and_close_are_explicit_no_implicit_recovery(self):
        wire = supervisor()
        wire.start(0.)
        with self.assertRaises(ValueError):
            wire.start(.01)
        wire.close(.02)
        self.assertEqual("closed", wire.close(.03)["state"])
        with self.assertRaises(ValueError):
            wire.reconnect(.04, operator_confirmed=True)
        count = wire.snapshot()["counters"]["submitted_packets"]
        wire.submit(packet(.05), .05)
        self.assertEqual(count, wire.snapshot()["counters"]["submitted_packets"])

    def test_constructor_and_injection_options_are_strict(self):
        for ids in ([], ("H1", "H1"), "H1", ("not valid",)):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                RuntimeWireSupervisor(ids, session_id="runtime-1")
        for options in ({"foreign": {}}, {"H1": {"delay": 1}}, {"H1": {"request_delay_s": math.inf}},
                        {"H1": {"drop_responses": True}}, {"H1": {"connected": 1}}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                supervisor(link_options=options)
        wire = supervisor()
        with self.assertRaises(ValueError):
            wire.configure_link("unknown", 0., connected=False)
        with self.assertRaises(ValueError):
            wire.inject_response("H1", b"{}\n", 0., delay_s=-1)


if __name__ == "__main__":
    unittest.main()
