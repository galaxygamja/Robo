from __future__ import annotations

import copy
import json
import unittest
from types import SimpleNamespace
from pathlib import Path

from robo_control.hardware_runtime import HardwareRuntime, _parser, calibrated_limits
from robo_control.hardware import HardwareConfig, UdpFleetTransport, default_profile, differential_pwm
from robo_control.runtime_session import LiveControlSession


IDS = ("H1", "H2", "B1", "B2")
POSITIONS = ((200., 250.), (850., 250.), (200., 900.), (850., 900.))


def detection(seq, now, **changes):
    row = {"status": "detected", "sequence": seq, "captured_at_s": now,
           "received_at_s": now, "source_name": "webcam:test", "is_replay": False,
           "coordinate_system": "bottom_left_x_right_y_up_mm", "field_size_mm": [1143., 1181.],
           "registered_robot_ids": list(IDS), "device_io": False,
           "observation_complete": True, "unknown_tag_ids": [], "duplicate_tag_ids": [],
           "objects": [], "robots": [{"robot_id": rid, "robot_center_mm": list(pos),
                                      "heading_rad": 0.} for rid, pos in zip(IDS, POSITIONS)]}
    row.update(changes)
    return row


class Config:
    def __init__(self):
        self.robots = {rid: SimpleNamespace(robot_id=rid, role="hamster" if rid == "H1" else "beaver",
            servo_presets={}, sensor_verified=rid == "H1", envelope_radius_mm=40.) for rid in IDS}
        self.validated = False

    def require_ready(self):
        self.validated = True


class Transport:
    def __init__(self):
        self.ready = False
        self.fault = None
        self.calls = []
        self.grant_ack = False
        self.states = {rid: {"robot_id": rid, "received_at_s": None, "telemetry": {}} for rid in IDS}

    def connect(self, now):
        self.calls.append(("connect", now))

    def poll(self, now):
        self.calls.append(("poll", now))
        if self.grant_ack:
            self.ready, self.grant_ack = True, False

    def send(self, packet, now, servo_us=None):
        self.calls.append(("send", copy.deepcopy(packet), now, servo_us))
        self.ready = False
        return True

    def stop(self, reason, emergency=False):
        self.calls.append(("stop", reason, emergency))
        self.ready = False

    def snapshot(self):
        return {"robots": copy.deepcopy(self.states), "ready": self.ready, "fault": self.fault}

    def close(self):
        self.calls.append(("close",))
        self.ready = False


class ProtocolSocket:
    """In-memory protocol fixture; no real sockets or robot measurements."""

    def __init__(self):
        self.queue, self.sent = [], []

    def setblocking(self, value):
        assert value is False

    def sendto(self, data, address):
        request = json.loads(data)
        self.sent.append(request)
        if request["type"] in {"stop", "estop"}:
            return len(data)
        reply = {"protocol": "robo-hw", "version": 1, "type": "response",
                 "request_id": request["request_id"], "robot_id": request["robot_id"],
                 "accepted": True, "hardware_enabled": True, "boot_id": "fixture-boot",
                 "permit": "fixture-permit-" + str(len(self.sent)), "reason": "fixture",
                 "state": "disarmed" if request["type"] == "hello" else "armed",
                 "seq": request.get("seq", 0), "lease_remaining_ms": 250, "uptime_ms": 50,
                 "left_pwm": request.get("left_pwm", 0), "right_pwm": request.get("right_pwm", 0),
                 "servo_us": request.get("servo_us", [0, 0]), "disc_present": False}
        self.queue.append((json.dumps(reply).encode(), address))
        return len(data)

    def recvfrom(self, maximum):
        if not self.queue:
            raise BlockingIOError()
        return self.queue.pop(0)

    def close(self):
        pass


def make_runtime(*, enabled=True, is_replay=False, drive_model="differential_body"):
    config, transport = Config(), Transport()
    session = LiveControlSession(roles={rid: config.robots[rid].role for rid in IDS},
        field_size_mm=(1143., 1181.), source_name="webcam:test", is_replay=is_replay,
        radii_mm={rid: 40. for rid in IDS}, drive_model=drive_model,
        goals={"H1": {"x_mm": 300., "y_mm": 250.}}, session_id="physical-test")
    runtime = HardwareRuntime(session, config, transport if enabled else None, enabled=enabled, clock=lambda: 0.)
    runtime.start(operator_confirmed=enabled)
    return runtime, transport


class HardwareRuntimeTests(unittest.TestCase):
    def test_slow_measured_curves_bound_translation_and_turning_without_clipping(self):
        data = default_profile()
        data["network_confirmed"] = True
        for index, profile in enumerate(data["robots"].values()):
            profile.update(wiring_verified=True, motion_calibrated=True,
                           track_width_mm=50.+index*10, envelope_radius_mm=40.)
            for key in ("left_forward", "left_reverse", "right_forward", "right_reverse"):
                profile[key] = [[0, 0.], [96, 60.+index*10]]
        config = HardwareConfig.from_dict(data)
        limits = calibrated_limits(config)
        self.assertEqual(30., limits.max_speed_mm_s)
        self.assertEqual(.75, limits.max_turn_rad_s)
        self.assertEqual(.25, limits.command_ttl_s)
        for robot in config.robots.values():
            for direction in (-1, 1):
                for turn in (-1, 1):
                    wheels = differential_pwm(robot, direction*limits.max_speed_mm_s, turn*limits.max_turn_rad_s)
                    self.assertTrue(all(abs(pwm) <= robot.max_pwm for pwm in wheels))
    def test_missing_external_permit_does_not_start_or_advance_mission(self):
        fleet = json.loads((Path(__file__).resolve().parents[1] / "config/qualifier_senior.json").read_text(encoding="utf-8"))
        roles = {rid: "hamster" if rid == "H1" else "beaver" for rid in IDS}
        plan = {"schema_version": 1, "coordinate_system": "bottom_left_x_right_y_up_mm",
                "radii_mm": {rid: 40. for rid in IDS}, "cell_mm": 40., "obstacles_mm": [],
                "tasks": [{"task_id": "move-D1", "pickup": {"x_mm": 200., "y_mm": 250., "heading_rad": 0.},
                           "drop": {"x_mm": 350., "y_mm": 250., "heading_rad": 0.},
                           "retreat": {"x_mm": 200., "y_mm": 250., "heading_rad": 0.}}]}
        session = LiveControlSession(roles=roles, field_size_mm=(1143., 1181.),
            source_name="webcam:test", mission_plan=plan, fleet=fleet)
        for seq in range(1, 6):
            session.advance(detection(seq, 10.+seq*.02), 10.+seq*.02, command_permitted=False)
        self.assertIsNone(session.mission.active)
        session.advance(detection(6, 10.12), 10.12, command_permitted=True)
        self.assertIsNotNone(session.mission.active)
        phase = session.mission.active.phase
        streak = session.mission.arrival_streak
        for seq in range(7, 10):
            session.advance(detection(seq, 10.+seq*.02), 10.+seq*.02, command_permitted=False)
        self.assertEqual(phase, session.mission.active.phase)
        self.assertEqual(streak, session.mission.arrival_streak)

    def test_real_transport_accepts_new_camera_packet_after_handshake_and_maps_all_four(self):
        data = default_profile()
        data["network_confirmed"] = True
        for profile in data["robots"].values():
            profile.update(wiring_verified=True, motion_calibrated=True,
                           track_width_mm=50., envelope_radius_mm=40.)
            for key in ("left_forward", "left_reverse", "right_forward", "right_reverse"):
                profile[key] = [[0, 0.], [96, 500.]]  # synthetic test curve, never deployment calibration
        config = HardwareConfig.from_dict(data)
        session = make_runtime(enabled=False)[0].session
        socket = ProtocolSocket()
        transport = UdpFleetTransport(config, "test-token-only-abcdefghijklmnop", sock=socket)
        runtime = HardwareRuntime(session, config, transport, enabled=True, clock=lambda: 0.)
        runtime.start(operator_confirmed=True)
        for seq in range(1, 7):
            event = runtime.advance(detection(seq, 10.+seq*.02), 10.+seq*.02)
        self.assertIsNone(runtime.closed_reason)
        self.assertGreater(runtime.sent_packets, 0)
        self.assertTrue(event["hardware_command_sent"])
        commands = [r for r in socket.sent if r["type"] == "command"]
        self.assertEqual(len(IDS) * runtime.sent_packets, len(commands))
        self.assertEqual(set(IDS), {r["robot_id"] for r in commands})
        self.assertTrue(any(r["left_pwm"] > 0 for r in commands if r["robot_id"] == "H1"))
        self.assertTrue(all(r["left_pwm"] == 0 and r["right_pwm"] == 0 for r in commands if r["robot_id"] != "H1"))

    def warmed(self):
        runtime, transport = make_runtime()
        for seq in (1, 2, 3):
            event = runtime.advance(detection(seq, 10. + (seq-1)*.02), 10. + (seq-1)*.02)
        self.assertEqual("hardware_connecting", event["status"])
        self.assertEqual(["connect"], [c[0] for c in transport.calls])
        self.assertEqual(0, runtime.sent_packets)
        transport.grant_ack = True
        event = runtime.advance(detection(4, 10.06), 10.06)
        self.assertTrue(event["hardware_command_sent"])
        self.assertEqual(["connect", "poll", "send"], [c[0] for c in transport.calls])
        return runtime, transport

    def test_live_full_fleet_observations_and_ack_precede_every_physical_packet(self):
        runtime, transport = self.warmed()
        first = next(c for c in transport.calls if c[0] == "send")[1]
        self.assertEqual(set(IDS), {r["robot_id"] for r in first["robots"]})
        self.assertEqual("differential_body", first["drive_model"])
        event = runtime.advance(detection(5, 10.08), 10.08)
        self.assertFalse(event["hardware_command_sent"])
        self.assertEqual(1, runtime.sent_packets)
        transport.grant_ack = True
        event = runtime.advance(detection(6, 10.10), 10.10)
        self.assertTrue(event["hardware_command_sent"])
        sent = [c[1] for c in transport.calls if c[0] == "send"]
        self.assertGreater(sent[-1]["sequence"], first["sequence"])
        self.assertEqual(10.10, sent[-1]["issued_at_s"])

    def test_ack_during_empty_poll_does_not_repeat_previous_packet(self):
        runtime, transport = self.warmed()
        transport.grant_ack = True
        runtime.advance(None, 10.08)
        self.assertEqual(1, runtime.sent_packets)
        self.assertTrue(transport.ready)
        runtime.advance(detection(5, 10.10), 10.10)
        self.assertEqual(2, runtime.sent_packets)

    def test_invalid_stale_replay_or_incomplete_frame_closes_all_without_retry(self):
        mutations = [lambda r: r.update(robots=[]),
            lambda r: r["robots"].pop(), lambda r: r.update(is_replay=True),
            lambda r: r.update(sequence=4), lambda r: r.update(captured_at_s=9.8),
            lambda r: r.update(unknown_tag_ids=[98]), lambda r: r.update(duplicate_tag_ids=[0]),
            lambda r: r["robots"][0].update(heading_rad=float("nan")),
            lambda r: r.update(source_name="video:replay"),
            lambda r: r.update(registered_robot_ids=list(IDS[:-1]))]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                runtime, transport = self.warmed()
                row = detection(5, 10.08)
                mutate(row)
                transport.grant_ack = True
                event = runtime.advance(row, 10.08)
                self.assertIsNotNone(runtime.closed_reason)
                self.assertFalse(event["motion_permitted"])
                self.assertEqual(1, runtime.sent_packets)
                self.assertIn("stop", [c[0] for c in transport.calls])
                before = list(transport.calls)
                runtime.advance(detection(6, 10.10), 10.10)
                self.assertEqual(before, transport.calls)
                with self.assertRaises(ValueError):
                    runtime.start(operator_confirmed=True)

    def test_silent_camera_expires_even_when_new_frame_arrives_at_deadline(self):
        runtime, transport = self.warmed()
        runtime.advance(None, 10.14)
        runtime.advance(None, 10.22)
        event = runtime.advance(detection(5, 10.26), 10.26)
        self.assertEqual("observation_watchdog", runtime.closed_reason)
        self.assertFalse(event["hardware_command_sent"])
        self.assertEqual(1, runtime.sent_packets)

    def test_transport_fault_latches_fleet_stop_before_new_calculation(self):
        runtime, transport = self.warmed()
        sequence = runtime.session.controller.sequence
        transport.fault = "receiver_watchdog"
        event = runtime.advance(detection(5, 10.08), 10.08)
        self.assertEqual("hardware_fault:receiver_watchdog", runtime.closed_reason)
        self.assertFalse(event["motion_permitted"])
        # Closing emits one stop command, but never another physical movement.
        self.assertEqual(sequence + 1, runtime.session.controller.sequence)
        self.assertEqual(1, runtime.sent_packets)

    def test_scheduler_and_slow_controller_deadlines_stop_physical_transport(self):
        runtime, transport = self.warmed()
        runtime.advance(detection(5, 10.17), 10.17)
        self.assertEqual("supervisor_deadline_missed", runtime.closed_reason)
        runtime, transport = self.warmed()
        clock = iter((0., .11))
        runtime.clock = lambda: next(clock)
        transport.grant_ack = True
        runtime.advance(detection(5, 10.08), 10.08)
        self.assertEqual("supervisor_deadline_missed", runtime.closed_reason)
        self.assertEqual(1, runtime.sent_packets)

    def test_dry_mode_creates_no_hardware_io_and_replay_cannot_enable(self):
        runtime, transport = make_runtime(enabled=False)
        for seq in range(1, 5):
            event = runtime.advance(detection(seq, 10.+seq*.02), 10.+seq*.02)
        self.assertFalse(event["device_io"])
        self.assertFalse(event["hardware_command_sent"])
        self.assertEqual([], transport.calls)
        with self.assertRaises(ValueError):
            make_runtime(is_replay=True)
        with self.assertRaises(ValueError):
            make_runtime(drive_model="mecanum")

    def test_telemetry_never_fabricates_servo_or_release_success(self):
        runtime, transport = self.warmed()
        runtime.session.mission = SimpleNamespace(active=SimpleNamespace(task=SimpleNamespace(robot_id="H1")),
                                                 session_id="s", command_id="c")
        transport.states["H1"].update(received_at_s=10.075, telemetry={"disc_present": True, "request_sent_at_s": 10.07,
             "servo_closed": True, "hopper_loaded": True})
        feedback = runtime._feedback(10.08)
        self.assertEqual({"optical_present": True, "optical_clear": False}, feedback["signals"])
        self.assertFalse(feedback["synthetic"])
        self.assertIsNone(runtime._feedback(10.3))
        transport.states["H1"]["received_at_s"] = 10.08
        transport.states["H1"]["telemetry"]["request_sent_at_s"] = 9.8
        self.assertIsNone(runtime._feedback(10.08))
        transport.states["H1"]["telemetry"]["request_sent_at_s"] = 10.07
        runtime.config.robots["H1"].sensor_verified = False
        self.assertIsNone(runtime._feedback(10.08))

    def test_servo_intents_need_explicit_calibrated_presets(self):
        runtime, _ = make_runtime()
        event = {"mission": {"manipulator_intent": {"robot_id": "H1", "servo_intent": "disc_latch_close"}}}
        with self.assertRaisesRegex(ValueError, "unsupported_servo_intent"):
            runtime._servo_targets(event)
        runtime.config.robots["H1"].servo_presets["disc_latch_close"] = (1450, 0)
        expected = {rid: (1450, 0) if rid == "H1" else (0, 0) for rid in IDS}
        self.assertEqual(expected, runtime._servo_targets(event))
        event["mission"]["manipulator_intent"]["servo_intent"] = "hold"
        self.assertEqual(expected, runtime._servo_targets(event))

    def test_cli_defaults_to_no_physical_enable_and_has_no_replay_argument(self):
        parser = _parser()
        args = parser.parse_args(["--hardware-config", "h.json", "--calibration", "c.json",
                                  "--fleet", "f.json", "--report", "new.jsonl"])
        self.assertFalse(args.enable_hardware)
        self.assertNotIn("--video", parser._option_string_actions)


if __name__ == "__main__":
    unittest.main()
