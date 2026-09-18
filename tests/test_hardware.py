"""Real transport state machine with deterministic local datagrams; no hardware."""
import copy
import json
import math
import unittest
from unittest.mock import MagicMock, patch
from pathlib import Path

from robo_control.hardware import (HardwareConfig, UdpFleetTransport, default_profile,
                                   differential_pwm, strict_json)


def profile(ids=None):
    data = default_profile()
    data["network_confirmed"] = True
    if ids:
        data["robots"] = {rid: data["robots"][rid] for rid in ids}
    for value in data["robots"].values():
        value.update(wiring_verified=True, motion_calibrated=True, track_width_mm=120,
                     envelope_radius_mm=100)
        for name in ("left_forward", "left_reverse", "right_forward", "right_reverse"):
            value[name] = [[0, 0], [48, 90], [96, 180]]
    return HardwareConfig.from_dict(data)


class Socket:
    def __init__(self):
        self.sent = []
        self.received = []
        self.closed = False
        self.fail_on_send = False

    def setblocking(self, value):
        assert value is False

    def sendto(self, data, address):
        if self.fail_on_send:
            raise OSError("send failure")
        self.sent.append((json.loads(data), address))

    def recvfrom(self, size):
        if not self.received:
            raise BlockingIOError
        return self.received.pop(0)

    def close(self):
        self.closed = True

    def answer(self, request, address, **changes):
        response = dict(protocol="robo-hw", version=1, type="response",
            robot_id=request["robot_id"], request_id=request["request_id"],
            boot_id="boot_"+request["robot_id"], permit="permit_"+request["request_id"],
            state="disarmed" if request["type"] == "hello" else "armed",
            accepted=True, reason="ok", seq=request.get("seq", 0), lease_remaining_ms=250,
            left_pwm=request.get("left_pwm", 0), right_pwm=request.get("right_pwm", 0),
            servo_us=request.get("servo_us", [0, 0]), disc_present=None,
            hardware_enabled=True, uptime_ms=1000)
        response.update(changes)
        self.received.append((json.dumps(response).encode(), address))
        return response


def packet(t=0.04, sequence=1, ids=None, v=30, omega=0):
    return dict(schema_version=1, drive_model="differential_body", session_id="test-session",
        sequence=sequence, issued_at_s=t, ttl_s=0.25, emergency_stop=False, stop_reason=None,
        mock_motion_permitted=True, robots=[dict(robot_id=rid, forward_velocity_mm_s=v,
            angular_velocity_rad_s=omega, pose_heading_rad=0., velocity_world_mm_s=[v, 0.],
            wheel_velocity_rad_s=[]) for rid in (ids or default_profile()["robots"])])


class HardwareTests(unittest.TestCase):
    def setUp(self):
        self.socket = Socket()
        self.config = profile()
        self.transport = UdpFleetTransport(self.config, "a"*32, sock=self.socket)

    def arm(self):
        self.transport.connect(0)
        for request, address in list(self.socket.sent):
            self.socket.answer(request, address)
        self.transport.poll(0.01)
        for request, address in self.socket.sent[4:]:
            self.socket.answer(request, address)
        self.transport.poll(0.02)
        self.assertTrue(self.transport.ready)

    def test_default_is_valid_but_never_motion_ready(self):
        config = HardwareConfig.from_dict(default_profile())
        with self.assertRaises(ValueError):
            config.require_ready()

    def test_duplicate_keys_rejected(self):
        with self.assertRaises(ValueError):
            strict_json('{"version":1,"version":1}')

    def test_nonfinite_rejected(self):
        for value in ("NaN", "Infinity", "-Infinity"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                strict_json('{"a":'+value+'}')

    def test_duplicate_endpoint_rejected(self):
        data = default_profile()
        data["robots"]["H2"]["host"] = data["robots"]["H1"]["host"]
        with self.assertRaises(ValueError):
            HardwareConfig.from_dict(data)

    def test_host_must_be_ipv4_text_not_integer_or_multicast(self):
        for host in (123456, True, "224.0.0.1", "0.0.0.0", "255.255.255.255", "robot.local"):
            data = default_profile()
            data["robots"]["H1"]["host"] = host
            with self.subTest(host=host), self.assertRaises(ValueError):
                HardwareConfig.from_dict(data)

    def test_bool_and_nonmonotonic_curves_rejected(self):
        for curve in ([[0, 0], [True, 40]], [[0, 0], [30, 40], [40, 20]], [[10, 0], [20, 5]]):
            data = default_profile()
            data["robots"]["H1"]["left_forward"] = curve
            with self.assertRaises(ValueError):
                HardwareConfig.from_dict(data)

    def test_bad_pulse_or_unused_channel_rejected(self):
        for pulses in ([800, 0], [1500, 1500], [True, 0]):
            data = default_profile()
            data["robots"]["H1"]["servo_presets"] = {"open": pulses}
            with self.assertRaises(ValueError):
                HardwareConfig.from_dict(data)

    def test_differential_forward_reverse_turn(self):
        robot = self.config.robots["H1"]
        self.assertEqual(differential_pwm(robot, 90, 0), (48, 48))
        self.assertEqual(differential_pwm(robot, -90, 0), (-48, -48))
        self.assertEqual(differential_pwm(robot, 0, 1.5), (-48, 48))
        self.assertEqual(differential_pwm(robot, 0, 0), (0, 0))
        with self.assertRaises(ValueError):
            differential_pwm(robot, 180, 1.5)

    def test_handshake_all_zero_then_drive(self):
        self.arm()
        self.assertTrue(self.transport.send(packet(), 0.04))
        commands = self.socket.sent[-4:]
        self.assertTrue(all(r["type"] == "command" and r["left_pwm"] == 16 for r, _ in commands))
        self.assertFalse(self.transport.ready)
        for r, a in commands:
            self.socket.answer(r, a)
        self.transport.poll(0.05)
        self.assertTrue(self.transport.ready)

    def test_hello_continues_device_sequence(self):
        self.transport.connect(0)
        for r, a in list(self.socket.sent):
            self.socket.answer(r, a, seq=120)
        self.transport.poll(0.01)
        self.assertTrue(all(r["seq"] == 121 for r, _ in self.socket.sent[-4:]))

    def test_secret_not_in_snapshot(self):
        self.arm()
        self.assertNotIn("a"*32, json.dumps(self.transport.snapshot()))

    def test_no_automatic_retry_on_missing_ack(self):
        self.transport.connect(0)
        self.transport.poll(0.15)
        self.assertIn("ack_timeout", self.transport.fault)
        self.assertTrue(all(r["type"] == "stop" for r, _ in self.socket.sent[-4:]))
        count = len(self.socket.sent)
        self.transport.poll(0.2)
        self.assertEqual(len(self.socket.sent), count)
        with self.assertRaises(RuntimeError):
            self.transport.connect(0.21)

    def test_late_ack_cannot_rescue_timeout(self):
        self.transport.connect(0)
        for r, a in list(self.socket.sent):
            self.socket.answer(r, a)
        self.transport.poll(0.16)
        self.assertIn("late_ack", self.transport.fault)

    def test_stale_packet_or_packet_before_permit_stops_all(self):
        self.arm()
        self.assertFalse(self.transport.send(packet(t=0.005), 0.04))
        self.assertIsNotNone(self.transport.fault)
        self.assertTrue(all(r["type"] == "stop" for r, _ in self.socket.sent[-4:]))

    def test_incomplete_fleet_never_sends_partial_command(self):
        self.arm()
        self.assertFalse(self.transport.send(packet(ids=["H1"]), 0.04))
        self.assertFalse(any(r["type"] == "command" for r, _ in self.socket.sent))

    def test_lateral_motion_is_rejected(self):
        self.arm()
        p = packet()
        p["robots"][0]["velocity_world_mm_s"] = [0, 30]
        self.assertFalse(self.transport.send(p, 0.04))
        self.assertIsNotNone(self.transport.fault)

    def test_nonobject_robot_command_fails_closed(self):
        self.arm()
        p = packet()
        p["robots"][0] = None
        self.assertFalse(self.transport.send(p, .04))
        self.assertIsNotNone(self.transport.fault)
        self.assertFalse(any(r["type"] == "command" for r, _ in self.socket.sent))

    def test_duplicate_controller_packet_rejected(self):
        self.arm()
        p = packet()
        self.transport.send(p, 0.04)
        for r, a in self.socket.sent[-4:]:
            self.socket.answer(r, a)
        self.transport.poll(0.05)
        p["issued_at_s"] = 0.06
        self.assertFalse(self.transport.send(p, 0.06))

    def test_outputs_disabled_not_hardware_ready(self):
        self.transport.connect(0)
        r, a = self.socket.sent[0]
        self.socket.answer(r, a, hardware_enabled=False)
        self.transport.poll(0.01)
        self.assertIsNotNone(self.transport.fault)

    def test_reboot_and_wrong_robot_ack_trip(self):
        for change in ({"boot_id": "different_boot"}, {"robot_id": "other_robot"}):
            self.setUp()
            self.arm()
            self.transport.send(packet(), 0.04)
            r, a = self.socket.sent[-1]
            self.socket.answer(r, a, **change)
            self.transport.poll(0.05)
            self.assertIsNotNone(self.transport.fault)

    def test_duplicate_response_does_not_renew_deadline(self):
        self.arm()
        r, a = self.socket.sent[-1]
        self.socket.answer(r, a)
        self.transport.poll(0.1)
        self.transport.poll(0.271)
        self.assertIsNotNone(self.transport.fault)

    def test_wrong_address_ignored(self):
        self.transport.connect(0)
        r, _ = self.socket.sent[0]
        self.socket.answer(r, ("10.0.0.77", 4210))
        self.transport.poll(0.01)
        self.assertIsNone(self.transport.fault)
        self.assertFalse(self.transport.ready)

    def test_malformed_current_peer_response_stops_all(self):
        self.arm()
        self.socket.received.append((b'{"x":NaN}', (self.config.robots["H1"].host, 4210)))
        self.transport.poll(0.04)
        self.assertIsNotNone(self.transport.fault)

    def test_handshake_nonzero_output_rejected(self):
        self.transport.connect(0)
        r, a = self.socket.sent[0]
        self.socket.answer(r, a, left_pwm=1)
        self.transport.poll(0.01)
        self.assertIsNotNone(self.transport.fault)

    def test_handshake_nonzero_servo_rejected(self):
        self.transport.connect(0)
        r, a = self.socket.sent[0]
        self.socket.answer(r, a, servo_us=[1500, 0])
        self.transport.poll(0.01)
        self.assertIsNotNone(self.transport.fault)

    def test_firmware_golden_responses_match_production_client(self):
        fixture = json.loads((Path(__file__).resolve().parents[1] /
            "firmware/tests/fixtures/protocol_v1.json").read_text(encoding="utf-8"))
        sock = Socket()
        client = UdpFleetTransport(profile(["H1"]), "a"*32, sock=sock)
        client.connect(0)
        for kind, stamp in (("hello", .01), ("arm", .02)):
            request, address = sock.sent[-1]
            response = dict(fixture["responses"][kind], request_id=request["request_id"])
            sock.received.append((json.dumps(response).encode(), address))
            client.poll(stamp)
            self.assertIsNone(client.fault)
        self.assertTrue(client.ready)
        self.assertTrue(client.bench("H1", 40, 40, [0, 0], .03))
        request, address = sock.sent[-1]
        response = dict(fixture["responses"]["command"], request_id=request["request_id"])
        sock.received.append((json.dumps(response).encode(), address))
        client.poll(.04)
        self.assertTrue(client.ready, client.fault)

    def test_bench_only_one_robot(self):
        self.arm()
        with self.assertRaises(ValueError):
            self.transport.bench("H1", 20, 20, [0, 0], 0.04)

    def test_stop_or_close_no_motion_afterwards(self):
        self.arm()
        self.transport.stop("operator", emergency=True)
        self.assertTrue(all(r["type"] == "estop" for r, _ in self.socket.sent[-4:]))
        self.assertFalse(self.transport.send(packet(), 0.04))
        self.transport.close()
        self.assertTrue(self.socket.closed)


class BenchCliTests(unittest.TestCase):
    def test_floor_bench_stops_before_diagnostic_output(self):
        from robo_control.hardware import main
        transport = MagicMock()
        transport.fault, transport.ready = None, True
        transport.snapshot.return_value = {"device_io": True, "physical_stop_confirmed": False}
        def printed(*args, **kwargs):
            transport.close.assert_called()
        with (patch("robo_control.hardware.HardwareConfig.load", return_value=profile()),
              patch("robo_control.hardware.UdpFleetTransport", return_value=transport),
              patch("robo_control.hardware.time.monotonic", side_effect=[0., .01, .6]),
              patch("robo_control.hardware.time.sleep"), patch("builtins.print", side_effect=printed)):
            result = main(["bench", "--config", "not-read.json", "--robot", "H1",
                           "--duration-s", "0.5", "--enable-hardware", "--clear-test-lane"])
        self.assertEqual(result, 0)
        transport.bench.assert_called_once()

    def test_floor_bench_rejects_long_pulse_before_network(self):
        from robo_control.hardware import main
        with (patch("robo_control.hardware.HardwareConfig.load", return_value=profile()),
              patch("robo_control.hardware.UdpFleetTransport") as factory,
              patch("sys.stderr"), self.assertRaises(SystemExit) as raised):
            main(["bench", "--config", "not-read.json", "--robot", "H1",
                  "--duration-s", "0.501", "--enable-hardware", "--clear-test-lane"])
        self.assertEqual(raised.exception.code, 2)
        factory.assert_not_called()


if __name__ == "__main__":
    unittest.main()
