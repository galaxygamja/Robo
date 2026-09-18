"""Physical ESP32-C3/DRV8833 profile, calibrated PWM mapping and bounded UDP IO.

This module is separate from the legacy synthetic robo-wire protocol. Network
output only exists after explicit transport construction. Import/validate/init
never opens a socket. PWM is a requested duty, NOT measured wheel velocity.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import math
import os
import re
import secrets
import socket
import time
from dataclasses import dataclass, field
from pathlib import Path

PROTOCOL = "robo-hw"
VERSION = 1
MAX_DATAGRAM = 1024
LEASE_S = 0.250
TOKEN = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def strict_json(data):
    if isinstance(data, bytes):
        data = data.decode("utf-8", errors="strict")
    return json.loads(data, object_pairs_hook=_pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError("nonfinite JSON")))


def _curve(data, maximum):
    if not isinstance(data, list):
        raise ValueError("wheel calibration must be a list of [PWM, measured mm/s]")
    if not data:
        return ()
    previous = (-1, -1.0)
    result = []
    for point in data:
        if (not isinstance(point, list) or len(point) != 2 or type(point[0]) is not int
                or not 0 <= point[0] <= maximum or not finite(point[1]) or not 0 <= point[1] <= 1000
                or point[0] <= previous[0] or point[1] <= previous[1]):
            raise ValueError("wheel curve needs strictly increasing PWM and measured speed")
        previous = tuple(point)
        result.append(previous)
    if len(result) < 2 or result[0] != (0, 0):
        raise ValueError("wheel curve must start [0,0] and contain measured nonzero points")
    return tuple(result)


@dataclass(frozen=True)
class RobotHardware:
    robot_id: str
    role: str
    display_name: str
    host: str
    port: int
    max_pwm: int
    track_width_mm: float | None
    envelope_radius_mm: float | None
    wiring_verified: bool
    motion_calibrated: bool
    sensor_verified: bool
    sensor_active_low: bool
    left_forward: tuple
    left_reverse: tuple
    right_forward: tuple
    right_reverse: tuple
    servo_presets: dict

    @classmethod
    def from_dict(cls, rid, value):
        from .fleet import robot_id_valid
        if not robot_id_valid(rid) or not isinstance(value, dict):
            raise ValueError("invalid hardware robot identity")
        expected = set(cls.__dataclass_fields__) - {"robot_id"}
        if set(value) != expected:
            raise ValueError(f"{rid}: hardware fields differ: {sorted(set(value) ^ expected)}")
        if value["role"] not in {"hamster", "beaver"}:
            raise ValueError("unknown robot role")
        if not isinstance(value["display_name"], str) or not value["display_name"]:
            raise ValueError("robot display name required")
        if not isinstance(value["host"], str):
            raise ValueError("device IPv4 address must be a string")
        address = ipaddress.IPv4Address(value["host"])
        if address.is_multicast or address.is_unspecified or str(address) == "255.255.255.255":
            raise ValueError("use an individual IPv4 device address")
        if type(value["port"]) is not int or not 1024 <= value["port"] <= 65535:
            raise ValueError("invalid UDP device port")
        maximum = value["max_pwm"]
        if type(maximum) is not int or not 1 <= maximum <= 96:
            raise ValueError("commissioning PWM limit must be 1..96/255")
        for key in ("track_width_mm", "envelope_radius_mm"):
            if value[key] is not None and (not finite(value[key]) or not 10 <= value[key] <= 500):
                raise ValueError(f"{key} needs measured positive mm or null")
        for key in ("wiring_verified", "motion_calibrated", "sensor_verified", "sensor_active_low"):
            if type(value[key]) is not bool:
                raise ValueError(f"{key} must be Boolean")
        parsed = dict(value)
        for key in ("left_forward", "left_reverse", "right_forward", "right_reverse"):
            parsed[key] = _curve(value[key], maximum)
        presets = value["servo_presets"]
        if not isinstance(presets, dict) or any(not isinstance(k, str) or not TOKEN.fullmatch(k) for k in presets):
            raise ValueError("servo_presets must map action names to two calibrated pulses")
        parsed["servo_presets"] = {}
        for action, pulses in presets.items():
            validate_servos(pulses, value["role"])
            parsed["servo_presets"][action] = tuple(pulses)
        return cls(robot_id=rid, **parsed)

    def require_ready(self):
        if not self.wiring_verified or not self.motion_calibrated:
            raise ValueError(f"{self.robot_id}: verify wiring and measure motor calibration first")
        if self.track_width_mm is None or self.envelope_radius_mm is None:
            raise ValueError(f"{self.robot_id}: measure wheel centre spacing and complete swept radius")
        if any(not getattr(self, name) for name in ("left_forward", "left_reverse", "right_forward", "right_reverse")):
            raise ValueError(f"{self.robot_id}: all four measured wheel speed curves required")


@dataclass(frozen=True)
class HardwareConfig:
    robots: dict[str, RobotHardware]
    network_confirmed: bool = False

    @classmethod
    def load(cls, path):
        return cls.from_dict(strict_json(Path(path).read_text(encoding="utf-8-sig")))

    @classmethod
    def from_dict(cls, data):
        if (not isinstance(data, dict) or set(data) != {"schema_version", "network_confirmed", "robots"}
                or type(data["schema_version"]) is not int or data["schema_version"] != 1
                or type(data["network_confirmed"]) is not bool
                or not isinstance(data["robots"], dict) or not data["robots"]):
            raise ValueError("hardware profile requires schema_version=1, network_confirmed, robots")
        robots = {rid: RobotHardware.from_dict(rid, value) for rid, value in data["robots"].items()}
        if len({(robot.host, robot.port) for robot in robots.values()}) != len(robots):
            raise ValueError("each robot needs a distinct endpoint")
        return cls(robots, data["network_confirmed"])

    def require_ready(self):
        if not self.network_confirmed:
            raise ValueError("confirm robot IPv4 addresses on the dedicated Wi-Fi first")
        for robot in self.robots.values():
            robot.require_ready()


def default_profile():
    robots = {}
    for index, (rid, role, label) in enumerate([
            ("H1", "hamster", "햄스터"), ("B1", "beaver", "한가한 비버"),
            ("B2", "beaver", "바쁜 비버"), ("H2", "beaver", "세 번째 비버 B3")]):
        robots[rid] = dict(role=role, display_name=label, host=f"192.168.4.{101+index}",
            port=4210, max_pwm=96, track_width_mm=None, envelope_radius_mm=None,
            wiring_verified=False, motion_calibrated=False, sensor_verified=False,
            sensor_active_low=True, left_forward=[], left_reverse=[], right_forward=[],
            right_reverse=[], servo_presets={})
    return {"schema_version": 1, "network_confirmed": False, "robots": robots}


def validate_servos(pulses, role):
    if (not isinstance(pulses, (list, tuple)) or len(pulses) != 2
            or any(type(p) is not int or (p != 0 and not 900 <= p <= 2100) for p in pulses)
            or role == "hamster" and pulses[1] != 0):
        raise ValueError("servo pulses need [channel0,channel1], 0=off or 900..2100 us; H1 second channel off")


def _speed_pwm(speed, forward, reverse):
    if speed == 0:
        return 0
    table = forward if speed > 0 else reverse
    target = abs(speed)
    for (p0, v0), (p1, v1) in zip(table, table[1:]):
        if target <= v1:
            pwm = round(p0 + (target - v0) * (p1 - p0) / (v1 - v0))
            return pwm if speed > 0 else -pwm
    raise ValueError("command exceeds measured wheel speed curve")


def differential_pwm(robot: RobotHardware, forward_mm_s, omega_rad_s):
    robot.require_ready()
    if not finite(forward_mm_s) or not finite(omega_rad_s):
        raise ValueError("finite body velocities required")
    left = forward_mm_s - omega_rad_s * robot.track_width_mm / 2
    right = forward_mm_s + omega_rad_s * robot.track_width_mm / 2
    # A requested speed outside calibration is rejected, not silently clipped.
    return (_speed_pwm(left, robot.left_forward, robot.left_reverse),
            _speed_pwm(right, robot.right_forward, robot.right_reverse))


@dataclass
class _Peer:
    state: str = "idle"
    boot_id: str | None = None
    permit: str | None = None
    permit_received_at: float = -math.inf
    pending: dict | None = None
    sent_at: float = 0.0
    seq: int = 0
    telemetry: dict = field(default_factory=dict)
    received_at_s: float | None = None


class UdpFleetTransport:
    """One outstanding request per device; no retransmit/reconnect/replay queue."""

    def __init__(self, config: HardwareConfig, token: str, *, sock=None):
        if not isinstance(token, str) or not TOKEN.fullmatch(token) or len(token) < 24:
            raise ValueError("ROBO_HW_TOKEN must contain 24..64 letters/digits/_/-")
        self.config, self._token = config, token
        self.peers = {rid: _Peer() for rid in config.robots}
        self.socket = sock if sock is not None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.fault = None
        self.started = self.closed = False
        self.now = -math.inf
        self._session = None
        self._sequence = 0
        self._issued_at = -math.inf

    @property
    def ready(self):
        return (self.started and not self.closed and self.fault is None
                and all(p.state == "armed" and p.pending is None and p.permit
                        and self.now - p.permit_received_at < LEASE_S for p in self.peers.values()))

    def _clock(self, now):
        if not finite(now) or now < self.now:
            self.stop("invalid_host_clock")
            raise ValueError("hardware transport requires monotonic finite time")
        self.now = now

    def _request(self, rid, kind, now, **payload):
        peer = self.peers[rid]
        request = dict(protocol=PROTOCOL, version=VERSION, robot_id=rid,
                       token=self._token, request_id=secrets.token_hex(16), type=kind, **payload)
        encoded = json.dumps(request, separators=(",", ":"), allow_nan=False).encode()
        if len(encoded) > MAX_DATAGRAM:
            raise ValueError("hardware datagram too large")
        robot = self.config.robots[rid]
        self.socket.sendto(encoded, (robot.host, robot.port))
        # Never store the shared token in snapshots/reports.
        peer.pending = {k: v for k, v in request.items() if k != "token"}
        peer.sent_at = now

    def connect(self, now_s):
        self._clock(now_s)
        if self.started or self.closed or self.fault:
            raise RuntimeError("new transport/session required after stop or disconnect")
        if not self.config.network_confirmed:
            raise ValueError("robot addresses not confirmed")
        self.started = True
        try:
            for rid in self.peers:
                self._request(rid, "hello", now_s)
                self.peers[rid].state = "connecting"
        except (OSError, ValueError) as exc:
            self.stop("hello_send_failed")
            raise RuntimeError("hardware hello failed") from exc

    def _accept(self, rid, response, now):
        peer = self.peers[rid]
        pending = peer.pending
        if not isinstance(response, dict):
            raise ValueError("response must be an object")
        if pending is None or response.get("request_id") != pending["request_id"]:
            return  # Old/duplicate/unsolicited responses cannot refresh a permit.
        if now - peer.sent_at >= 0.150:
            raise ValueError("late_ack:" + rid)
        if (response.get("protocol") != PROTOCOL or type(response.get("version")) is not int
                or response["version"] != VERSION or response.get("type") != "response"
                or response.get("robot_id") != rid or type(response.get("accepted")) is not bool
                or type(response.get("hardware_enabled")) is not bool
                or response.get("state") not in {"armed", "disarmed", "estop"}
                or not isinstance(response.get("boot_id"), str) or not TOKEN.fullmatch(response["boot_id"])
                or not isinstance(response.get("reason"), str)
                or type(response.get("seq")) is not int or not 0 <= response["seq"] < 2147483647
                or type(response.get("lease_remaining_ms")) is not int
                or not 0 <= response["lease_remaining_ms"] <= 250
                or type(response.get("uptime_ms")) is not int
                or response["uptime_ms"] < 0
                or any(type(response.get(key)) is not int or abs(response[key]) > self.config.robots[rid].max_pwm
                       for key in ("left_pwm", "right_pwm"))
                or (response.get("disc_present") is not None and type(response["disc_present"]) is not bool)):
            raise ValueError("malformed hardware response")
        validate_servos(response.get("servo_us"), self.config.robots[rid].role)
        if not response["accepted"] or not response["hardware_enabled"]:
            raise ValueError("device_rejected_or_outputs_disabled:" + response["reason"])
        kind = pending["type"]
        if kind != "hello" and response["boot_id"] != peer.boot_id:
            raise ValueError("device rebooted")
        if kind in {"arm", "command"} and response["seq"] != pending["seq"]:
            raise ValueError("wrong ACK sequence")
        expected_state = "disarmed" if kind == "hello" else "armed"
        if response["state"] != expected_state:
            raise ValueError("device_not_in_expected_state")
        permit = response.get("permit")
        if not isinstance(permit, str) or not TOKEN.fullmatch(permit) or response["lease_remaining_ms"] <= 0:
            raise ValueError("missing/expired device permit")
        if kind in {"hello", "arm"} and (any(response[key] != 0 for key in ("left_pwm", "right_pwm"))
                                        or response["servo_us"] != [0, 0]):
            raise ValueError("handshake_output_not_zero")
        if kind == "command" and any(response[key] != pending[key] for key in ("left_pwm", "right_pwm", "servo_us")):
            raise ValueError("output_ACK_mismatch")
        peer.boot_id, peer.permit, peer.permit_received_at = response["boot_id"], permit, now
        peer.telemetry = {k: response.get(k) for k in (
            "left_pwm", "right_pwm", "servo_us", "disc_present", "hardware_enabled", "uptime_ms", "reason")}
        peer.telemetry["request_sent_at_s"] = peer.sent_at
        peer.received_at_s = now
        peer.pending = None
        peer.state = expected_state
        if kind == "hello":
            peer.seq = response["seq"] + 1
            self._request(rid, "arm", now, boot_id=peer.boot_id, permit=peer.permit, seq=peer.seq)
            peer.state = "connecting"

    def poll(self, now_s):
        self._clock(now_s)
        if self.closed or self.fault:
            return self.snapshot()
        endpoints = {(r.host, r.port): rid for rid, r in self.config.robots.items()}
        try:
            # A flood cannot monopolize the host supervisor.
            for _ in range(64):
                try:
                    data, address = self.socket.recvfrom(MAX_DATAGRAM + 1)
                except BlockingIOError:
                    break
                rid = endpoints.get(address)
                if rid is None:
                    continue
                if len(data) > MAX_DATAGRAM:
                    raise ValueError("oversized response")
                self._accept(rid, strict_json(data), now_s)
            for rid, peer in self.peers.items():
                if peer.pending is not None and now_s - peer.sent_at >= 0.150:
                    raise ValueError("ack_timeout:" + rid)
                if peer.state == "armed" and now_s - peer.permit_received_at >= LEASE_S:
                    raise ValueError("host_permit_expired:" + rid)
        except (OSError, ValueError, TypeError, KeyError, RecursionError) as exc:
            self.stop("transport_fault:" + str(exc))
        return self.snapshot()

    def _send_outputs(self, outputs, now_s, ttl_ms):
        if not self.ready:
            return False
        if type(ttl_ms) is not int or not 1 <= ttl_ms <= 250 or set(outputs) != set(self.peers):
            raise ValueError("complete fleet and bounded TTL required")
        for rid, (left, right, servo) in outputs.items():
            robot = self.config.robots[rid]
            if any(type(v) is not int or abs(v) > robot.max_pwm for v in (left, right)):
                raise ValueError("PWM outside robot limit")
            validate_servos(servo, robot.role)
        try:
            for rid, (left, right, servo) in outputs.items():
                peer = self.peers[rid]
                if peer.seq >= 2147483646:
                    self.stop("sequence_exhausted")
                    return False
                peer.seq += 1
                self._request(rid, "command", now_s, boot_id=peer.boot_id, permit=peer.permit,
                    seq=peer.seq, ttl_ms=ttl_ms, left_pwm=left, right_pwm=right, servo_us=list(servo))
        except OSError:
            self.stop("command_send_failed")
            return False
        return True

    def send(self, packet, now_s, servo_us=None):
        self._clock(now_s)
        if not self.ready:
            return False
        try:
            self.config.require_ready()
            if (not isinstance(packet, dict) or packet.get("drive_model") != "differential_body"
                    or type(packet.get("sequence")) is not int or packet["sequence"] <= self._sequence
                    or not isinstance(packet.get("session_id"), str) or not packet["session_id"]
                    or self._session is not None and packet["session_id"] != self._session
                    or packet.get("emergency_stop") is not False or packet.get("stop_reason") is not None
                    or packet.get("mock_motion_permitted") is not True):
                raise ValueError("invalid/held differential controller packet")
            issued, ttl = packet.get("issued_at_s"), packet.get("ttl_s")
            if (not finite(issued) or not finite(ttl) or not 0 < ttl <= 0.3
                    or not self._issued_at <= issued <= now_s or now_s - issued >= min(ttl, LEASE_S)
                    or any(issued < p.permit_received_at for p in self.peers.values())):
                raise ValueError("packet predates permit or is stale")
            commands = packet.get("robots")
            if not isinstance(commands, list) or len(commands) != len(self.peers):
                raise ValueError("incomplete command fleet")
            outputs = {}
            servo_us = servo_us or {rid: [0, 0] for rid in self.peers}
            if set(servo_us) != set(self.peers):
                raise ValueError("complete servo map required")
            for cmd in commands:
                if not isinstance(cmd, dict):
                    raise ValueError("each robot command must be an object")
                rid = cmd.get("robot_id")
                if rid not in self.peers or rid in outputs:
                    raise ValueError("unknown/duplicate command robot")
                v, w, heading = cmd.get("forward_velocity_mm_s"), cmd.get("angular_velocity_rad_s"), cmd.get("pose_heading_rad")
                world = cmd.get("velocity_world_mm_s")
                if (not all(finite(x) for x in (v, w, heading)) or abs(v) > 180 or abs(w) > 1.5
                        or cmd.get("wheel_velocity_rad_s") != [] or not isinstance(world, (list, tuple)) or len(world) != 2
                        or not all(finite(x) for x in world)
                        or abs(world[0] - v * math.cos(heading)) > 1e-6
                        or abs(world[1] - v * math.sin(heading)) > 1e-6):
                    raise ValueError("lateral/invalid/out-of-bounds differential command")
                left, right = differential_pwm(self.config.robots[rid], v, w)
                outputs[rid] = (left, right, servo_us[rid])
            self._session, self._sequence, self._issued_at = packet["session_id"], packet["sequence"], issued
            return self._send_outputs(outputs, now_s, min(250, math.floor(ttl * 1000)))
        except (ValueError, TypeError, KeyError) as exc:
            self.stop("invalid_command:" + str(exc))
            return False

    def bench(self, robot_id, left_pwm, right_pwm, servo_us, now_s):
        """Explicit one-robot commissioning only, independent of speed curves."""
        self._clock(now_s)
        if len(self.peers) != 1 or robot_id not in self.peers:
            raise ValueError("bench transport must contain one selected robot")
        if not self.config.robots[robot_id].wiring_verified:
            raise ValueError("verify pin map and power rails before bench output")
        return self._send_outputs({robot_id: (left_pwm, right_pwm, servo_us)}, now_s, 200)

    def stop(self, reason="operator_stop", emergency=False):
        if self.closed:
            return
        self.fault = self.fault or str(reason)[:160]
        for rid, peer in self.peers.items():
            if self.started:
                try:
                    self._request(rid, "estop" if emergency else "stop", max(0.0, self.now))
                except (OSError, ValueError):
                    pass  # Firmware watchdog is independent of successful stop delivery.
            peer.state, peer.permit, peer.pending = "stopped", None, None

    def snapshot(self):
        return {"protocol": PROTOCOL, "ready": self.ready, "fault": self.fault,
                "device_io": self.started, "physical_stop_confirmed": False,
                "robots": {rid: {"state": p.state, "pending": p.pending["type"] if p.pending else None,
                    "request_sent_at_s": p.telemetry.get("request_sent_at_s"),
                    "received_at_s": p.received_at_s, "telemetry": dict(p.telemetry)} for rid, p in self.peers.items()}}

    def close(self):
        if not self.closed:
            self.stop("transport_closed")
            self.socket.close()
            self.closed = True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    init = sub.add_parser("init", help="write an uncalibrated local profile; no network IO")
    init.add_argument("--output", type=Path, default=Path("local_state/hardware.json"))
    validate = sub.add_parser("validate", help="validate and list missing commissioning measurements")
    validate.add_argument("--config", type=Path, required=True)
    bench = sub.add_parser("bench", help="one robot, finite low PWM/servo commissioning test")
    bench.add_argument("--config", type=Path, required=True)
    bench.add_argument("--robot", required=True)
    bench.add_argument("--left-pwm", type=int, default=0)
    bench.add_argument("--right-pwm", type=int, default=0)
    bench.add_argument("--servo-preset", help="calibrated named profile pose; default no servo pulses")
    bench.add_argument("--duration-s", type=float, default=0.5)
    bench.add_argument("--enable-hardware", action="store_true")
    test_mode = bench.add_mutually_exclusive_group()
    test_mode.add_argument("--wheels-raised", action="store_true")
    test_mode.add_argument("--clear-test-lane", action="store_true", help="supervised single-robot floor calibration, <=0.5s")
    args = parser.parse_args(argv)
    try:
        if args.action == "init":
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as file:
                json.dump(default_profile(), file, ensure_ascii=False, indent=2)
                file.write("\n")
            print(f"Created {args.output}; uncalibrated, output disabled. No device contacted.")
            return 0
        config = HardwareConfig.load(args.config)
        if args.action == "validate":
            missing = []
            if not config.network_confirmed:
                missing.append("network_confirmed is false")
            for robot in config.robots.values():
                try:
                    robot.require_ready()
                except ValueError as exc:
                    missing.append(str(exc))
            print(json.dumps({"valid_profile": True, "motion_ready": not missing,
                              "missing": missing, "device_io": False}, ensure_ascii=False, indent=2))
            return 2 if missing else 0
        if not args.enable_hardware or not (args.wheels_raised or args.clear_test_lane):
            raise ValueError("bench requires --enable-hardware and --wheels-raised or --clear-test-lane")
        max_duration = 0.5 if args.clear_test_lane else 2.0
        if not finite(args.duration_s) or not 0 < args.duration_s <= max_duration:
            raise ValueError(f"bench duration must be 0..{max_duration} seconds")
        robot = config.robots[args.robot]
        if not config.network_confirmed or not robot.wiring_verified:
            raise ValueError("confirm device address, wiring and measured supply voltages first")
        pulses = robot.servo_presets[args.servo_preset] if args.servo_preset else [0, 0]
        for pwm in (args.left_pwm, args.right_pwm):
            if abs(pwm) > robot.max_pwm:
                raise ValueError("requested bench PWM exceeds profile limit")
        transport = UdpFleetTransport(HardwareConfig({args.robot: robot}, True), os.environ.get("ROBO_HW_TOKEN", ""))
        try:
            transport.connect(time.monotonic())
            started = None
            while not transport.fault:
                now = time.monotonic()
                if started is not None and now - started >= args.duration_s:
                    break
                transport.poll(now)
                if transport.ready:
                    started = now if started is None else started
                    transport.bench(args.robot, args.left_pwm, args.right_pwm, pulses, now)
                time.sleep(0.02)
            failed = bool(transport.fault)
            # Stop before diagnostic IO: stdout can block indefinitely.
            transport.close()
            print(json.dumps(transport.snapshot(), ensure_ascii=False, indent=2))
            return 1 if failed else 0
        finally:
            transport.close()
    except (ValueError, KeyError, OSError, RuntimeError) as exc:
        parser.exit(2, f"Hardware configuration/error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
