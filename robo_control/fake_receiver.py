"""Receiver-clock protocol reference; no socket, motor, sensor or pose model.

The harness MUST tick each receiver independently of the host sender. This is
an executable contract for future firmware, not an independent physical stop.
"""
from __future__ import annotations

import math
import re
import uuid

from .control_loop import ControlLimits
from .fleet import robot_id_valid
from .wire_codec import WireError, decode_frame, encode_frame

PROTOCOL = "robo-wire"
VERSION = 1
MAX_SEQUENCE = 2**53 - 1
_TOKEN = re.compile(r"^[A-Za-z0-9_-]{1,128}$")
_COMMON = {"protocol", "version", "type", "robot_id"}
_CONTEXT = {"host_session_id", "boot_id", "link_id", "seq"}
_FIELDS = {
    "hello": _COMMON | {"host_session_id", "hello_id"},
    "arm": _COMMON | _CONTEXT | {"challenge_id"},
    "drive": _COMMON | _CONTEXT | {"permit_id", "ttl_ms", "v_mm_s", "omega_rad_s"},
    "stop": _COMMON | _CONTEXT | {"reason"},
    "estop": _COMMON | _CONTEXT | {"reason"},
    "status": _COMMON | _CONTEXT,
}


def _token(value):
    return isinstance(value, str) and _TOKEN.fullmatch(value) is not None


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _sequence(value):
    return type(value) is int and 0 < value <= MAX_SEQUENCE


class FakeRobotReceiver:
    """One addressed robot, one boot, one explicitly armed link at a time.

    ACKs describe simulated setpoint acceptance only. Permits are freshness
    challenges, NOT authentication. A production transport needs its own peer
    authentication/integrity, firmware watchdog and physical emergency stop.
    """

    def __init__(self, robot_id, *, lease_ms=300, boot_id=None, limits=None):
        if not robot_id_valid(robot_id):
            raise ValueError("A registered robot ID is required")
        if type(lease_ms) is not int or not 1 <= lease_ms <= 300:
            raise ValueError("Receiver permits expire within 300 ms")
        if boot_id is not None and not _token(boot_id):
            raise ValueError("Boot ID must be a unique token for this boot")
        self.robot_id = robot_id
        self.boot_id = boot_id or uuid.uuid4().hex
        self.lease_ms = lease_ms
        self.limits = ControlLimits() if limits is None else limits
        if not isinstance(self.limits, ControlLimits):
            raise TypeError("ControlLimits required")
        if self.limits.max_speed_mm_s > 180 or self.limits.max_turn_rad_s > 1.5:
            raise ValueError("Wire v1 permits tightening, not increasing the 180 mm/s and 1.5 rad/s test limits")
        self.host_session_id = self.link_id = self.hello_id = None
        self._challenge = self._permit = None
        self._challenge_deadline = self._permit_issued = self._deadline = None
        self._now = None
        self._clock_broken = False
        self._estop = False
        self._armed = False
        self._last_seq = 0
        self._accepted_drives = 0
        self._v = self._omega = 0.
        self.reason = "boot_disarmed"

    def _halt(self, reason):
        self._v = self._omega = 0.
        self._armed = False
        self._challenge = self._permit = None
        self._challenge_deadline = self._permit_issued = self._deadline = None
        self.reason = reason

    def _time(self, now_s):
        if (self._clock_broken or not _finite(now_s) or now_s < 0
                or (self._now is not None and now_s < self._now)):
            self._clock_broken = True
            self._halt("invalid_receiver_clock")
            raise ValueError("Receiver clock must be finite, nonnegative and monotonic; reboot after clock fault")
        self._now = float(now_s)
        if self._armed and self._now >= self._deadline - 1e-12:
            self._halt("command_watchdog")
        elif self._challenge and self._now >= self._challenge_deadline - 1e-12:
            self._halt("handshake_expired")

    def _new_permit(self):
        self._permit = uuid.uuid4().hex
        self._permit_issued = self._now

    def snapshot(self):
        """Historical diagnostic; call tick(now_s) to apply elapsed time."""
        return {
            "robot_id": self.robot_id, "boot_id": self.boot_id,
            "host_session_id": self.host_session_id, "link_id": self.link_id,
            "state": "estop" if self._estop else "armed" if self._armed else "disarmed",
            "reason": self.reason, "last_sequence": self._last_seq,
            "accepted_drive_count": self._accepted_drives,
            "v_mm_s": self._v, "omega_rad_s": self._omega,
            "receiver_at_s": self._now, "command_deadline_receiver_s": self._deadline,
            "synthetic": True, "execution": "unknown", "measured_velocity": None,
            "device_io": False, "hardware_ready": False, "motion_permitted": False,
        }

    def tick(self, now_s):
        self._time(now_s)
        return self.snapshot()

    def disconnect(self, now_s):
        self._time(now_s)
        self._halt("link_disconnected")
        return self.snapshot()

    def emergency_stop(self, now_s):
        self._time(now_s)
        self._estop = True
        self._halt("emergency_stop")
        return self.snapshot()

    def reset_emergency_stop(self, now_s):
        """Local test/operator action, deliberately not a wire message."""
        self._time(now_s)
        self._estop = False
        self._halt("local_reset_requires_handshake")
        return self.snapshot()

    def _reply(self, message, result, reason):
        message = message if isinstance(message, dict) else {}
        kind = message.get("type")
        request_type = kind if isinstance(kind, str) and kind in _FIELDS else "invalid"
        seq = message.get("seq")
        deadline = (self._permit_issued + self.lease_ms / 1000 if self._permit else
                    self._challenge_deadline if self._challenge else None)
        remaining = 0 if deadline is None else max(0, min(self.lease_ms, math.floor((deadline - self._now)*1000 + 1e-9)))
        return {
            "protocol": PROTOCOL, "version": VERSION, "type": "response",
            "robot_id": self.robot_id, "host_session_id": self.host_session_id,
            "boot_id": self.boot_id, "link_id": self.link_id, "hello_id": self.hello_id,
            "request_type": request_type, "seq": seq if _sequence(seq) else None,
            "result": result, "reason": reason,
            "state": "estop" if self._estop else "armed" if self._armed else "disarmed",
            "challenge_id": self._challenge, "permit_id": self._permit,
            "lease_remaining_ms": remaining, "v_mm_s": self._v, "omega_rad_s": self._omega,
            "synthetic": True, "execution": "unknown",
            "device_io": False, "hardware_ready": False, "motion_permitted": False,
        }

    def _reject(self, message, reason, *, halt=True):
        if halt:
            self._halt(reason)
        return self._reply(message, "rejected", reason)

    def receive_frame(self, frame, now_s):
        """Receive exactly one framed message. No physical transport is opened."""
        self._time(now_s)
        try:
            message = decode_frame(frame)
        except WireError:
            return encode_frame(self._reject(None, "malformed_wire"))
        return encode_frame(self.receive(message, now_s))

    def receive(self, message, now_s):
        self._time(now_s)
        if not isinstance(message, dict):
            return self._reject(message, "malformed_message")
        # A correctly addressed packet for another endpoint never changes this
        # robot's command/sequence. Time can still independently expire it.
        rid = message.get("robot_id")
        if robot_id_valid(rid) and rid != self.robot_id:
            return self._reject(message, "wrong_robot", halt=False)
        kind = message.get("type")
        if (message.get("protocol") != PROTOCOL or type(message.get("version")) is not int
                or message["version"] != VERSION or rid != self.robot_id
                or not isinstance(kind, str) or kind not in _FIELDS or set(message) != _FIELDS[kind]):
            return self._reject(message, "invalid_schema")
        if kind == "hello":
            if not _token(message["host_session_id"]) or not _token(message["hello_id"]):
                return self._reject(message, "invalid_identity")
            self._halt("new_handshake")
            if self._estop:
                return self._reply(message, "rejected", "emergency_stop")
            self.host_session_id, self.hello_id = message["host_session_id"], message["hello_id"]
            self.link_id = uuid.uuid4().hex
            self._last_seq = 0
            self._challenge = uuid.uuid4().hex
            self._challenge_deadline = self._now + self.lease_ms / 1000
            return self._reply(message, "challenge", "explicit_arm_required")
        if (message["host_session_id"] != self.host_session_id or message["boot_id"] != self.boot_id
                or message["link_id"] != self.link_id or self.link_id is None):
            return self._reject(message, "wrong_session_or_link")
        seq = message["seq"]
        if not _sequence(seq):
            return self._reject(message, "invalid_sequence")
        # Stop is deliberately idempotent and takes precedence over sequence
        # ordering in the CURRENT link. It cannot authorize further motion.
        if kind in {"stop", "estop"}:
            if not isinstance(message["reason"], str) or not 1 <= len(message["reason"]) <= 128:
                return self._reject(message, "invalid_stop_reason")
            self._last_seq = max(self._last_seq, seq)
            self._estop = self._estop or kind == "estop"
            self._halt("emergency_stop" if self._estop else "remote_stop")
            return self._reply(message, "accepted", self.reason)
        if seq <= self._last_seq:
            return self._reject(message, "out_of_order_sequence")
        self._last_seq = seq  # A well-scoped rejected attempt cannot be repaired/replayed.
        if self._estop:
            return self._reject(message, "emergency_stop")
        if kind == "status":
            return self._reply(message, "status", self.reason)
        if kind == "arm":
            if (self._armed or self._challenge is None or message["challenge_id"] != self._challenge):
                return self._reject(message, "invalid_arm_challenge")
            self._challenge = self._challenge_deadline = None
            self._armed = True
            self._v = self._omega = 0.
            self._deadline = self._now + self.lease_ms / 1000
            self._new_permit()
            self.reason = "armed_zero"
            return self._reply(message, "accepted", self.reason)
        if not self._armed or self._permit is None:
            return self._reject(message, "not_armed")
        if message["permit_id"] != self._permit:
            return self._reject(message, "invalid_permit")
        permit_issued = self._permit_issued
        self._permit = self._permit_issued = None  # One attempt, even if payload fails.
        ttl, v, omega = (message[k] for k in ("ttl_ms", "v_mm_s", "omega_rad_s"))
        if (type(ttl) is not int or not 1 <= ttl <= self.lease_ms or not _finite(v) or not _finite(omega)
                or abs(v) > self.limits.max_speed_mm_s or abs(omega) > self.limits.max_turn_rad_s):
            return self._reject(message, "invalid_drive_payload")
        deadline = permit_issued + ttl / 1000
        if self._now >= deadline - 1e-12:
            return self._reject(message, "expired_permit")
        self._v, self._omega, self._deadline = float(v), float(omega), deadline
        self._accepted_drives += 1
        self.reason = "setpoint_accepted_unverified"
        self._new_permit()
        return self._reply(message, "accepted", self.reason)
