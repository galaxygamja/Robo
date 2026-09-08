"""Fail-closed host-side protocol adapter for an entirely synthetic receiver.

Only complete differential controller packets can produce drive messages. Host
and receiver clocks are deliberately never compared. A received, single-use
permit must precede the controller packet it authorizes (causal freshness).
Nothing here opens a transport, reports physical completion, or enables motors.
"""
from __future__ import annotations

import copy
import math
import re
import uuid

from .control_loop import ControlLimits
from .fleet import DEFAULT_ROLES, robot_id_valid

_TOKEN = re.compile(r"[A-Za-z0-9_-]{1,128}\Z")
_MAX_SEQUENCE = 2**53 - 1
_RESPONSE_FIELDS = {"protocol", "version", "type", "robot_id", "host_session_id",
                    "boot_id", "link_id", "hello_id", "request_type", "seq", "result",
                    "reason", "state", "challenge_id", "permit_id", "lease_remaining_ms",
                    "v_mm_s", "omega_rad_s", "synthetic", "execution", "device_io",
                    "hardware_ready", "motion_permitted"}


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _token(value):
    return isinstance(value, str) and _TOKEN.fullmatch(value) is not None


def _response_valid(message):
    if not isinstance(message, dict) or set(message) != _RESPONSE_FIELDS:
        return False
    return (message["protocol"] == "robo-wire" and type(message["version"]) is int
            and message["version"] == 1 and message["type"] == "response"
            and robot_id_valid(message["robot_id"]) and _token(message["boot_id"])
            and all(message[key] is None or _token(message[key]) for key in
                    ("host_session_id", "link_id", "hello_id", "challenge_id", "permit_id"))
            and message["request_type"] in ("hello", "arm", "drive", "stop", "estop", "status", "invalid")
            and (message["seq"] is None or type(message["seq"]) is int
                 and 0 < message["seq"] <= _MAX_SEQUENCE)
            and message["result"] in ("challenge", "accepted", "rejected", "status")
            and isinstance(message["reason"], str) and 0 < len(message["reason"]) <= 128
            and message["state"] in ("disarmed", "armed", "estop")
            and type(message["lease_remaining_ms"]) is int and 0 <= message["lease_remaining_ms"] <= 300
            and _number(message["v_mm_s"]) and abs(message["v_mm_s"]) <= 180
            and _number(message["omega_rad_s"]) and abs(message["omega_rad_s"]) <= 1.5
            and message["synthetic"] is True and message["execution"] == "unknown"
            and all(message[key] is False for key in ("device_io", "hardware_ready", "motion_permitted")))


class WireCommandSender:
    """One robot's independent sender; polling does not retry or renew a lease.

    ``session_id`` is pinned to the existing controller process. Keep this
    sender for the entire host session, including explicit reconnects, so old
    controller sequence numbers cannot be wrapped in fresh wire envelopes.
    Call ``poll`` independently when no response arrives. A host-side fault
    cannot physically stop a disconnected device; receiver lease expiry is
    the separate synthetic stop mechanism.
    """

    def __init__(self, robot_id, *, session_id, robot_ids=tuple(DEFAULT_ROLES),
                 ack_timeout_s=.3):
        if not isinstance(robot_ids, (tuple, list)):
            raise ValueError("robot_ids must be an explicit complete fleet")  # noqa: TRY004 -- uniform API rejection
        if (not robot_ids or any(not robot_id_valid(rid) for rid in robot_ids)
                or len(set(robot_ids)) != len(robot_ids) or robot_id not in robot_ids):
            raise ValueError("A registered unique robot ID and complete fleet are required")
        if not _token(session_id):
            raise ValueError("session_id must be an ASCII protocol token")
        if not _number(ack_timeout_s) or not 0 < ack_timeout_s <= .3:
            raise ValueError("ACK timeout must be in (0, 0.3] host seconds")
        self.robot_id, self.session_id = robot_id, session_id
        self.robot_ids = tuple(robot_ids)
        self.ack_timeout_s = ack_timeout_s
        self._limits = ControlLimits()
        self._now = -math.inf
        self._sequence = self._source_sequence = 0
        self._source_issued_at = -math.inf
        self._boot_id = self._link_id = self._hello_id = self._challenge = self._permit = None
        self._authorization_at = None
        self._pending = None
        self._state, self._reason = "disconnected", "explicit_hello_required"
        self._faulted = False
        self._last_response = None

    def _fail(self, reason):
        self._pending = self._challenge = self._permit = None
        self._authorization_at = None
        self._state, self._reason, self._faulted = "fault", reason, True

    def _invalid(self, reason):
        self._fail(reason)
        raise ValueError(reason)

    def _time(self, now_s):
        if not _number(now_s) or now_s < 0 or now_s < self._now:
            self._invalid("host_clock_must_be_finite_nonnegative_and_monotonic")
        self._now = now_s
        if self._pending is not None:
            if now_s - self._pending["sent_at_s"] >= self.ack_timeout_s - 1e-12:
                self._fail("ack_timeout_requires_explicit_hello")
        elif (self._authorization_at is not None
              and now_s - self._authorization_at >= .3 - 1e-12):
            self._fail("authorization_expired_requires_explicit_hello")

    def _base(self, message_type, *, context=True):
        message = {"protocol": "robo-wire", "version": 1, "type": message_type,
                   "robot_id": self.robot_id}
        if context:
            if self._boot_id is None or self._link_id is None:
                self._invalid("receiver_context_missing_requires_explicit_hello")
            if self._sequence >= _MAX_SEQUENCE:
                self._invalid("wire_sequence_exhausted")
            self._sequence += 1
            message.update(host_session_id=self.session_id, boot_id=self._boot_id,
                           link_id=self._link_id, seq=self._sequence)
        return message

    def _send(self, message, now_s):
        self._pending = {"request": copy.deepcopy(message), "sent_at_s": now_s}
        self._state, self._reason = "awaiting_" + message["type"], "awaiting_ack"
        return message

    def _idle(self):
        if self._pending is not None:
            self._invalid("one_request_may_be_in_flight")
        if self._faulted:
            self._invalid("explicit_hello_required_after_fault")

    def hello(self, now_s):
        """Explicitly start a new receiver link; never resumes old movement."""
        self._time(now_s)
        self._pending = self._challenge = self._permit = None
        self._boot_id = self._link_id = self._authorization_at = None
        self._faulted = False
        message = self._base("hello", context=False)
        self._hello_id = uuid.uuid4().hex
        message.update(host_session_id=self.session_id, hello_id=self._hello_id)
        return self._send(message, now_s)

    def arm(self, now_s):
        self._time(now_s)
        self._idle()
        if self._challenge is None or self._state != "disarmed":
            self._invalid("fresh_hello_challenge_required")
        message = self._base("arm")
        message["challenge_id"] = self._challenge
        self._challenge = self._permit = self._authorization_at = None
        return self._send(message, now_s)

    def _controller_packet(self, packet, now_s):
        if not isinstance(packet, dict):
            self._invalid("complete_controller_packet_required")
        if (type(packet.get("schema_version")) is not int or packet["schema_version"] != 1
                or packet.get("session_id") != self.session_id
                or packet.get("drive_model") != "differential_body"):
            self._invalid("wrong_controller_schema_session_or_drive_model")
        seq = packet.get("sequence")
        if type(seq) is not int or not self._source_sequence < seq <= _MAX_SEQUENCE:
            self._invalid("controller_sequence_must_strictly_increase")
        # Consume the source envelope even if a later robot row is malformed.
        self._source_sequence = seq
        issued, ttl = packet.get("issued_at_s"), packet.get("ttl_s")
        if (not _number(issued) or not 0 <= issued <= now_s
                or issued < self._source_issued_at):
            self._invalid("invalid_controller_timestamp")
        self._source_issued_at = issued
        if (not _number(ttl) or not .001 <= ttl <= .3
                or now_s - issued >= ttl - 1e-12):
            self._invalid("expired_or_invalid_controller_ttl")
        if any(packet.get(flag) is not False
               for flag in ("device_io", "hardware_ready", "motion_permitted")):
            self._invalid("hardware_authorization_is_forbidden")
        if (any(type(packet.get(flag)) is not bool for flag in
                ("emergency_stop", "mock_motion_permitted", "localization_session_closed"))
                or packet.get("status") not in ("hold", "at_goal", "tracking_goal")
                or not (packet.get("stop_reason") is None
                        or isinstance(packet.get("stop_reason"), str)
                        and 0 < len(packet["stop_reason"]) <= 128)
                or not isinstance(packet.get("conflicts"), list)):
            self._invalid("invalid_controller_status")
        commands = packet.get("robots")
        if not isinstance(commands, list) or len(commands) != len(self.robot_ids):
            self._invalid("incomplete_controller_fleet")
        checked = {}
        for command in commands:
            if not isinstance(command, dict):
                self._invalid("invalid_controller_robot")
            rid = command.get("robot_id")
            if not isinstance(rid, str) or rid not in self.robot_ids or rid in checked:
                self._invalid("unknown_or_duplicate_controller_robot")
            velocity, forward, omega, heading = (command.get(key) for key in
                ("velocity_world_mm_s", "forward_velocity_mm_s", "angular_velocity_rad_s",
                 "pose_heading_rad"))
            if (not isinstance(velocity, (tuple, list)) or len(velocity) != 2
                    or command.get("wheel_velocity_rad_s") != []
                    or type(command.get("at_goal")) is not bool
                    or not all(_number(v) for v in (*velocity, forward, omega, heading))):
                self._invalid("invalid_controller_velocity")
            if (abs(forward) > self._limits.max_speed_mm_s
                    or math.hypot(*velocity) > self._limits.max_speed_mm_s + 1e-8
                    or abs(omega) > self._limits.max_turn_rad_s
                    or abs(heading) > 1e6
                    or abs(velocity[0] - forward * math.cos(heading)) > 1e-8
                    or abs(velocity[1] - forward * math.sin(heading)) > 1e-8):
                self._invalid("controller_speed_or_differential_kinematics_mismatch")
            if command["at_goal"] and any(v != 0 for v in (*velocity, forward, omega)):
                self._invalid("at_goal_command_must_be_zero")
            checked[rid] = (forward, omega, command["at_goal"])
        hold = (packet["status"] != "tracking_goal" or not packet["mock_motion_permitted"]
                or packet["stop_reason"] is not None or packet["localization_session_closed"]
                or packet["emergency_stop"] or bool(packet["conflicts"]))
        if hold and any(v != 0 for row in checked.values() for v in row[:2]):
            self._invalid("controller_hold_must_zero_the_complete_fleet")
        return checked[self.robot_id], issued, math.floor(ttl * 1000), hold

    def drive_from_packet(self, packet, now_s, *, keep_armed_zero=False):
        """Validate the entire controller output atomically, then address one robot.

        Runtime may opt into fresh zero setpoints while other robots move or
        a manipulation waits. This never overrides a genuine hold or fault.
        """
        self._time(now_s)
        if type(keep_armed_zero) is not bool:
            self._invalid("keep_armed_zero_must_be_boolean")
        command, issued, ttl_ms, hold = self._controller_packet(packet, now_s)
        benign_goal = (packet["status"] == "at_goal" and packet["mock_motion_permitted"]
                       and packet["stop_reason"] is None and not packet["emergency_stop"]
                       and not packet["localization_session_closed"] and not packet["conflicts"])
        if (hold and not (keep_armed_zero and benign_goal)) or (command[2] and not keep_armed_zero):
            return self.stop(now_s, reason=packet["stop_reason"] or "controller_hold",
                             emergency=packet["emergency_stop"])
        self._idle()
        if self._state != "armed" or self._permit is None or self._authorization_at is None:
            self._invalid("fresh_receiver_permit_required")
        if issued < self._authorization_at:
            self._invalid("controller_packet_predates_received_permit")
        message = self._base("drive")
        message.update(permit_id=self._permit, ttl_ms=ttl_ms,
                       v_mm_s=command[0], omega_rad_s=command[1])
        self._permit = self._authorization_at = None
        return self._send(message, now_s)

    def stop(self, now_s, *, reason="operator_stop", emergency=False):
        """Supersede an in-flight request; no movement permit is needed to stop."""
        self._time(now_s)
        if (not isinstance(reason, str) or not 0 < len(reason) <= 128
                or type(emergency) is not bool):
            self._invalid("invalid_stop_reason_or_emergency_flag")
        self._pending = self._challenge = self._permit = self._authorization_at = None
        message = self._base("estop" if emergency else "stop")
        message["reason"] = reason
        return self._send(message, now_s)

    def status(self, now_s):
        """Request diagnostics, without renewing the current movement permit."""
        self._time(now_s)
        self._idle()
        return self._send(self._base("status"), now_s)

    def accept_response(self, response, now_s):
        self._time(now_s)
        if not _response_valid(response):
            self._invalid("malformed_receiver_response")
        if self._pending is None:
            self._fail("unsolicited_or_late_response_requires_explicit_hello")
            return self.snapshot()
        request = self._pending["request"]
        expected = {"robot_id": self.robot_id, "host_session_id": self.session_id,
                    "request_type": request["type"], "seq": request.get("seq"),
                    "hello_id": self._hello_id}
        if request["type"] != "hello":
            expected.update(boot_id=self._boot_id, link_id=self._link_id)
        if any(response[key] != value for key, value in expected.items()):
            self._fail("foreign_duplicate_or_out_of_order_response")
            return self.snapshot()
        if response["result"] == "rejected":
            self._last_response = copy.deepcopy(response)
            self._fail("receiver_rejected:" + response["reason"])
            return self.snapshot()
        kind = request["type"]
        valid = False
        if kind == "hello":
            valid = (response["result"] == "challenge" and response["state"] == "disarmed"
                     and _token(response["boot_id"]) and _token(response["link_id"])
                     and _token(response["challenge_id"]) and response["permit_id"] is None
                     and response["v_mm_s"] == response["omega_rad_s"] == 0)
        elif kind in ("arm", "drive"):
            valid = (response["result"] == "accepted" and response["state"] == "armed"
                     and _token(response["permit_id"]) and response["challenge_id"] is None
                     and response["permit_id"] != request.get("permit_id")
                     and response["v_mm_s"] == request.get("v_mm_s", 0)
                     and response["omega_rad_s"] == request.get("omega_rad_s", 0))
        elif kind in ("stop", "estop"):
            valid = (response["result"] == "accepted"
                     and response["state"] in (("estop",) if kind == "estop" else ("disarmed", "estop"))
                     and response["permit_id"] is response["challenge_id"] is None
                     and response["v_mm_s"] == response["omega_rad_s"] == 0)
        elif kind == "status":
            valid = response["result"] == "status"
        if not valid:
            self._fail("receiver_ack_does_not_match_request_semantics")
            return self.snapshot()
        self._pending = None
        self._last_response = copy.deepcopy(response)
        self._state, self._reason = response["state"], "synthetic_ack_not_execution"
        if kind == "hello":
            self._boot_id, self._link_id = response["boot_id"], response["link_id"]
            self._challenge, self._authorization_at = response["challenge_id"], now_s
        elif kind in ("arm", "drive"):
            self._permit, self._authorization_at = response["permit_id"], now_s
        elif kind == "status":
            # Status cannot grant or replace a permit; a changed state invalidates it.
            if response["state"] != "armed":
                self._permit = self._authorization_at = None
            self._time(now_s)
        if self._faulted:
            self._state = "fault"
        return self.snapshot()

    def poll(self, now_s):
        self._time(now_s)
        return self.snapshot()

    @property
    def pending_body_setpoint(self):
        """Detached hazard data only; reading never sends, renews or polls.

        An ACK-waiting command may still be in transit or already applied.
        A supervisor's newer local zero output does not cancel that command.
        """
        request = self._pending["request"] if self._pending else None
        if request is None or request["type"] != "drive":
            return None
        return {"v_mm_s": request["v_mm_s"], "omega_rad_s": request["omega_rad_s"]}

    def snapshot(self):
        return {"robot_id": self.robot_id, "host_session_id": self.session_id,
                "boot_id": self._boot_id, "link_id": self._link_id,
                "state": self._state, "reason": self._reason,
                "wire_sequence": self._sequence, "controller_sequence": self._source_sequence,
                "pending_request": self._pending["request"]["type"] if self._pending else None,
                "permit_available": self._permit is not None and not self._faulted,
                "last_response": copy.deepcopy(self._last_response),
                "synthetic": True, "execution": "unknown", "device_io": False,
                "hardware_ready": False, "motion_permitted": False}
