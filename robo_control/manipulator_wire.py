"""Draft manipulation contract and host state machine; no physical transport.

Intentionally separate from robo-wire v1. Synthetic evidence is never promoted
to MissionExecutor feedback. Tokens prevent stale replay, not malicious peers.
"""
from __future__ import annotations

import math
import re
import uuid
from copy import deepcopy

from .fleet import robot_id_valid
from .wire_codec import decode_frame, encode_frame

PROTOCOL = "robo-manipulator-draft"
VERSION = 1
MAX_SEQUENCE = 2**53 - 1
MAX_OPERATIONS = 4096
ACK_TIMEOUT_S = .3
SENSOR_MAX_AGE_S = .2
PERMIT_LIFETIME_S = .3
PHASE_ACTIONS = {
    "close_servo": {"disc_latch_close", "gripper_close"},
    "confirm_grip": {"hold"}, "retract": {"arm_retract"},
    "confirm_load": {"hold"},
    "release_servo": {"disc_latch_open", "gripper_open", "hopper_gate_open_one"},
    "confirm_clear": {"hold"},
}
# Proposed logical signals, not a declaration that those sensors are installed.
# Placement/settling and object coordinates belong to reviewed visual evidence.
SIGNALS = frozenset({"servo_closed", "servo_open", "optical_present", "gripper_present",
                     "arm_retracted", "hopper_loaded", "optical_clear", "gripper_clear", "hopper_clear"})
IDENTIFIER = re.compile(r"[A-Za-z0-9_:-]{1,128}\Z")
OP_FIELDS = {"command_id", "phase", "action", "timeout_ms"}
REQUEST_FIELDS = {"protocol", "version", "type", "robot_id", "host_session_id",
                  "hello_id", "boot_id", "link_id", "seq", "permit_id", "operation"}
RESPONSE_FIELDS = REQUEST_FIELDS | {"request_type", "result", "reason", "state",
    "signals", "sample_sequence", "sample_age_ms", "synthetic", "device_io",
    "hardware_ready", "execution", "physical_success"}


def identifier(value):
    return type(value) is str and IDENTIFIER.fullmatch(value) is not None


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def sequence(value):
    return type(value) is int and 1 <= value <= MAX_SEQUENCE


def validate_operation(value):
    if (type(value) is not dict or set(value) != OP_FIELDS
            or not identifier(value["command_id"])
            or type(value["phase"]) is not str or value["phase"] not in PHASE_ACTIONS
            or type(value["action"]) is not str
            or value["action"] not in PHASE_ACTIONS[value["phase"]]
            or type(value["timeout_ms"]) is not int or not 1 <= value["timeout_ms"] <= 4000):
        raise ValueError("Invalid draft operation: explicit phase, action and 1..4000 ms timeout required")
    return deepcopy(value)


def operation_from_intent(intent, *, timeout_ms):
    """Copy a non-navigation MissionExecutor intent; never dispatch or advance it."""
    if (type(intent) is not dict or intent.get("device_io") is not False
            or intent.get("dispatch_enabled") is not False or intent.get("fault") is not None
            or not robot_id_valid(intent.get("robot_id")) or not identifier(intent.get("session_id"))):
        raise ValueError("A disabled, fault-free mission manipulation intent is required")
    return validate_operation({"command_id": intent.get("command_id"), "phase": intent.get("phase"),
                               "action": intent.get("servo_intent"), "timeout_ms": timeout_ms})


def validate_request(message):
    # Enforce framing size/types even for callers using the dictionary API.
    encode_frame(message)
    if (set(message) != REQUEST_FIELDS or message["protocol"] != PROTOCOL
            or type(message["version"]) is not int or message["version"] != VERSION
            or type(message["type"]) is not str
            or message["type"] not in {"hello", "execute", "sample", "cancel", "stop", "estop"}
            or not robot_id_valid(message["robot_id"])
            or any(not identifier(message[k]) for k in ("host_session_id", "hello_id"))
            or not sequence(message["seq"])):
        raise ValueError("Invalid manipulation request schema")
    validate_operation(message["operation"])
    for key in ("boot_id", "link_id", "permit_id"):
        if message["type"] == "hello":
            if message[key] is not None:
                raise ValueError("Hello must not reuse a device context")
        elif key == "permit_id" and message["type"] != "execute":
            if message[key] is not None:
                raise ValueError("Only execute consumes a permit")
        elif not identifier(message[key]):
            raise ValueError("Missing device context")


class ManipulatorSender:
    """One outstanding request/operation per robot; no retry or automatic recovery.

    Lifetime attempt IDs are retained across begin() calls, including timeouts.
    A new instance/boot cannot provide durable exactly-once execution: operator
    reconciliation of unknown physical outcomes is required before restarting.
    """

    def __init__(self, robot_id, *, session_id):
        if not robot_id_valid(robot_id) or not identifier(session_id):
            raise ValueError("Explicit robot and host session required")
        self.robot_id, self.session_id = robot_id, session_id
        self.state, self.reason = "idle", "not_started"
        self._now = None
        self._clock_broken = False
        self._seq = 0
        self._attempted = set()
        self._operation = self._pending = self._context = self._evidence = None
        self._deadline = self._permit_received = None
        self._last_sample = 0

    def _halt(self, reason):
        self.state, self.reason = "closed", reason
        self._pending = self._evidence = None

    def poll(self, now_s):
        if (self._clock_broken or not finite(now_s) or now_s < 0
                or self._now is not None and now_s < self._now):
            self._clock_broken = True
            self._halt("invalid_host_clock")
            raise ValueError("Host clock must be monotonic; reconstruct after clock fault")
        self._now = now_s
        if self.state != "closed":
            if self._pending and now_s >= self._pending[1] + ACK_TIMEOUT_S - 1e-12:
                self._halt("ack_timeout_outcome_unknown")
            elif self._deadline is not None and now_s >= self._deadline - 1e-12:
                self._halt("operation_timeout_outcome_unknown")
            elif (self.state == "ready" and now_s >= self._permit_received + PERMIT_LIFETIME_S - 1e-12):
                self._halt("permit_expired")
        if self._evidence and now_s >= self._evidence["valid_until_host_s"] - 1e-12:
            self._evidence = None
        return self.snapshot()

    def _request(self, kind):
        if self._seq >= MAX_SEQUENCE:
            self._halt("sequence_exhausted")
            raise ValueError("Sequence exhausted")
        self._seq += 1
        context = self._context or {}
        message = {"protocol": PROTOCOL, "version": VERSION, "type": kind,
            "robot_id": self.robot_id, "host_session_id": self.session_id,
            "hello_id": self._hello_id, "boot_id": context.get("boot_id"),
            "link_id": context.get("link_id"), "seq": self._seq,
            "permit_id": context.get("permit_id") if kind == "execute" else None,
            "operation": deepcopy(self._operation)}
        validate_request(message)
        self._pending = (deepcopy(message), self._now)
        return message

    def begin(self, operation, now_s):
        self.poll(now_s)
        if self.state not in {"idle", "closed"}:
            raise ValueError("Cancel/stop the previous operation explicitly first")
        operation = validate_operation(operation)
        if operation["command_id"] in self._attempted or len(self._attempted) >= MAX_OPERATIONS:
            raise ValueError("Operation already attempted or bounded history exhausted; reconcile, do not retry")
        self._attempted.add(operation["command_id"])
        self._operation, self._context, self._evidence = operation, None, None
        self._hello_id = uuid.uuid4().hex
        self._last_sample = 0
        self._deadline = now_s + operation["timeout_ms"] / 1000
        self.state, self.reason = "hello_pending", "awaiting_challenge"
        return self._request("hello")

    def execute(self, now_s):
        self.poll(now_s)
        if self.state != "ready" or self._pending:
            raise ValueError("A fresh matching challenge is required")
        self.state = "execute_pending"
        return self._request("execute")

    def sample(self, now_s):
        self.poll(now_s)
        if self.state != "active" or self._pending:
            raise ValueError("An active acknowledged operation and no pending request required")
        return self._request("sample")

    def stop(self, now_s, *, kind="stop"):
        self.poll(now_s)
        if kind not in {"cancel", "stop", "estop"}:
            raise ValueError("Use cancel, stop or estop")
        self._halt("local_" + kind + "_outcome_unknown")
        # A lost hello ACK cannot have authorized execute. The remote offer expires.
        if self._context is None:
            return None
        return self._request(kind)

    def accept_response(self, response, now_s):
        self.poll(now_s)
        try:
            encode_frame(response)
            if not self._pending:
                raise ValueError("unsolicited_or_late_response")
            request, sent_at = self._pending
            # Stop ACKs are also deadline-bound even though local state is closed.
            if now_s >= sent_at + ACK_TIMEOUT_S - 1e-12:
                raise ValueError("late_response")
            if (set(response) != RESPONSE_FIELDS or response["type"] != "response"
                    or response["request_type"] != request["type"]
                    or type(response["version"]) is not int
                    or type(response["seq"]) is not int
                    or any(response[k] != request[k] for k in
                           ("protocol", "version", "robot_id", "host_session_id", "hello_id", "seq", "operation"))
                    or not identifier(response["boot_id"]) or not identifier(response["link_id"])
                    or response["synthetic"] is not True or response["device_io"] is not False
                    or response["hardware_ready"] is not False or response["execution"] != "unknown"
                    or response["physical_success"] is not None):
                raise ValueError("wrong_response_identity_or_schema")
            validate_operation(response["operation"])
            kind = request["type"]
            if kind != "hello" and any(response[k] != request[k] for k in ("boot_id", "link_id")):
                raise ValueError("wrong_device_context")
            expected = "offered" if kind == "hello" else "active" if kind in {"execute", "sample"} else "closed"
            if (response["result"] != "accepted" or response["state"] != expected
                    or response["reason"] != "ok"):
                raise ValueError("device_rejected_or_closed")
            if kind == "hello":
                if not identifier(response["permit_id"]):
                    raise ValueError("missing_permit")
            elif response["permit_id"] is not None:
                raise ValueError("unexpected_permit")
            if kind != "sample":
                if any(response[k] is not None for k in ("signals", "sample_sequence", "sample_age_ms")):
                    raise ValueError("ack_is_not_sensor_evidence")
            else:
                signals, age, sample_seq = (response[k] for k in ("signals", "sample_age_ms", "sample_sequence"))
                if signals is None:
                    if age is not None or sample_seq is not None:
                        raise ValueError("invalid_unknown_sample")
                    self._evidence = None
                else:
                    if (type(signals) is not dict or set(signals) != SIGNALS
                            or any(v is not None and type(v) is not bool for v in signals.values())
                            or type(age) is not int or age < 0 or not sequence(sample_seq)
                            or sample_seq <= self._last_sample):
                        raise ValueError("invalid_or_duplicate_sensor_sample")
                    conservative_age = now_s - sent_at + age / 1000
                    if conservative_age >= SENSOR_MAX_AGE_S - 1e-12:
                        raise ValueError("stale_sensor_sample")
                    self._last_sample = sample_seq
                    self._evidence = {"robot_id": self.robot_id, "session_id": self.session_id,
                        "command_id": self._operation["command_id"], "signals": deepcopy(signals),
                        "sample_sequence": sample_seq, "age_upper_bound_s": conservative_age,
                        "valid_until_host_s": sent_at + SENSOR_MAX_AGE_S - age / 1000,
                        "synthetic": True, "mission_feedback_eligible": False,
                        "physical_success": None}
            self._pending = None
            if kind == "hello":
                self._context = {k: response[k] for k in ("boot_id", "link_id", "permit_id")}
                self._permit_received, self.state = now_s, "ready"
            elif kind == "execute":
                self.state = "active"
            self.reason = "accepted_" + kind
            return True
        except (ValueError, TypeError, KeyError):
            self._halt("invalid_response_outcome_unknown")
            return False

    def accept_frame(self, frame, now_s):
        try:
            response = decode_frame(frame)
        except ValueError:
            self.poll(now_s)
            self._halt("malformed_response")
            return False
        return self.accept_response(response, now_s)

    def snapshot(self):
        """Historical diagnostics only: poll with host time before using evidence."""
        return {"robot_id": self.robot_id, "state": self.state, "reason": self.reason,
                "operation": deepcopy(self._operation), "evidence": deepcopy(self._evidence),
                "synthetic": True, "device_io": False, "hardware_ready": False,
                "physical_success": None, "mission_feedback_eligible": False}
