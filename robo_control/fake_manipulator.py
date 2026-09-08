"""Synthetic manipulation endpoint, with explicit test-only sensor injection.

No motor/servo implementation and no physical safe-state assumption. A harness
must tick the receiver independently, including while host ACKs are missing.
"""
from __future__ import annotations

import math
import uuid
from copy import deepcopy

from .fleet import robot_id_valid
from .manipulator_wire import (
    MAX_OPERATIONS,
    MAX_SEQUENCE,
    PERMIT_LIFETIME_S,
    PHASE_ACTIONS,
    SIGNALS,
    finite,
    validate_request,
)
from .wire_codec import decode_frame, encode_frame


class FakeManipulatorReceiver:
    def __init__(self, robot_id, *, supported_actions):
        allowed = set().union(*PHASE_ACTIONS.values())
        if (not robot_id_valid(robot_id) or type(supported_actions) not in (set, frozenset)
                or not supported_actions or not supported_actions <= allowed):
            raise ValueError("Explicit robot ID and logical test capabilities required")
        self.robot_id, self.supported_actions = robot_id, frozenset(supported_actions)
        self.boot_id = uuid.uuid4().hex
        self.state, self.reason = "closed", "boot_requires_hello"
        self._now = None
        self._clock_broken = self._estop = False
        self._context = self._operation = self._permit = self._sample = None
        self._permit_at = self._deadline = self._executed_at = None
        self._last_seq = self._sample_seq = self.accepted_operations = 0
        self._consumed = set()

    def _halt(self, reason):
        self.state, self.reason = "closed", reason
        self._permit = self._sample = self._deadline = None

    def tick(self, now_s):
        if (self._clock_broken or not finite(now_s) or now_s < 0
                or self._now is not None and now_s < self._now):
            self._clock_broken = True
            self._halt("invalid_receiver_clock")
            raise ValueError("Receiver clock must be monotonic; reboot after clock fault")
        self._now = now_s
        if self.state == "offered" and now_s >= self._permit_at + PERMIT_LIFETIME_S - 1e-12:
            self._halt("permit_expired")
        elif self.state == "active" and now_s >= self._deadline - 1e-12:
            self._halt("operation_watchdog")
        return self.snapshot()

    def disconnect(self, now_s):
        self.tick(now_s)
        self._halt("disconnected")

    def emergency_stop(self, now_s):
        self.tick(now_s)
        self._estop = True
        self._halt("emergency_stop")

    def reset_emergency_stop(self, now_s):
        """Local test action only, never exposed as a wire reset."""
        self.tick(now_s)
        self._estop = False
        self._halt("local_reset_requires_new_operation")

    def inject_test_sample(self, command_id, signals, now_s):
        """Explicit fixture event AFTER execute; never synthesize success from ACK.

        Missing signals are unknown, not false/true. No object identity, pose,
        released/settled result, or real-feedback output is supported here.
        """
        self.tick(now_s)
        if (self.state != "active" or command_id != self._operation["command_id"]
                or now_s <= self._executed_at or type(signals) is not dict or not signals
                or not signals.keys() <= SIGNALS
                or any(v is not None and type(v) is not bool for v in signals.values())
                or self._sample_seq >= MAX_SEQUENCE):
            self._halt("invalid_test_sample")
            raise ValueError("A fresh, operation-scoped explicit synthetic sample is required")
        self._sample_seq += 1
        self._sample = ({k: signals.get(k) for k in SIGNALS}, now_s, self._sample_seq)

    def _reply(self, request, *, reason="ok"):
        result = deepcopy(request)
        result.update(type="response", request_type=request["type"], robot_id=self.robot_id,
            boot_id=self.boot_id, link_id=self._context["link_id"] if self._context else None,
            permit_id=self._permit if request["type"] == "hello" and reason == "ok" else None,
            result="accepted" if reason == "ok" else "rejected", reason=reason, state=self.state,
            signals=None, sample_sequence=None, sample_age_ms=None,
            synthetic=True, device_io=False, hardware_ready=False, execution="unknown", physical_success=None)
        if reason == "ok" and request["type"] == "sample" and self._sample:
            signals, sampled_at, seq = self._sample
            result.update(signals=deepcopy(signals), sample_sequence=seq,
                          sample_age_ms=max(0, math.ceil((self._now - sampled_at) * 1000)))
        return result

    def _reject(self, request, reason, *, halt=True):
        if halt:
            self._halt(reason)
        return self._reply(request, reason=reason)

    def receive(self, request, now_s):
        self.tick(now_s)
        try:
            validate_request(request)
        except (ValueError, TypeError):
            self._halt("invalid_request_schema")
            raise
        if request["robot_id"] != self.robot_id:
            return self._reject(request, "wrong_robot", halt=False)
        kind, operation = request["type"], request["operation"]
        if kind == "hello":
            self._halt("new_offer")
            if self._estop:
                return self._reject(request, "emergency_stop")
            if operation["action"] not in self.supported_actions:
                return self._reject(request, "unsupported_action")
            key = (request["host_session_id"], operation["command_id"])
            if key in self._consumed or len(self._consumed) >= MAX_OPERATIONS:
                return self._reject(request, "already_executed_or_history_full")
            self._operation = deepcopy(operation)
            self._context = {k: request[k] for k in ("host_session_id", "hello_id")}
            self._context["link_id"] = uuid.uuid4().hex
            self._last_seq = request["seq"]
            self._permit, self._permit_at = uuid.uuid4().hex, now_s
            self.state, self.reason = "offered", "awaiting_execute"
            return self._reply(request)
        if (not self._context or request["boot_id"] != self.boot_id
                or any(request[k] != v for k, v in self._context.items())
                or operation != self._operation):
            return self._reject(request, "wrong_operation_or_context")
        # Current-context cancellation always wins, even if its seq is old.
        if kind in {"cancel", "stop", "estop"}:
            self._last_seq = max(self._last_seq, request["seq"])
            self._estop = self._estop or kind == "estop"
            self._halt("emergency_stop" if self._estop else kind)
            return self._reply(request)
        if request["seq"] <= self._last_seq:
            return self._reject(request, "duplicate_or_reordered_request")
        self._last_seq = request["seq"]
        if self._estop:
            return self._reject(request, "emergency_stop")
        if kind == "execute":
            if self.state != "offered" or request["permit_id"] != self._permit:
                return self._reject(request, "invalid_or_expired_permit")
            deadline = self._permit_at + operation["timeout_ms"] / 1000
            if now_s >= deadline - 1e-12:
                return self._reject(request, "operation_expired_before_execute")
            self._consumed.add((request["host_session_id"], operation["command_id"]))
            self.accepted_operations += 1
            self.state, self.reason = "active", "logical_action_accepted"
            self._permit, self._deadline, self._executed_at = None, deadline, now_s
            self._sample = None
            return self._reply(request)
        if self.state != "active":
            return self._reject(request, "operation_not_active")
        return self._reply(request)  # sample never renews a permit or watchdog.

    def receive_frame(self, frame, now_s):
        self.tick(now_s)
        try:
            request = decode_frame(frame)
        except ValueError:
            self._halt("malformed_frame")
            raise
        return encode_frame(self.receive(request, now_s))

    def snapshot(self):
        return {"robot_id": self.robot_id, "state": self.state, "reason": self.reason,
            "requested_action": self._operation["action"] if self.state == "active" else None,
            "accepted_operation_count": self.accepted_operations, "estop_latched": self._estop,
            "deadline_receiver_s": self._deadline, "synthetic": True, "device_io": False,
            "hardware_ready": False, "execution": "unknown", "physical_success": None,
            "physical_safe_state": "unconfigured"}
