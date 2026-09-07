"""Nonblocking, bounded, in-memory wire supervision for the complete fleet.

The receiver watchdog is a SAME-PROCESS POLLED MODEL, not an independent
physical watchdog. No socket, serial port, process, motor or sensor is opened.
Receiver snapshots report modeled setpoints; physical execution stays unknown.
"""
from __future__ import annotations

import heapq
import math

from .fake_receiver import FakeRobotReceiver
from .fleet import robot_id_valid
from .wire_codec import MAX_FRAME_BYTES, FrameDecoder, WireError, encode_frame
from .wire_sender import WireCommandSender

MAX_QUEUED_EVENTS = 256
MAX_EVENTS_PER_POLL = 512
_MAX_COUNTER = 2**53 - 1
_OPTIONS = {"request_delay_s", "response_delay_s", "drop_requests", "drop_responses",
            "duplicate_responses", "connected"}


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _options(changes):
    if not isinstance(changes, dict) or not set(changes) <= _OPTIONS:
        raise ValueError("Unknown in-memory link option")
    result = dict(changes)
    for name, value in result.items():
        if name == "connected":
            if type(value) is not bool:
                raise ValueError("connected must be boolean")
        elif name.endswith("_delay_s"):
            if not _number(value) or not 0 <= value <= 5:
                raise ValueError("Synthetic delays must be between zero and five seconds")
        elif type(value) is not int or not 0 <= value <= 10000:
            raise ValueError("Synthetic drop/duplicate counts must be in [0, 10000]")
    return result


class RuntimeWireSupervisor:
    """One sticky fleet inhibit, with explicit start/reconnect and no retries.

    ``link_options`` contains per-robot test options. Drop counts consume the
    next N request/response frames; duplicate counts duplicate the next N ACKs.
    Delays affect newly queued bytes. A disconnected link drops communication
    but does NOT pretend the receiver has already stopped.
    """

    def __init__(self, robot_ids, *, session_id, limits=None, link_options=None):
        if (not isinstance(robot_ids, (tuple, list)) or not 1 <= len(robot_ids) <= 32
                or any(not robot_id_valid(rid) for rid in robot_ids)
                or len(set(robot_ids)) != len(robot_ids)):
            raise ValueError("An explicit unique fleet of 1 to 32 robot IDs is required")
        self.robot_ids = tuple(robot_ids)
        self.session_id = session_id
        if link_options is None:
            link_options = {}
        if not isinstance(link_options, dict) or not set(link_options) <= set(self.robot_ids):
            raise ValueError("Link options must address this fleet only")
        self._links = {}
        self._senders = {}
        self._receivers = {}
        self._request_decoders = {}
        self._response_decoders = {}
        self._origins = {}
        for index, rid in enumerate(self.robot_ids):
            self._links[rid] = {"request_delay_s": 0., "response_delay_s": 0.,
                                "drop_requests": 0, "drop_responses": 0,
                                "duplicate_responses": 0, "connected": True,
                                **_options(link_options.get(rid, {}))}
            self._senders[rid] = WireCommandSender(rid, session_id=session_id, robot_ids=self.robot_ids)
            self._receivers[rid] = FakeRobotReceiver(rid, limits=limits)
            self._request_decoders[rid] = FrameDecoder()
            self._response_decoders[rid] = FrameDecoder()
            self._origins[rid] = 1000. + index * 137.
        self._queue = []
        self._serial = self._generation = 0
        self._now = self._host_origin = None
        self._clock_broken = False
        self._started = self._closed = self._connecting = False
        self._fault = None
        self._stop_requested = dict.fromkeys(self.robot_ids, False)
        self._stop_acknowledged = dict.fromkeys(self.robot_ids, False)
        self._stop_sequences = dict.fromkeys(self.robot_ids)
        self._reconnect_receiver_reasons = {}
        self._counters = dict.fromkeys(("events_processed", "requests_delivered", "responses_delivered",
            "requests_dropped", "responses_dropped", "responses_duplicated", "retired_events",
            "ignored_late_responses", "submitted_packets", "faults", "reconnects"), 0)

    @property
    def ready(self):
        return (self._started and not self._closed and self._fault is None
                and not self._queue and all(
                    self._links[rid]["connected"]
                    and self._senders[rid].snapshot()["state"] == "armed"
                    and self._senders[rid].snapshot()["pending_request"] is None
                    and self._senders[rid].snapshot()["permit_available"]
                    and self._receivers[rid].snapshot()["state"] == "armed"
                    for rid in self.robot_ids))

    @property
    def fault(self):
        return self._fault

    @property
    def started(self):
        return self._started

    @property
    def closed(self):
        return self._closed

    def _count(self, key, amount=1):
        self._counters[key] = min(_MAX_COUNTER, self._counters[key] + amount)

    def _clock(self, now_s):
        if (self._clock_broken or not _number(now_s) or now_s < 0
                or (self._now is not None and now_s < self._now)):
            self._clock_broken = True
            self._trip("invalid_supervisor_clock", self._now if self._now is not None else 0.)
            raise ValueError("Supervisor clock must be finite, nonnegative and monotonic")
        self._now = float(now_s)
        if self._host_origin is None:
            self._host_origin = self._now

    def _receiver_time(self, rid, now_s):
        return self._origins[rid] + (now_s - self._host_origin)

    def _retire_events(self):
        self._count("retired_events", len(self._queue))
        self._queue.clear()
        self._generation += 1

    def _enqueue(self, rid, direction, frame, now_s, *, delay_s=None):
        if type(frame) is not bytes or len(frame) > MAX_FRAME_BYTES:
            raise WireError("Synthetic transport accepts at most one bounded bytes frame")
        if len(self._queue) >= MAX_QUEUED_EVENTS or self._serial >= _MAX_COUNTER:
            raise WireError("Synthetic wire event capacity exhausted")
        if delay_s is None:
            delay_s = self._links[rid][direction + "_delay_s"]
        self._serial += 1
        heapq.heappush(self._queue, (now_s + delay_s, self._serial, self._generation,
                                   rid, direction, frame))

    def _request_stops(self, now_s, reason, *, emergency=False):
        if all(self._stop_requested.values()):
            return
        self._retire_events()
        self._connecting = False
        # Prepare all stops before delivering any. Missing context is explicitly
        # unconfirmed, not a fabricated ACK or zero receiver setpoint.
        prepared = []
        for rid, sender in self._senders.items():
            self._stop_requested[rid] = True
            self._stop_acknowledged[rid] = False
            try:
                message = sender.stop(now_s, reason=reason[:128], emergency=emergency)
                self._stop_sequences[rid] = message["seq"]
                prepared.append((rid, encode_frame(message)))
            except (ValueError, WireError):
                self._stop_sequences[rid] = None
        for rid, frame in prepared:
            try:
                self._enqueue(rid, "request", frame, now_s)
            except WireError:
                self._count("requests_dropped")

    def _trip(self, reason, now_s, *, emergency=False):
        if self._fault is None:
            self._fault = str(reason)[:256]
            self._count("faults")
            self._request_stops(now_s, self._fault, emergency=emergency)

    def _tick_endpoints(self, now_s):
        for rid in self.robot_ids:
            try:
                receiver = self._receivers[rid].tick(self._receiver_time(rid, now_s))
                sender = self._senders[rid].poll(now_s)
            except ValueError:
                self._trip("endpoint_clock_fault:" + rid, now_s)
                continue
            if self._started and self._fault is None and not self._closed:
                if not self._links[rid]["connected"]:
                    self._trip("link_disconnected:" + rid, now_s)
                elif (receiver["reason"] in ("command_watchdog", "handshake_expired", "invalid_receiver_clock")
                      and not (self._connecting and sender["pending_request"] == "hello"
                               and receiver["reason"] == self._reconnect_receiver_reasons.get(rid))):
                    self._trip("receiver_" + receiver["reason"] + ":" + rid, now_s)
                elif sender["state"] == "fault":
                    self._trip("sender_" + sender["reason"] + ":" + rid, now_s)

    def _deliver_request(self, rid, frame, now_s):
        messages = self._request_decoders[rid].feed(frame)
        for message in messages:
            if (self._fault is not None or self._closed) and message.get("type") not in ("stop", "estop"):
                self._count("requests_dropped")
                continue
            response = self._receivers[rid].receive(message, self._receiver_time(rid, now_s))
            self._count("requests_delivered")
            encoded = encode_frame(response)
            self._enqueue(rid, "response", encoded, now_s)
            if self._links[rid]["duplicate_responses"]:
                self._links[rid]["duplicate_responses"] -= 1
                self._count("responses_duplicated")
                self._enqueue(rid, "response", encoded, now_s)

    def _deliver_response(self, rid, frame, now_s):
        for response in self._response_decoders[rid].feed(frame):
            self._count("responses_delivered")
            if ((self._fault is not None or self._closed)
                    and response.get("request_type") not in ("stop", "estop")):
                self._count("ignored_late_responses")
                continue
            sender = self._senders[rid]
            state = sender.accept_response(response, now_s)
            if (self._stop_requested[rid] and response.get("request_type") in ("stop", "estop")
                    and response.get("seq") == self._stop_sequences[rid]
                    and response.get("result") == "accepted"
                    and state["last_response"] == response):
                self._stop_acknowledged[rid] = True
            if self._fault is not None or self._closed:
                continue
            if state["state"] == "fault":
                self._trip("receiver_response:" + rid + ":" + state["reason"], now_s)
            elif (self._connecting and response["request_type"] == "hello"
                    and state["state"] == "disarmed"):
                self._enqueue(rid, "request", encode_frame(sender.arm(now_s)), now_s)

    def poll(self, now_s):
        """Apply elapsed time even without camera frames; never replay a packet."""
        self._clock(now_s)
        self._tick_endpoints(now_s)
        processed = 0
        while self._queue and self._queue[0][0] <= now_s:
            if processed >= MAX_EVENTS_PER_POLL:
                self._trip("wire_drain_capacity_exhausted", now_s)
                break
            _, _, generation, rid, direction, frame = heapq.heappop(self._queue)
            processed += 1
            self._count("events_processed")
            if generation != self._generation:
                self._count("retired_events")
                continue
            link = self._links[rid]
            drop_key = "drop_requests" if direction == "request" else "drop_responses"
            counter = "requests_dropped" if direction == "request" else "responses_dropped"
            if not link["connected"] or link[drop_key]:
                if link[drop_key]:
                    link[drop_key] -= 1
                self._count(counter)
                continue
            try:
                if direction == "request":
                    self._deliver_request(rid, frame, now_s)
                else:
                    self._deliver_response(rid, frame, now_s)
            except (ValueError, WireError):
                self._trip("malformed_or_rejected_" + direction + ":" + rid, now_s)
        if self.ready:
            self._connecting = False
        return self.snapshot()

    def _begin(self, now_s):
        self._connecting = True
        prepared = [(rid, encode_frame(sender.hello(now_s))) for rid, sender in self._senders.items()]
        for rid, frame in prepared:
            self._enqueue(rid, "request", frame, now_s)
        return self.poll(now_s)

    def start(self, now_s):
        self._clock(now_s)
        if self._closed or self._fault is not None or self._started:
            raise ValueError("Start is once only; recovery requires explicit reconnect")
        self._started = True
        return self._begin(now_s)

    def submit(self, packet, now_s):
        self.poll(now_s)
        if self._fault is not None or self._closed:
            return self.snapshot()
        if not self.ready:
            self._trip("submit_requires_complete_ready_fleet", now_s)
            return self.poll(now_s)
        prepared = []
        try:
            for rid, sender in self._senders.items():
                message = sender.drive_from_packet(packet, now_s, keep_armed_zero=True)
                prepared.append((rid, message, encode_frame(message)))
            if any(message["type"] in ("stop", "estop") for _, message, _ in prepared):
                self._trip("controller_stop_requires_explicit_reconnect", now_s,
                           emergency=any(message["type"] == "estop" for _, message, _ in prepared))
            else:
                for rid, _, frame in prepared:
                    self._enqueue(rid, "request", frame, now_s)
                self._count("submitted_packets")
        except (ValueError, WireError):
            self._trip("invalid_controller_packet", now_s)
        return self.poll(now_s)

    def stop(self, now_s, reason="operator_stop"):
        self._clock(now_s)
        if not isinstance(reason, str) or not reason:
            raise ValueError("A stop reason is required")
        self._trip(reason, now_s)
        return self.poll(now_s)

    def reconnect(self, now_s, *, operator_confirmed):
        self._clock(now_s)
        if operator_confirmed is not True:
            raise ValueError("Explicit operator confirmation is required to reconnect")
        if self._closed or not self._started:
            raise ValueError("A closed or never-started supervisor cannot reconnect")
        if not all(link["connected"] for link in self._links.values()):
            raise ValueError("Reconnect requires every modeled transport link to be connected")
        self._retire_events()
        self._fault = None
        self._stop_requested = dict.fromkeys(self.robot_ids, False)
        self._stop_acknowledged = dict.fromkeys(self.robot_ids, False)
        self._stop_sequences = dict.fromkeys(self.robot_ids)
        self._reconnect_receiver_reasons = {
            rid: receiver.snapshot()["reason"] for rid, receiver in self._receivers.items()}
        for rid in self.robot_ids:
            self._request_decoders[rid].reset()
            self._response_decoders[rid].reset()
        self._count("reconnects")
        return self._begin(now_s)

    def close(self, now_s, reason="runtime_closed"):
        if not isinstance(reason, str) or not reason:
            raise ValueError("A close reason is required")
        if self._clock_broken:
            # A latched clock failure must not throw again during the runtime's
            # worker/report cleanup. Time is no longer trustworthy: retain the
            # last modeled receiver state and queue best-effort stops only.
            # Do not drain bytes, invent expiry, fabricate zero or claim ACKs.
            self._closed = True
            trusted = self._now if self._now is not None else 0.
            self._request_stops(trusted, reason)
            return self.snapshot()
        self._clock(now_s)
        self._closed = True
        self._request_stops(now_s, reason)
        return self.poll(now_s)

    def configure_link(self, robot_id, now_s, **changes):
        """Fault-injection control for tests only, never a hardware connector."""
        if robot_id not in self._links:
            raise ValueError("Unknown robot link")
        validated = _options(changes)
        self._clock(now_s)
        self._links[robot_id].update(validated)
        return self.poll(now_s)

    def inject_response(self, robot_id, frame, now_s, *, delay_s=0.):
        """Queue bounded test bytes, including intentionally bad/old ACKs."""
        if robot_id not in self._links:
            raise ValueError("Unknown robot link")
        _options({"response_delay_s": delay_s})
        self._clock(now_s)
        try:
            self._enqueue(robot_id, "response", frame, now_s, delay_s=delay_s)
        except WireError:
            self._trip("injected_response_capacity_or_size", now_s)
        return self.poll(now_s)

    def snapshot(self):
        ready = self.ready
        state = ("closed" if self._closed else "fault" if self._fault is not None
                 else "idle" if not self._started else "ready" if ready
                 else "connecting" if self._connecting else "pending")
        return {"mode": "fake_wire", "state": state, "session_id": self.session_id,
            "fault": self._fault, "started": self._started, "closed": self._closed, "ready": ready,
            "host_at_s": self._now, "pending_event_count": len(self._queue),
            "counters": dict(self._counters),
            "unconfirmed_stop_robot_ids": [rid for rid in self.robot_ids
                if self._stop_requested[rid] and not self._stop_acknowledged[rid]],
            "robots": [{"robot_id": rid, "connected": self._links[rid]["connected"],
                "sender": self._senders[rid].snapshot(), "receiver": self._receivers[rid].snapshot(),
                "stop_requested": self._stop_requested[rid], "stop_acknowledged": self._stop_acknowledged[rid]}
                for rid in self.robot_ids],
            "receiver_watchdog_model": "same_process_polled", "synthetic": True,
            "physical_execution": "unknown", "execution": "unknown", "device_io": False,
            "hardware_ready": False, "motion_permitted": False}
