"""Four-robot byte-loop fault lab with independent virtual clock origins.

This executable test bench opens no sockets, serial ports, cameras or devices.
It runs the existing differential controller on fixed synthetic observations;
it never integrates commands into a position or claims a physical result.
"""
from __future__ import annotations

import argparse
import json
import math

from .control_loop import ClosedLoopController
from .fake_receiver import FakeRobotReceiver
from .fleet import DEFAULT_ROLES
from .wire_codec import FrameDecoder, WireError, encode_frame
from .wire_sender import WireCommandSender


class WireLab:
    """Deterministic in-memory transport; clocks advance even without host sends."""

    def __init__(self, *, host_origin_s=1000., device_origins_s=(5., 4000., 20., 9000.)):
        origins = (host_origin_s, *device_origins_s)
        if (len(device_origins_s) != 4
                or any(type(t) not in (float, int) or not math.isfinite(t) or not 0 <= t <= 1e6 for t in origins)):
            raise ValueError("Finite nonnegative four-device clock origins required")
        self.ids = tuple(DEFAULT_ROLES)
        self.elapsed_s = 0.
        self.host_origin_s = host_origin_s
        self.device_origins_s = dict(zip(self.ids, device_origins_s))
        self.senders = {rid: WireCommandSender(rid, session_id="wire-lab-controller") for rid in self.ids}
        self.receivers = {rid: FakeRobotReceiver(rid) for rid in self.ids}
        self._rx = {rid: FrameDecoder() for rid in self.ids}
        self._tx = {rid: FrameDecoder() for rid in self.ids}
        self._frame_seq = 0
        self.controller = ClosedLoopController(roles=DEFAULT_ROLES, session_id="wire-lab-controller",
            drive_model="differential_body", field_size_mm=(2000., 2200.),
            radii_mm={rid: 20. for rid in self.ids},
            goals={rid: {"x_mm": 1500., "y_mm": 350.+i*500.} for i, rid in enumerate(self.ids)})

    @property
    def host_now(self):
        return self.host_origin_s + self.elapsed_s

    def device_now(self, robot_id):
        return self.device_origins_s[robot_id] + self.elapsed_s

    def advance(self, seconds):
        if type(seconds) not in (float, int) or not math.isfinite(seconds) or not 0 <= seconds <= 10:
            raise ValueError("Bounded nonnegative virtual time step required")
        self.elapsed_s += seconds
        for rid in self.ids:
            self.receivers[rid].tick(self.device_now(rid))
            self.senders[rid].poll(self.host_now)

    def connect(self):
        """Explicit operator-like handshake, not an automatic retry policy."""
        for rid in self.ids:
            self._rx[rid].reset()
            self._tx[rid].reset()
            self.exchange(rid, self.senders[rid].hello(self.host_now))
            self.exchange(rid, self.senders[rid].arm(self.host_now))

    def deliver(self, robot_id, data):
        """Deliver arbitrary bytes for corruption/fragmentation fault tests."""
        try:
            messages = self._rx[robot_id].feed(data)
        except WireError:
            self.receivers[robot_id].disconnect(self.device_now(robot_id))
            raise
        return [self.receivers[robot_id].receive(message, self.device_now(robot_id)) for message in messages]

    def exchange(self, robot_id, message, *, delay_s=0., ack_delay_s=0., drop_request=False, drop_ack=False):
        """Serialize both directions; arbitrary link delays use receiver-local TTL."""
        self.advance(delay_s)
        if drop_request:
            return None
        raw = encode_frame(message)
        # Exercise stream framing, not a direct sender->receiver dict shortcut.
        responses = self.deliver(robot_id, raw[:7]) + self.deliver(robot_id, raw[7:])
        if len(responses) != 1:
            raise ValueError("One request/response expected")
        response = responses[0]
        self.advance(ack_delay_s)
        if not drop_ack:
            encoded = encode_frame(response)
            decoded = self._tx[robot_id].feed(encoded[:11]) + self._tx[robot_id].feed(encoded[11:])
            if len(decoded) != 1:
                raise ValueError("One response expected")
            self.senders[robot_id].accept_response(decoded[0], self.host_now)
        return response

    def controller_packet(self):
        """Existing controller, synthetic stationary camera evidence, no plant."""
        self._frame_seq += 1
        record = {"source_name": "wire-lab-synthetic", "sequence": self._frame_seq,
            "captured_at_s": self.host_now, "observation_usable": True, "stop_required": False,
            "tracks": [{"robot_id": rid, "robot_center_mm": [500., 350.+i*500.], "heading_rad": 0.,
                "observed_at_s": self.host_now, "velocity_mm_s": [0., 0.], "angular_velocity_rad_s": 0.,
                "state": "observed", "valid_for_control": True} for i, rid in enumerate(self.ids)]}
        return self.controller.tick(record, self.host_now)

    def send_controller_packet(self, packet):
        # Prepare all four BEFORE delivering any: malformed fleet input cannot
        # partially reach the robots. This is not distributed atomic actuation.
        messages = {rid: self.senders[rid].drive_from_packet(packet, self.host_now) for rid in self.ids}
        return {rid: self.exchange(rid, messages[rid]) for rid in self.ids}

    def stopped(self):
        return all(r.snapshot()["v_mm_s"] == r.snapshot()["omega_rad_s"] == 0 for r in self.receivers.values())


def run_demo():
    """Return reproducible checks, not simulated competition scores."""
    checks = {}
    lab = WireLab()
    lab.connect()
    for _ in range(6):
        lab.advance(.02)
        replies = lab.send_controller_packet(lab.controller_packet())
    checks["four_robot_roundtrip"] = all(r["result"] == "accepted" and r["v_mm_s"] > 0 for r in replies.values())
    lab.advance(.3)
    checks["host_silence_receiver_watchdog"] = lab.stopped()

    lab = WireLab()
    lab.connect()
    lab.advance(.01)
    message = lab.senders["H1"].drive_from_packet(lab.controller_packet(), lab.host_now)
    reply = lab.exchange("H1", message, delay_s=.31)
    checks["delayed_packet_cannot_restart"] = reply["result"] == "rejected" and lab.stopped()

    lab = WireLab()
    lab.connect()
    lab.advance(.01)
    message = lab.senders["H1"].drive_from_packet(lab.controller_packet(), lab.host_now)
    lab.exchange("H1", message, drop_ack=True)
    count = lab.receivers["H1"].snapshot()["accepted_drive_count"]
    lab.advance(.3)
    checks["lost_ack_no_retry_or_success"] = (count == 1 and lab.stopped()
        and lab.senders["H1"].snapshot()["execution"] == "unknown")

    lab = WireLab()
    lab.connect()
    lab.advance(.01)
    message = lab.senders["H1"].drive_from_packet(lab.controller_packet(), lab.host_now)
    lab.exchange("H1", message)
    duplicate = lab.deliver("H1", encode_frame(message))[0]
    checks["duplicate_rejected_once_only"] = (duplicate["result"] == "rejected" and lab.stopped()
        and lab.receivers["H1"].snapshot()["accepted_drive_count"] == 1)

    lab = WireLab()
    lab.connect()
    lab.advance(.01)
    old = lab.senders["H1"].drive_from_packet(lab.controller_packet(), lab.host_now)
    lab.connect()
    reply = lab.deliver("H1", encode_frame(old))[0]
    checks["reconnect_retires_old_link"] = reply["result"] == "rejected" and lab.stopped()

    lab = WireLab()
    lab.connect()
    lab.advance(.01)
    lab.send_controller_packet(lab.controller_packet())
    for rid in lab.ids:
        lab.exchange(rid, lab.senders[rid].stop(lab.host_now, reason="demo_end", emergency=True))
    checks["four_robot_estop_zero"] = lab.stopped() and all(r.snapshot()["state"] == "estop" for r in lab.receivers.values())
    return {"schema_version": 1, "test": "four_robot_wire_fault_lab", "checks": checks,
        "passed": all(checks.values()), "clock_mode": "independent_origin_virtual_clocks",
        "transport": "in_memory_framed_bytes", "synthetic": True, "physical_execution": "unknown",
        "final_all_zero": lab.stopped(), "device_io": False, "hardware_ready": False, "motion_permitted": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compact", action="store_true", help="Print compact JSON")
    args = parser.parse_args(argv)
    result = run_demo()
    print(json.dumps(result, ensure_ascii=False, indent=None if args.compact else 2, allow_nan=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
