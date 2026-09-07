"""Read-only diagnostics for a real-camera runtime JSONL session."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

from .wire_sender import _response_valid

_WIRE_STATES = {"idle", "connecting", "ready", "pending", "fault", "closed"}
_REQUESTS = {"hello", "arm", "drive", "stop", "estop", "status"}
_TOKEN = re.compile(r"[A-Za-z0-9_-]{1,128}\Z")


def _number(value):
    try:
        return not isinstance(value, bool) and isinstance(value, (float, int)) and math.isfinite(value)
    except OverflowError:
        return False


def _token(value, *, optional=False):
    return (optional and value is None) or isinstance(value, str) and _TOKEN.fullmatch(value) is not None


def _counter(value):
    return type(value) is int and 0 <= value <= 2**53 - 1


def _synthetic(envelope):
    return (isinstance(envelope, dict) and envelope.get("synthetic") is True
            and envelope.get("execution") == "unknown"
            and all(envelope.get(key) is False
                    for key in ("device_io", "hardware_ready", "motion_permitted")))


def _wire_snapshot(wire, *, registry, session, at_s, terminal):
    """Validate recorded synthetic evidence, without advancing any fake clock.

    Receiver setpoints and stop acknowledgements are separate observations.
    Neither proves movement, sensor completion, or a physical stop.
    """
    if (not _synthetic(wire) or not _token(session) or wire.get("mode") != "fake_wire"
            or wire.get("session_id") != session or not isinstance(wire.get("state"), str)
            or wire["state"] not in _WIRE_STATES
            or any(type(wire.get(key)) is not bool for key in ("started", "closed", "ready"))
            or wire.get("physical_execution") != "unknown"
            or wire.get("receiver_watchdog_model") != "same_process_polled"):
        raise ValueError("Invalid fake-wire envelope")
    state, fault = wire["state"], wire.get("fault")
    if (fault is not None and (not isinstance(fault, str) or not 0 < len(fault) <= 512)
            or wire["closed"] != (state == "closed") or wire["ready"] != (state == "ready")
            or (state == "fault" and fault is None)
            or (fault is not None and state not in {"fault", "closed"})
            or (state == "idle" and wire["started"])
            or (state in {"connecting", "ready", "pending"} and not wire["started"])
            or (terminal and not wire["closed"])):
        raise ValueError("Inconsistent fake-wire state/closed/ready/fault")
    stamp = wire.get("host_at_s")
    if not (stamp is None and state == "idle") and (not _number(stamp) or not 0 <= stamp <= at_s):
        raise ValueError("Invalid fake-wire host time")
    if (not _counter(wire.get("pending_event_count"))
            or (wire["ready"] and wire["pending_event_count"] != 0)):
        raise ValueError("Invalid fake-wire pending event count")
    counters = wire.get("counters")
    if (not isinstance(counters, dict)
            or any(not isinstance(key, str) or not key or not _counter(value) for key, value in counters.items())):
        raise ValueError("Invalid fake-wire counters")
    robots = wire.get("robots")
    if (not isinstance(robots, list) or len(robots) != len(registry)
            or any(not isinstance(robot, dict) or not isinstance(robot.get("robot_id"), str) for robot in robots)
            or {robot["robot_id"] for robot in robots} != registry):
        raise ValueError("Incomplete or duplicate fake-wire fleet")
    unconfirmed, zero = [], True
    for robot in robots:
        rid, sender, receiver = robot["robot_id"], robot.get("sender"), robot.get("receiver")
        if (any(type(robot.get(key)) is not bool for key in ("connected", "stop_requested", "stop_acknowledged"))
                or not _synthetic(sender) or not _synthetic(receiver)
                or sender.get("robot_id") != rid or receiver.get("robot_id") != rid
                or not {"boot_id", "link_id", "last_response"} <= set(sender)
                or not {"host_session_id", "link_id", "receiver_at_s", "command_deadline_receiver_s"} <= set(receiver)
                or sender.get("host_session_id") != session
                or receiver.get("host_session_id") not in (None, session)):
            raise ValueError("Invalid fake-wire robot/session/output flags")
        if (not all(_token(sender.get(key), optional=True) for key in ("boot_id", "link_id"))
                or not _token(receiver.get("boot_id")) or not _token(receiver.get("link_id"), optional=True)
                or (sender["boot_id"] is None) != (sender["link_id"] is None)
                or (receiver["host_session_id"] is None) != (receiver["link_id"] is None)):
            raise ValueError("Invalid fake-wire boot/link identity")
        pending, sender_state = sender.get("pending_request"), sender.get("state")
        if (not isinstance(sender_state, str)
                or sender_state not in {"disconnected", "disarmed", "armed", "estop", "fault",
                                *("awaiting_" + request for request in _REQUESTS)}
                or not (pending is None or isinstance(pending, str))
                or pending not in {None, *_REQUESTS}
                or (pending is not None and sender_state != "awaiting_" + pending)
                or (pending is None and sender_state.startswith("awaiting_"))
                or type(sender.get("permit_available")) is not bool
                or not all(_counter(sender.get(key)) for key in ("wire_sequence", "controller_sequence"))
                or not isinstance(sender.get("reason"), str) or not sender["reason"]):
            raise ValueError("Invalid fake-wire sender state")
        response = sender.get("last_response")
        if response is not None and (not _response_valid(response) or response["robot_id"] != rid
                or response["host_session_id"] != session
                or (response["state"] != "armed" and (response["v_mm_s"] != 0 or response["omega_rad_s"] != 0))):
            raise ValueError("Invalid fake-wire recorded response")
        receiver_state = receiver.get("state")
        velocity, turn = receiver.get("v_mm_s"), receiver.get("omega_rad_s")
        receiver_at, deadline = receiver.get("receiver_at_s"), receiver.get("command_deadline_receiver_s")
        if (not isinstance(receiver_state, str) or receiver_state not in {"disarmed", "armed", "estop"}
                or not _number(velocity) or abs(velocity) > 180
                or not _number(turn) or abs(turn) > 1.5
                or not all(_counter(receiver.get(key)) for key in ("last_sequence", "accepted_drive_count"))
                or not isinstance(receiver.get("reason"), str) or not receiver["reason"]
                or receiver.get("measured_velocity", "missing") is not None
                or (receiver_at is not None and (not _number(receiver_at) or receiver_at < 0))
                or (deadline is not None and (not _number(deadline) or deadline < 0))
                or (receiver_state == "armed" and
                    (receiver_at is None or deadline is None or deadline <= receiver_at
                     or receiver["host_session_id"] != session))
                or (receiver_state != "armed" and (velocity != 0 or turn != 0 or deadline is not None))):
            raise ValueError("Invalid fake-wire receiver state/clock/velocity")
        if wire["ready"] and (not robot["connected"] or sender_state != "armed"
                or pending is not None or not sender["permit_available"] or receiver_state != "armed"
                or robot["stop_requested"] or robot["stop_acknowledged"]
                or receiver["host_session_id"] != session
                or any(sender[key] != receiver[key] for key in ("boot_id", "link_id"))):
            raise ValueError("Fake-wire ready flag lacks complete armed fleet evidence")
        if wire["closed"] and not robot["stop_requested"]:
            raise ValueError("Closed fake-wire session must request every robot stop")
        if robot["stop_acknowledged"]:
            if (not robot["stop_requested"] or response is None
                    or response["request_type"] not in {"stop", "estop"}
                    or response["result"] != "accepted" or response["state"] not in {"disarmed", "estop"}
                    or response["v_mm_s"] != 0 or response["omega_rad_s"] != 0
                    or response["permit_id"] is not None or response["challenge_id"] is not None
                    or response["lease_remaining_ms"] != 0 or receiver_state == "armed"
                    or response["seq"] != sender["wire_sequence"]
                    or response["seq"] > receiver["last_sequence"]
                    or any(response[key] != sender[key] or response[key] != receiver[key]
                           for key in ("boot_id", "link_id"))):
                raise ValueError("Fake-wire stop acknowledgement lacks matching response evidence")
        elif robot["stop_requested"]:
            unconfirmed.append(rid)
        zero = zero and velocity == 0 and turn == 0
    declared = wire.get("unconfirmed_stop_robot_ids")
    if (not isinstance(declared, list) or len(declared) != len(unconfirmed)
            or any(not isinstance(rid, str) for rid in declared) or set(declared) != set(unconfirmed)):
        raise ValueError("Inconsistent fake-wire unconfirmed stop registry")
    return {"zero": zero, "unconfirmed": sorted(unconfirmed), "fault": fault,
            "host_at_s": stamp, "counters": dict(counters)}


def _reject_constant(value):
    raise ValueError(f"Nonfinite JSON constant: {value}")


def _finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"Nonfinite JSON number: {value}")
    return number


def inspect_report(path: str | Path) -> dict:
    """Stream bounded lines; validate session continuity and final zero output.

    These are measured HOST timings and dry-run output records. The result
    cannot establish sensor exposure latency, physical position accuracy,
    firmware operation, or whether a real motor stopped.
    """
    session = registry = mode = None
    transport_mode = "mock"
    wire_fault_ticks = 0
    wire_faults = Counter()
    final_wire = None
    previous_wire_at = None
    first_at = previous_at = None
    previous_tick = 0
    tick_count = frames = usable = commands = 0
    statuses = Counter()
    worst_gap_s = worst_age_ms = age_total_ms = 0.0
    last = None
    drive_model = "mecanum"
    with Path(path).open("r", encoding="utf-8-sig") as handle:
        line_number = 0
        while True:
            line = handle.readline(1048577)
            if not line:
                break
            line_number += 1
            if len(line.encode("utf-8")) > 1048576:
                raise ValueError(f"Line {line_number} exceeds the 1 MiB record limit")
            if not line.endswith("\n"):
                raise ValueError(f"Line {line_number} is incomplete; writer may have been interrupted")
            try:
                row = json.loads(line, parse_constant=_reject_constant, parse_float=_finite_float)
            except (ValueError, json.JSONDecodeError) as exc:
                raise ValueError(f"Invalid JSON on line {line_number}: {exc}") from exc
            if not isinstance(row, dict) or type(row.get("schema_version")) is not int or row["schema_version"] != 1:
                raise ValueError(f"Invalid runtime schema on line {line_number}")
            if (any(row.get(key) is not False for key in ("device_io", "motion_permitted"))
                    or ("hardware_ready" in row and row["hardware_ready"] is not False)):
                raise ValueError(f"Unexpected device output flag on line {line_number}")
            if line_number == 1:
                roles = row.get("roles")
                if (row.get("event") != "session_started" or not isinstance(row.get("session_id"), str)
                        or not row["session_id"] or not isinstance(roles, dict) or not roles
                        or row.get("input_mode") not in {"live_camera", "video_replay"}
                        or row.get("output_mode") != "dry_run_commands"):
                    raise ValueError("Report must begin with a valid session_started configuration")
                session, registry, mode = row["session_id"], set(roles), row["input_mode"]
                drive_model = row.get("drive_model", "mecanum")
                if drive_model not in ("mecanum", "differential_body"):
                    raise ValueError("Unknown drive model")
                transport_mode = row.get("transport_mode", "mock")
                if transport_mode not in ("mock", "fake_wire"):
                    raise ValueError("Unknown runtime transport mode")
                if transport_mode == "fake_wire" and drive_model != "differential_body":
                    raise ValueError("Fake-wire transport requires differential body commands")
                continue
            if (row.get("event") != "runtime_tick" or row.get("session_id") != session
                    or row.get("input_mode") != mode or row.get("output_mode") != "dry_run_commands"):
                raise ValueError(f"Mixed session or mode on line {line_number}")
            if last is not None and last.get("status") == "closed":
                raise ValueError("Records after a closed session are not permitted")
            stamp, tick, status = row.get("at_s"), row.get("tick_sequence"), row.get("status")
            if (not _number(stamp) or stamp < 0 or (previous_at is not None and stamp < previous_at)
                    or type(tick) is not int or tick < previous_tick
                    or (status != "closed" and tick <= previous_tick)
                    or not isinstance(status, str)):
                raise ValueError(f"Invalid timestamp/sequence/status on line {line_number}")
            if status == "closed" and (not isinstance(row.get("closed_reason"), str) or not row["closed_reason"]):
                raise ValueError("Closed session must record a nonempty close reason")
            if row.get("transport_mode", "mock") != transport_mode:
                raise ValueError("Mixed runtime transport mode")
            wire = row.get("wire")
            if transport_mode == "mock":
                if wire is not None:
                    raise ValueError("Mock runtime cannot contain fake-wire evidence")
            else:
                final_wire = _wire_snapshot(wire, registry=registry, session=session,
                                            at_s=stamp, terminal=status == "closed")
                wire_at = final_wire["host_at_s"]
                if wire_at is not None:
                    if previous_wire_at is not None and wire_at < previous_wire_at:
                        raise ValueError("Fake-wire host clock moved backwards")
                    previous_wire_at = wire_at
                if final_wire["fault"] is not None:
                    wire_fault_ticks += status != "closed"
                    wire_faults[final_wire["fault"]] += 1
            if first_at is None:
                first_at = stamp
            if previous_at is not None:
                worst_gap_s = max(worst_gap_s, stamp - previous_at)
            previous_at, previous_tick = stamp, tick
            statuses[status] += 1
            tick_count += status != "closed"
            actuator = row.get("actuator")
            if (not isinstance(actuator, dict) or actuator.get("session_id") != session
                    or any(actuator.get(key) is not False
                           for key in ("device_io", "hardware_ready", "motion_permitted"))):
                raise ValueError(f"Invalid dry-run actuator envelope on line {line_number}")
            robots = actuator.get("robots")
            if actuator.get("drive_model", "mecanum") != drive_model:
                raise ValueError("Mixed actuator drive model")
            if (not isinstance(robots, list) or any(not isinstance(robot, dict) for robot in robots)
                    or any(not isinstance(robot.get("robot_id"), str) for robot in robots)
                    or len(robots) != len(registry) or {robot["robot_id"] for robot in robots} != registry):
                raise ValueError(f"Incomplete or duplicate actuator fleet on line {line_number}")
            for robot in robots:
                velocity, omega, wheels = (robot.get(key) for key in
                    ("velocity_world_mm_s", "angular_velocity_rad_s", "wheel_velocity_rad_s"))
                if (not isinstance(velocity, list) or len(velocity) != 2
                        or not isinstance(wheels, list) or len(wheels) != (4 if drive_model == "mecanum" else 0)
                        or not all(_number(v) for v in (*velocity, omega, *wheels))):
                    raise ValueError(f"Invalid command values on line {line_number}")
                if drive_model == "differential_body":
                    forward = robot.get("forward_velocity_mm_s")
                    if not _number(forward) or (status == "closed" and forward != 0):
                        raise ValueError("Invalid differential body command")
                if status == "closed" and any(v != 0 for v in (*velocity, omega, *wheels)):
                    raise ValueError("Closed session contains nonzero dry-run output")
            observation = row.get("observation")
            if observation is not None:
                if not isinstance(observation, dict):
                    raise ValueError(f"Malformed observation on line {line_number}")
                captured = observation.get("captured_at_s")
                if not _number(captured) or not 0 <= captured <= stamp:
                    raise ValueError(f"Invalid observation time on line {line_number}")
                age = (stamp - captured) * 1000
                worst_age_ms = max(worst_age_ms, age)
                age_total_ms += age
                frames += 1
                usable += observation.get("observation_usable") is True
            commands += row.get("command") is not None
            last = row
    if last is None or last.get("status") != "closed":
        raise ValueError("Report has no final closed/zero-output record")
    return {"session_id": session, "input_mode": mode, "output_mode": "dry_run_commands",
            "transport_mode": transport_mode, "wire_fault_ticks": wire_fault_ticks,
            "wire_fault_reasons": dict(wire_faults),
            "wire_counters": final_wire["counters"] if final_wire else None,
            "final_wire_zero": final_wire["zero"] if final_wire else None,
            "final_wire_stop_acknowledged": not final_wire["unconfirmed"] if final_wire else None,
            "unconfirmed_stop_robot_ids": final_wire["unconfirmed"] if final_wire else [],
            "wire_zero_evidence": "synthetic_receiver_setpoints" if final_wire else None,
            "drive_model": drive_model, "mission": last.get("mission"),
            "log_complete": True, "final_stop_recorded": True, "closed_reason": last["closed_reason"],
            "elapsed_s": previous_at - first_at, "supervisor_ticks": tick_count,
            "max_tick_gap_ms": worst_gap_s * 1000, "observation_frames": frames,
            "usable_observation_frames": usable, "command_records": commands,
            "max_host_observation_age_ms": worst_age_ms,
            "mean_host_observation_age_ms": age_total_ms / frames if frames else None,
            "status_counts": dict(statuses), "device_io": False,
            "physical_position_accuracy_verified": False, "physical_stop_verified": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    args = parser.parse_args(argv)
    try:
        result = inspect_report(args.report)
    except (OSError, ValueError, KeyError) as exc:
        print(f"runtime-report: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=True, allow_nan=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
