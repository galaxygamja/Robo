"""Read-only diagnostics for a real-camera runtime JSONL session."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

from .mission_bindings import label, validate_binding_plan
from .observed_pickup import PickupPolicy
from .obstacle_report import inspect_obstacle_snapshot
from .wire_sender import _response_valid
from .world_state import PieceSpec

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


def _review_world(world, *, header, at_s):
    """Source consistency for the new reviewed-operation evidence modes."""
    if (not isinstance(world, dict) or world.get("session_id") != header["session_id"]
            or world.get("source_name") != header.get("source_name")
            or world.get("configuration_id") != header.get("configuration_id")
            or world.get("is_replay") is not header.get("is_replay")
            or world.get("generated_at_s") != at_s
            or world.get("coordinate_system") != "bottom_left_x_right_y_up_mm"
            or type(world.get("ready")) is not bool
            or type(world.get("source_sequence")) is not int or world["source_sequence"] < 0
            or any(world.get(k) is not False for k in
                   ("device_io", "hardware_ready", "motion_permitted", "physical_identity_verified"))):
        raise ValueError("Reviewed operation needs matching current world provenance/evidence")
    dimensions = world.get("field_size_mm")
    if (not isinstance(dimensions, (list, tuple)) or len(dimensions) != 2
            or any(not _number(v) or not 0 < v <= 1e6 for v in dimensions)):
        raise ValueError("Reviewed world needs bounded field dimensions in mm")
    for name in ("binding_plan", "calibration"):
        context = header.get(name)
        if isinstance(context, dict) and context.get("field_size_mm") != list(dimensions):
            raise ValueError("Reviewed world field dimensions changed from startup evidence")


def _binding_snapshot(value, *, plan, world, session):
    """Validate historical mapping claims, not physical identity or live freshness."""
    required = {"schema_version", "mode", "status", "reason", "session_id", "applied_sequence",
                "mappings", "candidates", "reviewed_by", "evidence", "physical_identity_verified",
                "device_io", "motion_permitted"}
    if (not isinstance(value, dict) or set(value) != required
            or type(value["schema_version"]) is not int or value["schema_version"] != 1
            or value["mode"] != "reviewed_startup_regions"
            or not isinstance(value["status"], str) or value["status"] not in {"waiting", "bound", "fault"}
            or any(value[k] is not False for k in ("physical_identity_verified", "device_io", "motion_permitted"))
            or value["session_id"] not in (None, session)
            or value["reviewed_by"] != plan["reviewed_by"] or value["evidence"] != plan["evidence"]):
        raise ValueError("Invalid reviewed binding envelope")
    ids = {entry["piece_id"] for entry in plan["bindings"]}
    mappings, sequence, reason = value["mappings"], value["applied_sequence"], value["reason"]
    if (not isinstance(mappings, dict) or any(not label(oid) for oid in mappings.values())
            or len(set(mappings.values())) != len(mappings)
            or not (reason is None or label(reason))):
        raise ValueError("Invalid binding map/reason")
    if sequence is None:
        if mappings or value["status"] == "bound":
            raise ValueError("Unapplied bindings cannot claim a completed mapping")
    elif (type(sequence) is not int or not 1 <= sequence <= world["source_sequence"]
          or set(mappings) != ids or value["session_id"] != session or value["status"] == "waiting"):
        raise ValueError("Applied bindings require a complete unique map and recorded source sequence")
    if sequence is not None:
        pieces = world.get("pieces")
        if (not isinstance(pieces, list) or len(pieces) > 512
                or any(not isinstance(p, dict) or not label(p.get("piece_id")) for p in pieces)
                or len({p["piece_id"] for p in pieces}) != len(pieces)):
            raise ValueError("Applied binding world needs a unique piece catalog")
        by_id = {p["piece_id"]: p for p in pieces}
        if any(pid not in by_id or by_id[pid].get("track_id") != oid for pid, oid in mappings.items()):
            raise ValueError("Applied binding map contradicts current world piece identities")
        if any(by_id[e["piece_id"]].get("kind") != e["kind"] or by_id[e["piece_id"]].get("colour") not in (None, e["colour"])
               for e in plan["bindings"]):
            raise ValueError("Applied binding world classes changed from the reviewed catalog")
    if (value["status"] == "bound") != (reason is None):
        raise ValueError("Binding status/reason disagree")
    candidates = value["candidates"]
    if (not isinstance(candidates, list) or len(candidates) not in (0, len(ids))
            or any(not isinstance(row, dict) or set(row) != {"piece_id", "candidate_track_ids", "reason"}
                   or not isinstance(row["piece_id"], str) for row in candidates)
            or candidates and {row["piece_id"] for row in candidates} != ids):
        raise ValueError("Invalid binding candidate registry")
    for row in candidates:
        tracks = row["candidate_track_ids"]
        if (not isinstance(tracks, list) or len(tracks) > 512 or any(not label(oid) for oid in tracks)
                or len(set(tracks)) != len(tracks) or not (row["reason"] is None or label(row["reason"]))):
            raise ValueError("Invalid binding candidate evidence")
    return value


def _pickup_snapshot(value, *, mission, plan, session, at_s):
    """Check the stated derivation; this still cannot verify physical geometry."""
    keys = {"mode", "task_id", "piece_id", "session_id", "track_id", "source_sequence", "observed_at_s",
            "object_position_mm", "anchor_position_mm", "anchor_drift_mm", "robot_goal", "replans",
            "fault", "position_evidence", "physical_pickup_verified", "device_io", "motion_permitted"}
    if (not isinstance(value, dict) or set(value) != keys or value["mode"] != "observed_piece"
            or any(value[k] is not False for k in ("physical_pickup_verified", "device_io", "motion_permitted"))
            or not label(value["task_id"]) or not label(value["piece_id"])
            or value["session_id"] not in (None, session)
            or not isinstance(plan, dict) or type(plan.get("schema_version")) is not int
            or plan["schema_version"] != 2 or not isinstance(plan.get("tasks"), list)):
        raise ValueError("Invalid observed pickup envelope or missing schema 2 plan")
    entries = [entry for entry in plan["tasks"] if isinstance(entry, dict) and entry.get("task_id") == value["task_id"]]
    world = mission.get("world")
    if (len(entries) != 1 or not isinstance(world, dict) or world.get("session_id") != session
            or type(world.get("source_sequence")) is not int or world["source_sequence"] < 0):
        raise ValueError("Observed pickup requires a matching plan task/world")
    policy = PickupPolicy.parse(entries[0].get("pickup"), field_size_mm=world["field_size_mm"])
    if (type(value["source_sequence"]) is not int or not 0 <= value["source_sequence"] <= world["source_sequence"]
            or type(value["replans"]) is not int or not 0 <= value["replans"] <= policy.max_replans
            or not (value["fault"] is None or label(value["fault"]))):
        raise ValueError("Invalid observed pickup sequence/budget/fault")
    point, anchor, goal = value["object_position_mm"], value["anchor_position_mm"], value["robot_goal"]
    if point is None:
        if any(value[k] is not None for k in ("anchor_position_mm", "robot_goal", "observed_at_s",
                                            "position_evidence", "track_id", "anchor_drift_mm")):
            raise ValueError("Unobserved pickup cannot contain geometry evidence")
    else:
        if (any(not isinstance(p, (list, tuple)) or len(p) != 2 or not all(_number(v) for v in p)
                for p in (point, anchor)) or not label(value["track_id"])
                or not _number(value["observed_at_s"]) or not 0 <= value["observed_at_s"] <= at_s
                or value["position_evidence"] != "vision" or not _number(value["anchor_drift_mm"])
                or not math.isclose(value["anchor_drift_mm"], math.dist(point, anchor), abs_tol=1e-6)
                or value["session_id"] != session or value["source_sequence"] == 0):
            raise ValueError("Invalid observed pickup position/time evidence")
        if any(not 0 <= v <= limit for p in (point, anchor) for v, limit in zip(p, world["field_size_mm"])):
            raise ValueError("Observed pickup evidence is outside the calibrated world")
    if goal is not None:
        if (point is None or value["fault"] is not None or not isinstance(goal, dict)
                or set(goal) != {"x_mm", "y_mm", "heading_rad"} or not all(_number(v) for v in goal.values())
                or value["anchor_drift_mm"] > policy.max_anchor_drift_mm):
            raise ValueError("Faulted or unobserved pickup cannot claim a usable goal")
        heading, forward, left = policy.heading_rad, policy.tool_forward_mm, policy.tool_left_mm
        expected = {"x_mm": point[0]-forward*math.cos(heading)+left*math.sin(heading),
                    "y_mm": point[1]-forward*math.sin(heading)-left*math.cos(heading), "heading_rad": heading}
        if any(not math.isclose(goal[k], v, rel_tol=0, abs_tol=1e-6) for k, v in expected.items()):
            raise ValueError("Observed pickup goal does not match measured position + reviewed offset")
    if type(mission.get("observed_pickup_in_use")) is not bool:
        raise ValueError("Observed pickup needs an explicit current-use flag")
    if mission["observed_pickup_in_use"] and (goal is None or value["task_id"] != mission.get("active_task_id")
            and mission.get("active_task_id") is not None or value["fault"] is not None
            or mission.get("fault") is not None or mission.get("status") == "closed"):
        raise ValueError("Invalid current-use claim for observed pickup")
    if mission["observed_pickup_in_use"]:
        task_index = mission.get("task_index")
        if (type(task_index) is not int or not 0 <= task_index < len(plan["tasks"])
                or plan["tasks"][task_index].get("task_id") != value["task_id"]):
            raise ValueError("Previous task's observed pickup cannot be current during next-task startup")
        if (world.get("ready") is not True or value["source_sequence"] != world["source_sequence"]
                or value["observed_at_s"] != world.get("captured_at_s")
                or mission.get("phase") not in (None, "approach", "align_pickup")):
            raise ValueError("Current pickup needs the current ready world evidence and pickup phase")
        selected = []
        for name, key, identity in (("pieces", "piece_id", value["piece_id"]),
                                    ("objects", "object_id", value["track_id"])):
            rows = world.get(name)
            if (not isinstance(rows, list) or len(rows) > 512
                    or any(not isinstance(row, dict) or not label(row.get(key)) for row in rows)
                    or len({row[key] for row in rows}) != len(rows)):
                raise ValueError("Current pickup world needs bounded unique identity evidence")
            matching = [row for row in rows if row[key] == identity]
            if len(matching) != 1:
                raise ValueError("Current pickup identity has no matching world evidence")
            row = matching[0]
            if (row.get("valid_for_pick") is not True or row.get("position_valid") is not True
                    or row.get("observed_at_s") != value["observed_at_s"]
                    or not isinstance(row.get("position_mm"), (list, tuple))
                    or not all(_number(v) for v in row["position_mm"])
                    or tuple(row["position_mm"]) != tuple(point)):
                raise ValueError("Current pickup position is not current pickable world evidence")
            selected.append(row)
        piece, obj = selected
        if (piece.get("track_id") != value["track_id"] or obj.get("identity_uncertain") is not False
                or obj.get("position_evidence") != "vision" or obj.get("kind") == "cube"
                or piece.get("kind") != obj.get("kind")):
            raise ValueError("Current pickup world identity/class evidence is inconsistent")
    return value


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
    binding_plan = binding_map = binding_sequence = binding_fault = None
    binding_plan_checked = False
    binding_wait_ticks = binding_fault_ticks = 0
    observe_only = False
    header = None
    pickup_history = {}
    obstacle_plan = previous_obstacles = None
    obstacle_fault_ticks = 0
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
                header = row
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
                binding_plan = row.get("binding_plan")
                obstacle_plan = row.get("obstacle_plan")
                observe_only = row.get("mission_observe_only", False)
                if (type(observe_only) is not bool or binding_plan is not None and not isinstance(binding_plan, dict)
                        or (observe_only or binding_plan is not None) and drive_model != "differential_body"
                        or observe_only and (binding_plan is not None or transport_mode != "mock")):
                    raise ValueError("Invalid mission observation/binding mode")
                if obstacle_plan is not None and (not isinstance(obstacle_plan, dict) or drive_model != "differential_body"):
                    raise ValueError("Object obstacle plan needs differential mission mode")
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
            if row.get("mission_observe_only", False) is not observe_only:
                raise ValueError("Mixed mission observation-only mode")
            mission = row.get("mission")
            obstacles = row.get("object_obstacles")
            pickup = mission.get("observed_pickup") if isinstance(mission, dict) else None
            if binding_plan is not None or obstacle_plan is not None or pickup is not None:
                _review_world(mission.get("world") if isinstance(mission, dict) else None, header=header, at_s=stamp)
            if obstacle_plan is None:
                if obstacles is not None:
                    raise ValueError("Unconfigured runtime contains object-obstacle claims")
            else:
                previous_obstacles = inspect_obstacle_snapshot(obstacles, plan=obstacle_plan,
                    world=mission.get("world") if isinstance(mission, dict) else None,
                    header=header, at_s=stamp, previous=previous_obstacles)
                obstacle_fault_ticks += status != "closed" and obstacles["fault"] is not None
            if pickup is not None:
                _pickup_snapshot(pickup, mission=mission, plan=header.get("mission_plan"), session=session, at_s=stamp)
                old = pickup_history.get(pickup["task_id"])
                if old is not None and (pickup["source_sequence"] < old["source_sequence"]
                        or pickup["replans"] < old["replans"] or pickup["anchor_position_mm"] != old["anchor_position_mm"]
                        or pickup["track_id"] != old["track_id"] or pickup["piece_id"] != old["piece_id"]
                        or old["fault"] is not None and pickup["fault"] != old["fault"]):
                    raise ValueError("Observed pickup anchor/identity/budget history changed")
                pickup_history[pickup["task_id"]] = pickup
            bindings = row.get("bindings")
            if binding_plan is None:
                if bindings is not None:
                    raise ValueError("Unconfigured runtime contains binding claims")
            else:
                mission = row.get("mission")
                world = mission.get("world") if isinstance(mission, dict) else None
                if (not isinstance(world, dict) or world.get("session_id") != session
                        or type(world.get("source_sequence")) is not int or world["source_sequence"] < 0):
                    raise ValueError("Binding report requires the matching mission world")
                if not binding_plan_checked:
                    pieces = world.get("pieces")
                    if (not isinstance(pieces, list) or len(pieces) > 512
                            or any(not isinstance(p, dict) or not {"piece_id", "kind", "colour"} <= set(p)
                                   for p in pieces)):
                        raise ValueError("Binding world needs a bounded piece catalog")
                    catalog = [PieceSpec(p["piece_id"], p["kind"], p["colour"]) for p in pieces]
                    if len({p.piece_id for p in catalog}) != len(catalog):
                        raise ValueError("Duplicate binding world piece ID")
                    validate_binding_plan(binding_plan, pieces=catalog, field_size_mm=world["field_size_mm"],
                        source_name=header.get("source_name"), is_replay=header.get("is_replay"),
                        observation_profile_id=header.get("observation_profile_id"), required_piece_ids=())
                    binding_plan_checked = True
                _binding_snapshot(bindings, plan=binding_plan, world=world, session=session)
                if binding_sequence is not None and (bindings["applied_sequence"] != binding_sequence
                                                      or bindings["mappings"] != binding_map):
                    raise ValueError("Binding batch changed after application")
                if bindings["applied_sequence"] is not None:
                    binding_sequence, binding_map = bindings["applied_sequence"], bindings["mappings"]
                if binding_fault is not None and (bindings["status"] != "fault" or bindings["reason"] != binding_fault):
                    raise ValueError("Latched binding fault disappeared or changed")
                if bindings["status"] == "fault":
                    binding_fault = bindings["reason"]
                binding_wait_ticks += status != "closed" and bindings["status"] == "waiting"
                binding_fault_ticks += status != "closed" and bindings["status"] == "fault"
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
                if (observe_only or bindings is not None and bindings["status"] != "bound") and (
                        any(v != 0 for v in (*velocity, omega, *wheels))
                        or drive_model == "differential_body" and robot["forward_velocity_mm_s"] != 0):
                    raise ValueError("Observation-only or blocked bindings cannot contain movement output")
                if obstacles is not None and not obstacles["ready"] and (
                        any(v != 0 for v in (*velocity, omega, *wheels))
                        or drive_model == "differential_body" and robot["forward_velocity_mm_s"] != 0):
                    raise ValueError("Blocked obstacle map cannot contain movement output")
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
            "mission_observe_only": observe_only, "bindings": last.get("bindings"),
            "binding_wait_ticks": binding_wait_ticks, "binding_fault_ticks": binding_fault_ticks,
            "object_obstacles": last.get("object_obstacles"), "object_obstacle_fault_ticks": obstacle_fault_ticks,
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
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"runtime-report: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=True, allow_nan=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
