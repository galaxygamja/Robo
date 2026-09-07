"""Read-only diagnostics for a real-camera runtime JSONL session."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path


def _number(value):
    return not isinstance(value, bool) and isinstance(value, (float, int)) and math.isfinite(value)


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
            if any(row.get(key) is not False for key in ("device_io", "motion_permitted")):
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
                if drive_model not in {"mecanum", "differential_body"}:
                    raise ValueError("Unknown drive model")
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
