"""Infer candidate wheel-speed curve points from a live measured-pose JSONL log.

No device IO or profile writes occur. This uses differential-drive axle
kinematics, not wheel encoders: wheel slip and the actual applied PWM are not
verified. Select only a constant-PWM, steady-speed portion of the recording.
Record with `python -m robo_control.vision detect --camera 0 ... --track --report ...`.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from .fleet import robot_id_valid
from .vision.calibration import COORDINATE_SYSTEM


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _records(path):
    with Path(path).open(encoding="utf-8-sig") as handle:
        for number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except (ValueError, RecursionError) as exc:
                raise ValueError(f"Invalid JSONL at line {number}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"JSONL line {number} must be an object")
            yield row


def _window(records, start_s, end_s):
    for row in records:
        if not isinstance(row, dict):
            raise ValueError("Every log record must be an object")
        if row.get("event") == "runtime_tick":
            observation = row.get("observation")
            if observation is None:
                at = row.get("at_s")
                if (_finite(at) and start_s <= at <= end_s
                        and (row.get("fresh_streak", 0) == 0 or row.get("closed_reason"))):
                    raise ValueError("Selected runtime window contains lost/invalid observations")
                continue  # ordinary between-camera-frame control tick
            row = observation
            if not isinstance(row, dict):
                raise ValueError("Malformed runtime observation")
        elif row.get("event") in {"session_started", "hardware_session_started"}:
            continue
        stamp = row.get("captured_at_s")
        if not _finite(stamp):
            raise ValueError("A frame without a finite capture timestamp cannot be assigned to a window")
        # Validate data only AFTER selection; calibration deliberately ignores
        # acceleration, stopping and rejected frames outside the chosen window.
        if start_s <= stamp <= end_s:
            yield row


def _stats(values, durations):
    total = sum(durations)
    mean = sum(v * dt for v, dt in zip(values, durations)) / total
    std = math.sqrt(sum(dt * (v - mean) ** 2 for v, dt in zip(values, durations)) / total)
    return {"mean": mean, "stddev": std, "minimum": min(values), "maximum": max(values),
            "relative_stddev": std / abs(mean) if abs(mean) >= 1. else None}


def estimate(records, *, robot_id, track_width_mm, start_s, end_s,
             left_pwm, right_pwm, max_lateral_mm_s=20.):
    if not robot_id_valid(robot_id):
        raise ValueError("A registered robot ID is required")
    if not _finite(track_width_mm) or not 10 <= track_width_mm <= 500:
        raise ValueError("Measured wheel-centre track width must be in [10,500] mm")
    if not all(_finite(v) for v in (start_s, end_s)) or not 0 <= start_s < end_s:
        raise ValueError("Use increasing nonnegative capture-time window bounds")
    if (any(type(v) is not int or abs(v) > 96 for v in (left_pwm, right_pwm))
            or left_pwm == right_pwm == 0):
        raise ValueError("Supply commanded left/right PWM in [-96,96], with at least one nonzero")
    if not _finite(max_lateral_mm_s) or not 0 < max_lateral_mm_s <= 100:
        raise ValueError("Lateral residual threshold must be in (0,100] mm/s")
    selected = list(_window(records, start_s, end_s))
    if len(selected) < 4:
        raise ValueError("Selected window needs at least four measured frames")
    poses = []
    identity = None
    previous_sequence = None
    sequence_gaps = 0
    for row in selected:
        stamp, sequence = row["captured_at_s"], row.get("sequence")
        processed = row.get("processed_at_s")
        source = row.get("source_name")
        if (row.get("status") != "detected" or row.get("observation_usable") is not True
                or row.get("observation_complete") is not True or row.get("is_replay") is not False
                or row.get("synthetic") is True or not isinstance(source, str) or not source.startswith("webcam:")
                or row.get("coordinate_system") != COORDINATE_SYSTEM
                or row.get("unknown_tag_ids") or row.get("duplicate_tag_ids")):
            raise ValueError("Selected frames must be complete usable live-camera detections; replay/synthetic/loss rejected")
        if not _finite(processed) or not 0 <= processed - stamp < .2:
            raise ValueError("Selected frame was already stale when processed")
        if "host_age_ms" in row and (not _finite(row["host_age_ms"]) or not 0 <= row["host_age_ms"] < 200):
            raise ValueError("Invalid or stale recorded frame age")
        if type(sequence) is not int or sequence < 1 or previous_sequence is not None and sequence <= previous_sequence:
            raise ValueError("Duplicate or backward selected frame sequence")
        if previous_sequence is not None:
            sequence_gaps += sequence - previous_sequence - 1
        previous_sequence = sequence
        field = row.get("field_size_mm")
        if (not isinstance(field, (list, tuple)) or len(field) != 2
                or any(not _finite(v) or v <= 0 for v in field)):
            raise ValueError("Selected detection needs finite field-mm dimensions")
        frame_identity = (source, row.get("configuration_id"), tuple(field))
        if identity is not None and frame_identity != identity:
            raise ValueError("Camera/configuration/field changed during selected window")
        identity = frame_identity
        registered = row.get("registered_robot_ids")
        if not isinstance(registered, list) or registered.count(robot_id) != 1:
            raise ValueError("Selected robot is not uniquely registered")
        robots = row.get("robots")
        if not isinstance(robots, list) or any(not isinstance(r, dict) for r in robots):
            raise ValueError("Malformed selected robot observations")
        matches = [r for r in robots if r.get("robot_id") == robot_id]
        if len(matches) != 1:
            raise ValueError("Selected robot is missing or duplicated in a frame")
        point, heading = matches[0].get("robot_center_mm"), matches[0].get("heading_rad")
        if (not isinstance(point, (list, tuple)) or len(point) != 2
                or any(not _finite(v) or not 0 <= v <= bound for v, bound in zip(point, field))
                or not _finite(heading)):
            raise ValueError("Nonfinite/out-of-field axle position or heading")
        if poses and not 0 < stamp - poses[-1][0] <= .2 + 1e-9:
            raise ValueError("Duplicate/backward capture time or selected frame gap exceeds 200 ms")
        poses.append((stamp, point[0], point[1], heading))
    duration = poses[-1][0] - poses[0][0]
    if duration < .15 - 1e-9:
        raise ValueError("Actual measured window must span at least 0.15 seconds")
    forward, lateral, yaw, left, right, durations = [], [], [], [], [], []
    for a, b in zip(poses, poses[1:]):
        dt = b[0] - a[0]
        dyaw = (b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi
        omega = dyaw / dt
        if abs(omega) > 15:
            raise ValueError("Implausible/ambiguous heading jump in calibration window")
        mid = a[3] + dyaw / 2.
        dx, dy = b[1] - a[1], b[2] - a[2]
        v = (dx * math.cos(mid) + dy * math.sin(mid)) / dt
        sideways = (-dx * math.sin(mid) + dy * math.cos(mid)) / dt
        if max(abs(v - omega * track_width_mm / 2), abs(v + omega * track_width_mm / 2)) > 1000:
            raise ValueError("Inferred speed exceeds supported calibration range")
        durations.append(dt)
        forward.append(v)
        lateral.append(sideways)
        yaw.append(omega)
        left.append(v - omega * track_width_mm / 2)
        right.append(v + omega * track_width_mm / 2)
    lateral_rms = math.sqrt(sum(v*v*dt for v, dt in zip(lateral, durations)) / duration)
    if lateral_rms > max_lateral_mm_s:
        raise ValueError("Excess lateral axle drift: verify axle-centre offset, headings, traction and window")
    stats = {"forward_mm_s": _stats(forward, durations), "lateral_mm_s": _stats(lateral, durations),
             "yaw_rad_s": _stats(yaw, durations), "left_mm_s": _stats(left, durations),
             "right_mm_s": _stats(right, durations)}
    candidates = {}
    for side, pwm in (("left", left_pwm), ("right", right_pwm)):
        if not pwm:
            continue
        speed = stats[f"{side}_mm_s"]["mean"]
        if speed * (1 if pwm > 0 else -1) < 1.:
            raise ValueError(f"{side}: inferred direction disagrees with PWM or speed is below 1 mm/s")
        candidates[f"{side}_{'forward' if pwm > 0 else 'reverse'}"] = [abs(pwm), abs(speed)]
    noisy = any(stats[f"{side}_mm_s"]["relative_stddev"] is None
                or stats[f"{side}_mm_s"]["relative_stddev"] > .2
                for side, pwm in (("left", left_pwm), ("right", right_pwm)) if pwm)
    return {"schema_version": 1, "robot_id": robot_id, "track_width_mm": track_width_mm,
            "requested_window_s": [start_s, end_s], "measured_window_s": [poses[0][0], poses[-1][0]],
            "duration_s": duration, "frames": len(poses), "sequence_gaps": sequence_gaps,
            "commanded_pwm_assumption": {"left": left_pwm, "right": right_pwm},
            "candidate_curve_points": candidates, "statistics": stats, "lateral_rms_mm_s": lateral_rms,
            "high_variability": noisy, "review_required": True,
            "setup_marked_verified": all(r.get("hardware_verified") is True for r in selected),
            "measurement_basis": "camera_axle_kinematics_without_wheel_encoders",
            "limitations": ["robot_center_mm must be the measured axle midpoint",
                            "constant applied PWM is operator-supplied and not verified by this log",
                            "wheel slip is unknown; candidate speed is inferred, not directly measured wheel speed",
                            "review noise and repeat steady-speed runs before editing a profile"],
            "device_io": False, "hardware_result": False, "profile_updated": False,
            "motion_calibrated": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--robot", required=True)
    parser.add_argument("--track-width-mm", type=float, required=True)
    parser.add_argument("--start-s", type=float, required=True, help="host captured_at_s value, not time since file start")
    parser.add_argument("--end-s", type=float, required=True)
    parser.add_argument("--left-pwm", type=int, required=True)
    parser.add_argument("--right-pwm", type=int, required=True)
    parser.add_argument("--max-lateral-mm-s", type=float, default=20.)
    args = parser.parse_args(argv)
    try:
        result = estimate(_records(args.log), robot_id=args.robot, track_width_mm=args.track_width_mm,
            start_s=args.start_s, end_s=args.end_s, left_pwm=args.left_pwm, right_pwm=args.right_pwm,
            max_lateral_mm_s=args.max_lateral_mm_s)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))
        return 0
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(f"hardware_calibration: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
