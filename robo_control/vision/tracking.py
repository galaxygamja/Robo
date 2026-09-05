"""Measured-pose history for an arbitrary registry; never extrapolate through loss.

This is a localization boundary, not a motor driver. Host monotonic timestamps
are local to one camera process/session and must not be compared across hosts.
"""
from __future__ import annotations

import math
from collections import Counter

from ..fleet import robot_id_valid


class PoseTracker:
    def __init__(self, robot_ids, *, stale_after_s=0.5, max_speed_mm_s=1500.0,
                 max_turn_rad_s=15.0):
        ids = tuple(robot_ids)
        if not ids or len(set(ids)) != len(ids) or not all(map(robot_id_valid, ids)):
            raise ValueError("Unique registered robot IDs required")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0
               for v in (stale_after_s, max_speed_mm_s, max_turn_rad_s)):
            raise ValueError("Positive finite tracking limits required")
        self.ids = ids
        self.stale_after_s = stale_after_s
        self.max_speed_mm_s = max_speed_mm_s
        self.max_turn_rad_s = max_turn_rad_s
        self.last = {}
        self.source = None
        self.sequence = 0
        self.stamp = -math.inf
        self.now = -math.inf

    def update(self, record: dict, now_s: float) -> dict:
        if not isinstance(now_s, (int, float)) or isinstance(now_s, bool) or not math.isfinite(now_s) or now_s < self.now:
            raise ValueError("Tracker clock must be finite and monotonic")
        self.now = now_s
        if not isinstance(record, dict):
            record = {"status": "malformed_record"}
        sequence = record.get("sequence", record.get("frame_sequence"))
        stamp = record.get("captured_at_s")
        source = record.get("source_name")
        reason = None
        if not isinstance(source, str) or not source or (self.source is not None and source != self.source):
            reason = "source_changed"
        elif type(sequence) is not int or sequence <= self.sequence:
            reason = "out_of_order_sequence"
        elif isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            reason = "invalid_timestamp"
        elif not self.stamp < stamp <= now_s or now_s - stamp > self.stale_after_s:
            reason = "stale_or_out_of_order_timestamp"
        else:
            self.source, self.sequence, self.stamp = source, sequence, stamp
        observations = record.get("robots", []) if reason is None and record.get("status") == "detected" else []
        if not isinstance(observations, (list, tuple)) or any(not isinstance(o, dict)
                or not robot_id_valid(o.get("robot_id")) for o in observations):
            observations = []
            reason = "malformed_observations"
        counts = Counter(o.get("robot_id") for o in observations)
        accepted, invalid = set(), {}
        for obs in observations:
            rid = obs.get("robot_id")
            if rid not in self.ids or counts[rid] != 1:
                invalid[str(rid)] = "unknown_or_duplicate_robot"
                continue
            point, angle = obs.get("robot_center_mm"), obs.get("heading_rad")
            values = [*(point if isinstance(point, (list, tuple)) else []), angle]
            if len(values) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
                invalid[rid] = "invalid_pose"
                continue
            previous = self.last.get(rid)
            vx = vy = omega = 0.0
            # Video decoding is unpaced. Media time is used ONLY for replay
            # kinematics; freshness always stays on the host monotonic clock.
            motion_stamp = record.get("media_time_s") if record.get("is_replay") else stamp
            if motion_stamp is not None and (isinstance(motion_stamp, bool) or not isinstance(motion_stamp, (int,float)) or not math.isfinite(motion_stamp)):
                invalid[rid] = "invalid_media_time"
                continue
            if previous is not None and stamp - previous["observed_at_s"] <= self.stale_after_s:
                dt = None if motion_stamp is None or previous["motion_time_s"] is None else motion_stamp - previous["motion_time_s"]
                if dt is None:
                    dt = math.inf  # A still image has no velocity evidence.
                if dt <= 0:
                    invalid[rid] = "nonpositive_pose_dt"
                    continue
                vx, vy = (point[i] - previous["robot_center_mm"][i] for i in range(2))
                vx, vy = vx / dt, vy / dt
                omega = ((angle - previous["heading_rad"] + math.pi) % (2 * math.pi) - math.pi) / dt
                if math.hypot(vx, vy) > self.max_speed_mm_s or abs(omega) > self.max_turn_rad_s:
                    invalid[rid] = "pose_jump"
                    continue
            self.last[rid] = {"robot_id": rid, "robot_center_mm": list(point), "heading_rad": angle,
                              "observed_at_s": stamp, "motion_time_s": motion_stamp,
                              "velocity_mm_s": [vx, vy], "angular_velocity_rad_s": omega}
            accepted.add(rid)
        tracks = []
        for rid in self.ids:
            pose = self.last.get(rid)
            age = None if pose is None else now_s - pose["observed_at_s"]
            state = "observed" if rid in accepted else "stale" if age is not None and age >= self.stale_after_s else "missing"
            tracks.append({**(pose or {"robot_id": rid, "robot_center_mm": None, "heading_rad": None}),
                           "age_ms": None if age is None else age * 1000, "state": state,
                           "valid_for_control": rid in accepted, "reason": invalid.get(rid, reason)})
        usable = (len(accepted) == len(self.ids) and not invalid and not record.get("unknown_tag_ids")
                  and not record.get("duplicate_tag_ids") and record.get("observation_complete") is True)
        return {"tracks": tracks, "tracking_rejections": invalid, "tracking_frame_reason": reason,
                "observation_usable": usable, "stop_required": not usable,
                "measurement_setup_marked_verified": usable and record.get("hardware_verified") is True
                    and isinstance(record.get("tag_size_mm"), (int, float))
                    and not isinstance(record.get("tag_size_mm"), bool) and math.isfinite(record["tag_size_mm"]) and record["tag_size_mm"] > 0
                    and record.get("is_replay") is False,
                "hardware_ready": False, "motion_permitted": False, "device_io": False}

    def snapshot(self, now_s: float) -> dict:
        """Watchdog tick for a consumer even when no camera frames arrive."""
        return self.update({"status": "no_frame"}, now_s)
