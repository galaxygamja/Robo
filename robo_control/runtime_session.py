"""Real measured-image supervision, independent of image decoding and display.

There is no simulated world or pose integrator. Commands are calculated from
observations and retained only in the existing dry-run actuator endpoint.
"""

from __future__ import annotations

import math
from copy import deepcopy

from .control_loop import ClosedLoopController, ControlLimits, MockActuatorBank
from .vision.calibration import COORDINATE_SYSTEM
from .vision.object_tracking import ObjectTracker
from .vision.tracking import PoseTracker


def _finite(value):
    return (not isinstance(value, bool) and isinstance(value, (float, int))
            and math.isfinite(value))


class LiveControlSession:
    """One camera session, one clock domain, finite command lifetime.

    Call advance() even when no records arrive. A missing/new invalid frame
    stops immediately; a silent source expires at the last capture + 200 ms.
    No tick repeats a previous frame as a new observation or renews its TTL.
    Three consecutive usable observations are required after any loss.
    The OS scheduler is not real-time; a late supervisor tick closes the
    session. This is a host runtime, not a replacement for firmware watchdogs.
    """

    def __init__(self, *, roles, field_size_mm, source_name, is_replay=False,
                 goals=None, radii_mm=None, recovery_frames=3, max_tick_gap_s=0.1,
                 limits=None, track_objects=False, session_id=None, configuration_id=None):
        if not isinstance(source_name, str) or not source_name:
            raise ValueError("Pin a nonempty expected camera source name")
        if type(is_replay) is not bool or type(track_objects) is not bool:
            raise ValueError("Replay and object tracking flags must be Boolean")
        if configuration_id is not None and (not isinstance(configuration_id, str) or not configuration_id):
            raise ValueError("Configuration identity must be a nonempty string")
        if type(recovery_frames) is not int or not 2 <= recovery_frames <= 100:
            raise ValueError("Recovery needs 2..100 consecutive fresh frames")
        if not _finite(max_tick_gap_s) or not 0 < max_tick_gap_s <= 0.1:
            raise ValueError("Supervisor maximum tick gap must be in (0, 0.1] s")
        limits = limits or ControlLimits()
        if limits.max_pose_age_s > 0.2:
            raise ValueError("Live runtime accepts observations for at most 200 ms")
        self.controller = ClosedLoopController(roles=roles, goals=goals, radii_mm=radii_mm,
            limits=limits, field_size_mm=field_size_mm, session_id=session_id)
        self.bank = MockActuatorBank(self.controller.ids, session_id=self.controller.session_id,
                                    watchdog_s=limits.command_ttl_s, limits=limits)
        self.tracker = PoseTracker(self.controller.ids)
        self.object_tracker = ObjectTracker(self.controller.ids, max_frame_age_s=limits.max_pose_age_s) if track_objects else None
        self.configuration_id = configuration_id
        self.source_name, self.is_replay = source_name, is_replay
        self.field_size_mm = self.controller.field_size_mm
        self.recovery_frames, self.max_tick_gap_s = recovery_frames, max_tick_gap_s
        self.now = None
        self.tick_sequence = 0
        self.fresh_streak = 0
        self.last_capture_s = None
        self.closed_reason = None
        self.status = "awaiting_observation"
        self.command = None

    def _input_reason(self, record):
        if not isinstance(record, dict):
            return "malformed_detection_record"
        if self.configuration_id is not None and record.get("configuration_id") != self.configuration_id:
            return "configuration_changed"
        if record.get("source_name") != self.source_name:
            return "unexpected_camera_source"
        if record.get("is_replay") is not self.is_replay:
            return "unexpected_replay_mode"
        if record.get("coordinate_system") != COORDINATE_SYSTEM:
            return "wrong_coordinate_system"
        dimensions = record.get("field_size_mm")
        if (not isinstance(dimensions, (tuple, list)) or len(dimensions) != 2
                or any(not _finite(v) for v in dimensions)
                or tuple(dimensions) != self.field_size_mm):
            return "wrong_field_size"
        ids = record.get("registered_robot_ids")
        if (not isinstance(ids, list) or any(not isinstance(rid, str) for rid in ids)
                or len(ids) != len(self.controller.ids) or set(ids) != set(self.controller.ids)):
            return "wrong_robot_registry"
        if record.get("device_io") is not False:
            return "unexpected_device_io"
        if record.get("status") == "detected":
            robots = record.get("robots")
            if not isinstance(robots, list):
                return "malformed_robot_observations"
            for robot in robots:
                point = robot.get("robot_center_mm") if isinstance(robot, dict) else None
                if (not isinstance(point, (tuple, list)) or len(point) != 2
                        or any(not _finite(v) or not 0 <= v <= bound
                               for v, bound in zip(point, self.field_size_mm))):
                    return "out_of_field_or_invalid_pose"
        return None

    def _hold(self, now_s, reason, record=None):
        self.fresh_streak = 0
        self.controller.set_paused(True)
        packet = self.controller.tick(record, now_s)
        packet["stop_reason"] = reason
        self.bank.receive(packet, now_s)
        self.command, self.status = packet, reason
        return packet

    def _event(self, now_s, *, command=None, observation=None):
        return {"schema_version": 1, "event": "runtime_tick",
                "session_id": self.controller.session_id, "tick_sequence": self.tick_sequence,
                "at_s": now_s, "status": self.status, "closed_reason": self.closed_reason,
                "fresh_streak": self.fresh_streak, "recovery_frames": self.recovery_frames,
                "last_observation_age_ms": None if self.last_capture_s is None else
                    max(0.0, (now_s - self.last_capture_s) * 1000),
                "observation": deepcopy(observation), "command": deepcopy(command),
                "object_tracking": self.object_tracker.poll(now_s) if self.object_tracker else None,
                "actuator": self.bank.snapshot(), "is_replay": self.is_replay,
                "input_mode": "video_replay" if self.is_replay else "live_camera",
                "output_mode": "dry_run_commands",
                "device_io": False, "hardware_ready": False, "motion_permitted": False}

    def advance(self, record, now_s):
        if not _finite(now_s) or now_s < 0 or (self.now is not None and now_s < self.now):
            self.close(self.now or 0.0, "invalid_runtime_clock")
            raise ValueError("Runtime clock must be finite, nonnegative and monotonic")
        previous_tick, self.now = self.now, now_s
        self.tick_sequence += 1
        self.bank.tick(now_s)
        if self.closed_reason is not None:
            return self._event(now_s)
        if previous_tick is not None and now_s - previous_tick > self.max_tick_gap_s + 1e-9:
            return self.close(now_s, "supervisor_deadline_missed")
        if isinstance(record, dict) and record.get("status") == "source_closed":
            return self.close(now_s, "source_closed")
        # Expiry resets recovery even if a new valid record arrives this tick.
        expired = (self.last_capture_s is not None
                   and now_s >= self.last_capture_s + self.controller.limits.max_pose_age_s)
        if expired:
            self.fresh_streak = 0
        if record is None:
            if expired and self.status != "observation_watchdog":
                packet = self._hold(now_s, "observation_watchdog")
                return self._event(now_s, command=packet)
            return self._event(now_s)
        # Copy at the boundary: neither upstream mutation nor caller edits to
        # a returned diagnostic record may change tracking/controller history.
        record = deepcopy(record)
        rejection = self._input_reason(record)
        if rejection is not None:
            packet = self._hold(now_s, rejection)
            if self.object_tracker is not None:
                # A known-bad frame is not an ordinary between-frame poll.
                # Invalidate pick evidence without feeding foreign coordinates
                # or a changed configuration into the object's history.
                self.object_tracker.snapshot(now_s)
            return self._event(now_s, command=packet)
        tracking = self.tracker.update(record, now_s)
        record.update(tracking)
        if self.object_tracker is not None:
            record.update(self.object_tracker.update(record, now_s))
        stamp = record.get("captured_at_s")
        fresh = (_finite(stamp) and 0 <= now_s - stamp < self.controller.limits.max_pose_age_s)
        if tracking["observation_usable"] is not True or not fresh:
            reason = tracking["tracking_frame_reason"] or record.get("reason") or "localization_incomplete"
            if not fresh:
                reason = "stale_observation"
            packet = self._hold(now_s, reason, record)
        else:
            self.last_capture_s = stamp
            self.fresh_streak = min(self.recovery_frames, self.fresh_streak + 1)
            self.controller.set_paused(self.fresh_streak < self.recovery_frames)
            packet = self.controller.tick(record, now_s)
            if self.fresh_streak < self.recovery_frames:
                packet["stop_reason"] = "confirming_observation"
            self.bank.receive(packet, now_s)
            self.command = packet
            self.status = packet["stop_reason"] or packet["status"]
        return self._event(now_s, command=deepcopy(packet), observation=record)

    def close(self, now_s, reason="operator_stop"):
        if not _finite(now_s) or now_s < 0 or (self.now is not None and now_s < self.now):
            raise ValueError("Close time must not precede the runtime clock")
        self.now = now_s
        if self.closed_reason is None:
            self.closed_reason = reason
            # Output first: a future tracker diagnostic error must not delay
            # the local stop latch during shutdown.
            self.controller.emergency_stop()
            self.bank.emergency_stop()
            self.tracker.close(now_s)
            if self.object_tracker is not None:
                self.object_tracker.close(now_s)
            self._hold(now_s, reason, {"status": "source_closed"})
        self.status = "closed"
        return self._event(now_s, command=deepcopy(self.command))
