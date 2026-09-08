"""Real measured-image supervision, independent of image decoding and display.

There is no simulated world or pose integrator. Commands are calculated from
observations and retained only in the existing dry-run actuator endpoint.
"""

from __future__ import annotations

import math
from copy import deepcopy

from .control_loop import ClosedLoopController, ControlLimits, MockActuatorBank
from .mission_bindings import MissionBindings
from .mission_runtime import MissionExecutor
from .observed_obstacles import ObservedObstacleMap
from .runtime_wire import RuntimeWireSupervisor
from .vision.calibration import COORDINATE_SYSTEM
from .vision.object_tracking import ObjectTracker
from .vision.tracking import PoseTracker
from .world_state import ObservationWorldAdapter, PieceSpec


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
                 limits=None, track_objects=False, session_id=None, configuration_id=None,
                 mission_plan=None, fleet=None, wire_fake=False, wire_options=None,
                 binding_plan=None, observation_profile_id=None, mission_observe_only=False,
                 obstacle_plan=None):
        if obstacle_plan is not None and (mission_plan is None or track_objects is not True):
            raise ValueError("Observed object obstacles require a mission and object tracking")
        if (type(mission_observe_only) is not bool or mission_observe_only
                and (mission_plan is None or wire_fake or binding_plan is not None)):
            raise ValueError("Mission observation-only needs a mission and no wire/binding execution")
        self.mission_observe_only = mission_observe_only
        if binding_plan is not None and (mission_plan is None or track_objects is not True):
            raise ValueError("Reviewed bindings require a mission and object tracking")
        if type(wire_fake) is not bool or (wire_options is not None and not wire_fake):
            raise ValueError("Explicit fake-wire mode is required for wire options")
        if wire_fake and mission_plan is None:
            raise ValueError("Fake-wire runtime requires a differential mission plan")
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
        if mission_plan is not None and (goals or radii_mm is not None):
            raise ValueError("Mission plan and explicit goals/radii are mutually exclusive")
        self.mission = (MissionExecutor(mission_plan, fleet, roles=roles,
            field_size_mm=field_size_mm, limits=limits) if mission_plan is not None else None)
        if self.mission and self.mission.pickup_policies and not mission_observe_only and binding_plan is None:
            raise ValueError("Observed-piece pickup execution requires reviewed --bindings")
        drive_model = "differential_body" if self.mission else "mecanum"
        self.controller = ClosedLoopController(roles=roles, goals=goals,
            radii_mm=self.mission.radii if self.mission else radii_mm,
            limits=limits, field_size_mm=field_size_mm, session_id=session_id, drive_model=drive_model)
        self.bank = MockActuatorBank(self.controller.ids, session_id=self.controller.session_id,
                                    watchdog_s=limits.command_ttl_s, limits=limits, drive_model=drive_model)
        self.transport_mode = "fake_wire" if wire_fake else "mock"
        self.wire = (RuntimeWireSupervisor(self.controller.ids, session_id=self.controller.session_id,
                    limits=limits, link_options=wire_options) if wire_fake else None)
        self._wire_fault_handled = False
        if self.mission:
            self.mission.session_id = self.controller.session_id
        self.world_adapter = (ObservationWorldAdapter(roles=roles,
            pieces=[PieceSpec.from_piece(p) for p in self.mission.inventory.values()],
            source_name=source_name, session_id=self.controller.session_id,
            field_size_mm=field_size_mm, is_replay=is_replay, configuration_id=configuration_id,
            max_age_s=limits.max_pose_age_s, recovery_frames=max(3, recovery_frames)) if self.mission else None)
        self.bindings = (MissionBindings(binding_plan,
            pieces=[PieceSpec.from_piece(p) for p in self.mission.inventory.values()],
            field_size_mm=field_size_mm, source_name=source_name, is_replay=is_replay,
            observation_profile_id=observation_profile_id,
            required_piece_ids={self.mission.tasks[e["task_id"]].piece_id for e in self.mission.entries
                if self.mission.inventory[self.mission.tasks[e["task_id"]].piece_id].kind != "cube"})
            if binding_plan is not None else None)
        self.object_obstacles = (ObservedObstacleMap(obstacle_plan, field_size_mm=field_size_mm,
            session_id=self.controller.session_id, max_age_s=limits.max_pose_age_s)
            if obstacle_plan is not None else None)
        if self.mission:
            self.mission.obstacle_map = self.object_obstacles
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
        if self.mission:
            self.mission.interrupt(reason)
        self.controller.set_paused(True)
        packet = self.controller.tick(record, now_s)
        packet["stop_reason"] = reason
        self.bank.receive(packet, now_s)
        if self.wire and self.wire.started and not self.wire.fault and not self.wire.closed:
            self.wire.stop(now_s, reason[:128])
        elif self.wire and not self.wire.closed:
            self.wire.poll(now_s)  # Only stop/idle work can remain on a faulted/closed link.
        self.command, self.status = packet, reason
        return packet

    def _wire_failure(self, now_s):
        """One fleet fault interrupts the mission once; camera recovery cannot rearm."""
        reason = "wire_fault:" + str(self.wire.fault)
        if not self._wire_fault_handled:
            self._wire_fault_handled = True
            self._hold(now_s, reason)
        self.status = reason
        if self.mission.fault:
            return self.close(now_s, self.mission.fault)
        return self._event(now_s, command=self.command)

    def reconnect_wire(self, now_s, *, operator_confirmed=False):
        """Explicit API recovery; CLI users start a new reviewed runtime session.

        A closed session or interrupted manipulation is never resumed here.
        Reconnection invalidates prior observations and needs three NEW frames.
        """
        if self.wire is None or self.closed_reason is not None or self.mission.fault:
            raise ValueError("Only an open, non-faulted mission with fake-wire mode can reconnect")
        if not _finite(now_s) or now_s < 0 or (self.now is not None and now_s < self.now):
            raise ValueError("Reconnect requires the current monotonic runtime clock")
        if operator_confirmed is not True or not self.wire.fault:
            raise ValueError("A latched wire fault and explicit operator confirmation are required")
        if self.world_adapter.poll(now_s).status == "closed":
            raise ValueError("A closed observation source requires a new reviewed runtime session")
        self._hold(now_s, "wire_explicit_reconnect")
        if self.mission.fault:
            raise ValueError("Interrupted manipulation requires a new reviewed mission session")
        self.wire.reconnect(now_s, operator_confirmed=True)
        self.now = now_s
        self._wire_fault_handled = False
        self.last_capture_s = None
        self.world_adapter.invalidate(now_s, reason="wire_reconnect_requires_new_observations")
        self.status = "wire_reconnecting"
        return self._event(now_s, command=self.command)

    def _event(self, now_s, *, command=None, observation=None):
        if self.world_adapter:
            self.mission.world = self.world_adapter.poll(now_s).as_dict()
        return {"schema_version": 1, "event": "runtime_tick",
                "session_id": self.controller.session_id, "tick_sequence": self.tick_sequence,
                "at_s": now_s, "status": self.status, "closed_reason": self.closed_reason,
                "fresh_streak": self.fresh_streak, "recovery_frames": self.recovery_frames,
                "last_observation_age_ms": None if self.last_capture_s is None else
                    max(0.0, (now_s - self.last_capture_s) * 1000),
                "observation": deepcopy(observation), "command": deepcopy(command),
                "object_tracking": self.object_tracker.poll(now_s) if self.object_tracker else None,
                "mission": self.mission.snapshot() if self.mission else None,
                "bindings": self.bindings.snapshot() if self.bindings else None,
                "mission_observe_only": self.mission_observe_only,
                "object_obstacles": (self.object_obstacles.poll(now_s, world_ready=self.mission.world["ready"])
                                     if self.object_obstacles else None),
                "actuator": self.bank.snapshot(), "is_replay": self.is_replay,
                "input_mode": "video_replay" if self.is_replay else "live_camera",
                "output_mode": "dry_run_commands",
                "transport_mode": self.transport_mode,
                "wire": self.wire.snapshot() if self.wire else None,
                "device_io": False, "hardware_ready": False, "motion_permitted": False}

    def advance(self, record, now_s, *, feedback=None):
        if not _finite(now_s) or now_s < 0 or (self.now is not None and now_s < self.now):
            self.close(self.now or 0.0, "invalid_runtime_clock")
            raise ValueError("Runtime clock must be finite, nonnegative and monotonic")
        previous_tick, self.now = self.now, now_s
        self.tick_sequence += 1
        self.bank.tick(now_s)
        if self.closed_reason is not None:
            if self.wire:
                self.wire.poll(now_s)
            return self._event(now_s)
        if previous_tick is not None and now_s - previous_tick > self.max_tick_gap_s + 1e-9:
            return self.close(now_s, "supervisor_deadline_missed")
        if self.mission and not self.mission_observe_only:
            self.mission.poll(now_s)
            if self.mission.fault:
                return self.close(now_s, self.mission.fault)
        if isinstance(record, dict) and record.get("status") == "source_closed":
            return self.close(now_s, "source_closed")
        # Expiry resets recovery even if a new valid record arrives this tick.
        expired = (self.last_capture_s is not None
                   and now_s >= self.last_capture_s + self.controller.limits.max_pose_age_s)
        if expired:
            if self.wire and self.wire.started and not self.wire.fault:
                # Stop BEFORE draining queued motion, including when a new
                # camera frame happens to arrive on the old expiry boundary.
                self.wire.stop(now_s, "observation_watchdog")
                return self._wire_failure(now_s)
            self.fresh_streak = 0
            if self.mission:
                self.mission.interrupt("observation_watchdog")
        if self.wire and self.wire.fault:
            self.wire.poll(now_s)
            if not self._wire_fault_handled:
                return self._wire_failure(now_s)
        if record is None:
            if expired and self.status != "observation_watchdog":
                packet = self._hold(now_s, "observation_watchdog")
                return self._event(now_s, command=packet)
            if self.wire:
                self.wire.poll(now_s)
                if self.wire.fault and not self._wire_fault_handled:
                    return self._wire_failure(now_s)
            return self._event(now_s)
        # Copy at the boundary: neither upstream mutation nor caller edits to
        # a returned diagnostic record may change tracking/controller history.
        record = deepcopy(record)
        rejection = self._input_reason(record)
        if rejection is not None:
            if self.world_adapter:
                self.world_adapter.update(record, now_s, source_session_id=self.controller.session_id)
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
        world = (self.world_adapter.update(record, now_s, source_session_id=self.controller.session_id)
                 if self.world_adapter else None)
        stamp = record.get("captured_at_s")
        fresh = (_finite(stamp) and 0 <= now_s - stamp < self.controller.limits.max_pose_age_s)
        if (tracking["observation_usable"] is not True or not fresh
                or (world is not None and not world.ready and world.status != "confirming")):
            reason = tracking["tracking_frame_reason"] or record.get("reason") or "localization_incomplete"
            if world is not None and not world.ready:
                reason = "mission_world:" + ",".join(world.reasons)
            if not fresh:
                reason = "stale_observation"
            packet = self._hold(now_s, reason, record)
        else:
            if self.object_obstacles:
                self.object_obstacles.update(world, record.get("objects"), now_s)
                if self.object_obstacles.fault and not self.mission_observe_only:
                    return self.close(now_s, self.object_obstacles.fault)
            if self.mission_observe_only:
                # Preflight records the mission catalog + live object tracks
                # without starting a task, assigning goals or arming receivers.
                self.last_capture_s = stamp
                self.fresh_streak = min(self.recovery_frames, self.fresh_streak + 1)
                self.controller.set_paused(True)
                packet = self.controller.tick(record, now_s)
                packet["stop_reason"] = "mission_observation_only"
                self.bank.receive(packet, now_s)
                self.command, self.status = packet, packet["stop_reason"]
                return self._event(now_s, command=packet, observation=record)
            if self.object_obstacles and not self.object_obstacles.ready:
                self.last_capture_s = stamp
                self.fresh_streak = min(self.recovery_frames, self.fresh_streak + 1)
                self.controller.set_paused(True)
                packet = self.controller.tick(record, now_s)
                packet["stop_reason"] = "awaiting_object_obstacles"
                self.bank.receive(packet, now_s)
                self.command, self.status = packet, packet["stop_reason"]
                return self._event(now_s, command=packet, observation=record)
            if self.bindings:
                # Identity checks precede wire.poll(): a queued movement must
                # not reach a receiver on the tick its pickup target is lost.
                world = self.bindings.update(self.world_adapter, now_s)
                if self.bindings.require_pickup(world, self.mission.pickup_piece_id):
                    return self.close(now_s, self.bindings.fault)
                if not self.bindings.activated:
                    self.last_capture_s = stamp
                    self.fresh_streak = min(self.recovery_frames, self.fresh_streak + 1)
                    self.controller.set_paused(True)
                    packet = self.controller.tick(record, now_s)
                    packet["stop_reason"] = "awaiting_piece_bindings"
                    self.bank.receive(packet, now_s)
                    self.command, self.status = packet, packet["stop_reason"]
                    return self._event(now_s, command=packet, observation=record)
            if self.mission:
                self.mission.observe_pickup(world)
                if self.mission.fault:
                    return self.close(now_s, self.mission.fault)
                if world.ready:
                    # New hazards are checked against current measured motion
                    # and the last output BEFORE any delayed drive is delivered.
                    self.mission.guard_command(self.bank.snapshot(), record)
                    if self.wire:
                        poses = {p["robot_id"]: p for p in record["tracks"]}
                        for candidate in self.wire.motion_candidates():
                            rid, speed = candidate["robot_id"], candidate["v_mm_s"]
                            heading = poses[rid]["heading_rad"]
                            # Old body commands act along the CURRENT measured
                            # heading, not the heading at their creation time.
                            self.mission.guard_command({"robots": [{"robot_id": rid,
                                "velocity_world_mm_s": [speed*math.cos(heading), speed*math.sin(heading)]}]}, record)
                    self.mission.recheck_observed_route(record)
                    if self.mission.fault:
                        return self.close(now_s, self.mission.fault)
            if self.wire:
                # Validate current observations and known deadlines BEFORE
                # allowing queued drive delivery; then process ACKs before
                # generating a causally new controller command.
                self.wire.poll(now_s)
                if self.wire.fault:
                    if not self._wire_fault_handled:
                        return self._wire_failure(now_s)
                    self.controller.set_paused(True)
                    packet = self.controller.tick(record, now_s)
                    packet["stop_reason"] = "wire_fault:" + str(self.wire.fault)
                    self.bank.receive(packet, now_s)
                    self.command, self.status = packet, packet["stop_reason"]
                    return self._event(now_s, command=packet, observation=record)
            self.last_capture_s = stamp
            self.fresh_streak = min(self.recovery_frames, self.fresh_streak + 1)
            ready = self.fresh_streak >= self.recovery_frames and (world is None or world.ready)
            started_now = False
            if self.wire and ready and not self.wire.started:
                # Explicit fake-wire mode authorizes this FIRST handshake only.
                # Camera startup cannot spend a receiver's movement lease.
                self.wire.start(now_s)
                started_now = True
            transport_ready = self.wire is None or (self.wire.ready and not started_now)
            self.controller.set_paused(not ready or not transport_ready)
            if self.mission and ready and transport_ready:
                self.controller.set_goals(self.mission.prepare(record, now_s, feedback, world=world))
                if self.mission.fault:
                    return self.close(now_s, self.mission.fault)
            packet = self.controller.tick(record, now_s)
            if not ready:
                packet["stop_reason"] = "confirming_observation"
            if self.mission and ready and transport_ready:
                self.mission.guard_command(packet, record)
                if self.mission.fault:
                    return self.close(now_s, self.mission.fault)
            self.bank.receive(packet, now_s)
            if self.wire and ready and transport_ready:
                self.wire.submit(packet, now_s)
            if self.wire and self.wire.fault:
                return self._wire_failure(now_s)
            if (self.mission and ready and self.bank.reason == "mock_active"
                    and (self.wire is None or self.wire.ready)):
                self.mission.accept_arrival(packet, record, now_s)
                if self.mission.done:
                    return self.close(now_s, "mission_completed")
            self.command = packet
            self.status = ("wire_connecting" if started_now else "wire_awaiting_ack"
                           if self.wire and ready and not transport_ready else packet["stop_reason"] or packet["status"])
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
            if self.wire:
                self.wire.close(now_s, reason[:128])
            if self.mission:
                self.mission.close()
                self.world_adapter.close(now_s)
            self.tracker.close(now_s)
            if self.object_tracker is not None:
                self.object_tracker.close(now_s)
            self._hold(now_s, reason, {"status": "source_closed"})
        self.status = "closed"
        return self._event(now_s, command=deepcopy(self.command))
