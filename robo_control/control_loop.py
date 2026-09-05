"""Measured-pose closed-loop rehearsal with an independently expiring mock actuator.

Coordinates are millimetres, +X is heading zero, and positive rotation is CCW.
There is deliberately no socket, serial, GPIO or physical motor implementation.
The collision envelope and braking limits are assumptions, NOT hardware safety
certification. The demo uses a spacious synthetic test floor, not the arena.
"""
from __future__ import annotations

import argparse
import json
import math
import uuid
from collections.abc import Mapping
from dataclasses import dataclass
from itertools import combinations

from .fleet import validate_roles


def _number(value) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _angle(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def _norm_limit(x: float, y: float, limit: float) -> tuple[float, float]:
    scale = min(1.0, limit / max(math.hypot(x, y), 1e-12))
    return x * scale, y * scale


@dataclass(frozen=True)
class ControlLimits:
    max_speed_mm_s: float = 180.0
    max_turn_rad_s: float = 1.5
    acceleration_mm_s2: float = 400.0
    angular_acceleration_rad_s2: float = 3.0
    braking_mm_s2: float = 400.0
    position_gain_s: float = 1.5
    heading_gain_s: float = 2.0
    position_tolerance_mm: float = 3.0
    heading_tolerance_rad: float = 0.03
    max_pose_age_s: float = 0.2
    command_ttl_s: float = 0.3
    prediction_horizon_s: float = 0.4
    clearance_mm: float = 15.0
    wheel_radius_mm: float = 20.0
    wheel_half_length_plus_width_mm: float = 115.0

    def __post_init__(self):
        if any(not _number(v) or not 1e-9 <= v <= 1e6 for v in vars(self).values()):
            raise ValueError("Control limits must be finite and in the numerical range [1e-9, 1e6]")
        if self.command_ttl_s > 0.3:
            raise ValueError("Mock commands expire no later than 300 ms")


def mecanum_wheels(vx_mm_s: float, vy_mm_s: float, omega_rad_s: float,
                   heading_rad: float, limits: ControlLimits | None = None) -> list[float]:
    """Ideal X-layout wheel speeds [front-left, front-right, rear-left, rear-right].

    Robot +X is forward, +Y is left. Physical roller arrangement, motor polarity
    and encoder signs must be verified separately before any hardware adapter.
    """
    if not all(_number(v) for v in (vx_mm_s, vy_mm_s, omega_rad_s, heading_rad)):
        raise ValueError("Finite velocity and heading required")
    limits = limits or ControlLimits()
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    forward, left = c * vx_mm_s + s * vy_mm_s, -s * vx_mm_s + c * vy_mm_s
    turn = limits.wheel_half_length_plus_width_mm * omega_rad_s
    r = limits.wheel_radius_mm
    return [(forward - left - turn) / r, (forward + left + turn) / r,
            (forward + left - turn) / r, (forward - left + turn) / r]


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _hull(points):
    points = sorted(set(points))
    if len(points) <= 1:
        return points
    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _point_segment(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy)
                       / max(dx * dx + dy * dy, 1e-18)))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def _inside(point, polygon):
    return len(polygon) >= 3 and all(_cross(polygon[i], polygon[(i + 1) % len(polygon)], point) >= -1e-9
                                     for i in range(len(polygon)))


def _polygon_distance(a, b):
    if _inside(a[0], b) or _inside(b[0], a):
        return 0.0
    result = math.inf
    for i, p in enumerate(a):
        q = a[(i + 1) % len(a)]
        for j, r in enumerate(b):
            s = b[(j + 1) % len(b)]
            crosses = (_cross(p, q, r), _cross(p, q, s), _cross(r, s, p), _cross(r, s, q))
            if crosses[0] * crosses[1] < 0 and crosses[2] * crosses[3] < 0:
                return 0.0
            result = min(result, _point_segment(p, r, s), _point_segment(q, r, s),
                         _point_segment(r, p, q), _point_segment(s, p, q))
    return result


def predicted_conflicts(poses: dict, commands: dict, radii_mm: dict,
                        limits: ControlLimits | None = None) -> list[dict]:
    """Conservative swept circular envelopes, including reaction and braking.

    Each centre sweep is the convex hull of current, measured-velocity and
    proposed-velocity endpoints over the horizon. Pairwise distance ignores
    arrival time (deliberately conservative). Inflate each hull by its complete
    body/arm/load radius + v*command_TTL + v²/(2*assumed_braking). This detects
    risk; it is not a route planner, dynamic-model proof or deadlock resolver.
    """
    limits = limits or ControlLimits()
    if set(poses) != set(commands) or set(poses) != set(radii_mm):
        raise ValueError("Prediction requires the same complete fleet in every input")
    sweeps, envelopes = {}, {}
    for rid, pose in poses.items():
        position, measured = pose["robot_center_mm"], pose["velocity_mm_s"]
        commanded = commands[rid][:2]
        radius = radii_mm[rid]
        if (len(position) != 2 or len(measured) != 2 or len(commanded) != 2
                or any(not _number(v) or abs(v) > 1e6 for v in (*position, *measured, *commanded, radius)) or radius <= 0):
            raise ValueError("Prediction requires finite positions, velocities and positive radii")
        speed = max(math.hypot(*measured), math.hypot(*commanded))
        sweeps[rid] = _hull([tuple(position), *[
            (position[0] + v[0] * limits.prediction_horizon_s,
             position[1] + v[1] * limits.prediction_horizon_s) for v in (measured, commanded)]])
        envelopes[rid] = radius + speed * limits.command_ttl_s + speed * speed / (2 * limits.braking_mm_s2)
    result = []
    for first, second in combinations(sorted(poses), 2):
        distance = _polygon_distance(sweeps[first], sweeps[second])
        required = envelopes[first] + envelopes[second] + limits.clearance_mm
        if distance <= required:
            result.append({"robot_ids": [first, second], "predicted_clearance_mm": distance - required,
                           "reason": "swept_or_braking_envelope"})
    return result


class ClosedLoopController:
    """A bounded proportional measured-pose controller, exclusively for mock IO."""
    def __init__(self, *, roles=None, goals=None, session_id=None, radii_mm=None,
                 limits: ControlLimits | None = None, enable_hardware=False):
        if enable_hardware is not False:
            raise ValueError("Hardware output is not implemented and cannot be enabled")
        self.roles = validate_roles(roles)
        self.ids = tuple(self.roles)
        self.limits = limits or ControlLimits()
        self.session_id = session_id or uuid.uuid4().hex
        if not isinstance(self.session_id, str) or not self.session_id or len(self.session_id) > 128:
            raise ValueError("A nonempty process session ID is required")
        self.radii_mm = dict(radii_mm) if radii_mm is not None else {rid: 150.0 for rid in self.ids}
        if set(self.radii_mm) != set(self.ids) or any(not _number(v) or not 0 < v <= 1e6 for v in self.radii_mm.values()):
            raise ValueError("Every robot requires a positive complete body/arm/load envelope radius")
        self.goals = {}
        self.set_goals(goals or {})
        self.sequence = 0
        self.source = None
        self.frame_sequence = 0
        self.frame_stamp = -math.inf
        self.localization_closed = False
        self.now = -math.inf
        self.paused = False
        self.estop_latched = False
        self.previous = {rid: (0., 0., 0.) for rid in self.ids}

    def set_goals(self, goals):
        if not isinstance(goals, Mapping) or not set(goals) <= set(self.ids):
            raise ValueError("Goals must name registered robots")
        validated = {}
        for rid, goal in goals.items():
            if (not isinstance(goal, Mapping) or not {"x_mm", "y_mm"} <= set(goal)
                    or not set(goal) <= {"x_mm", "y_mm", "heading_rad"}
                    or not all(_number(v) and abs(v) <= 1e6 for v in goal.values())):
                raise ValueError("A goal needs finite x_mm, y_mm and optional heading_rad")
            validated[rid] = dict(goal)
        self.goals = validated

    def set_paused(self, paused=True):
        if type(paused) is not bool:
            raise ValueError("paused must be Boolean")
        self.paused = paused

    def emergency_stop(self):
        self.estop_latched = True

    def reset_emergency_stop(self):
        """Explicit operator reset; the next command still needs a fresh valid frame."""
        self.estop_latched = False
        self.previous = {rid: (0., 0., 0.) for rid in self.ids}

    def _poses(self, record, now_s):
        if self.localization_closed:
            return None, "localization_session_closed"
        if not isinstance(record, dict):
            return None, "malformed_record"
        if (record.get("status") == "source_closed" or record.get("tracking_session_closed") is True
                or record.get("localization_session_closed") is True):
            self.localization_closed = True
            return None, "localization_session_closed"
        seq, stamp, source = record.get("sequence", record.get("frame_sequence")), record.get("captured_at_s"), record.get("source_name")
        if not isinstance(source, str) or not source or (self.source is not None and source != self.source):
            return None, "source_changed"
        if type(seq) is not int or seq <= self.frame_sequence:
            return None, "out_of_order_frame"
        # Coarse monotonic clocks may repeat a timestamp, but the frame sequence
        # above must still strictly advance. Never infer extra elapsed time.
        if not _number(stamp) or not self.frame_stamp <= stamp <= now_s:
            return None, "invalid_frame_timestamp"
        # Consume every well-ordered envelope, even when its observations fail.
        self.source, self.frame_sequence, self.frame_stamp = source, seq, stamp
        if now_s - stamp > self.limits.max_pose_age_s:
            return None, "stale_frame"
        if record.get("observation_usable") is not True or record.get("stop_required") is not False:
            return None, "localization_incomplete"
        tracks = record.get("tracks")
        if not isinstance(tracks, list) or len(tracks) != len(self.ids):
            return None, "incomplete_tracks"
        poses = {}
        for track in tracks:
            if not isinstance(track, dict):
                return None, "malformed_track"
            rid = track.get("robot_id")
            if not isinstance(rid, str) or rid not in self.roles or rid in poses:
                return None, "unknown_or_duplicate_robot"
            position, heading, observed = track.get("robot_center_mm"), track.get("heading_rad"), track.get("observed_at_s")
            velocity, turn = track.get("velocity_mm_s"), track.get("angular_velocity_rad_s")
            if (not isinstance(position, (list, tuple)) or len(position) != 2
                    or not isinstance(velocity, (list, tuple)) or len(velocity) != 2
                    or not all(_number(v) for v in (*position, heading, observed, *velocity, turn))
                    or any(abs(v) > 1e6 for v in (*position, heading, *velocity, turn))):
                return None, "malformed_pose"
            if (track.get("state") != "observed" or track.get("valid_for_control") is not True
                    or observed != stamp or not 0 <= now_s - observed <= self.limits.max_pose_age_s):
                return None, "stale_or_missing_pose"
            poses[rid] = track
        return poses, None

    def tick(self, record, now_s: float) -> dict:
        if not _number(now_s) or now_s < self.now:
            raise ValueError("Controller clock must be finite and monotonic")
        dt = min(0.1, now_s - self.now) if math.isfinite(self.now) else 0.02
        self.now = now_s
        poses, reason = self._poses(record, now_s)
        proposals, reached = {}, {}
        limits = self.limits
        for rid in self.ids:
            goal, pose = self.goals.get(rid), poses.get(rid) if poses else None
            vx = vy = omega = 0.
            at_goal = goal is None
            if goal is not None and pose is not None:
                dx, dy = goal["x_mm"] - pose["robot_center_mm"][0], goal["y_mm"] - pose["robot_center_mm"][1]
                error = _angle(goal.get("heading_rad", pose["heading_rad"]) - pose["heading_rad"])
                position_done = math.hypot(dx, dy) <= limits.position_tolerance_mm
                heading_done = abs(error) <= limits.heading_tolerance_rad
                at_goal = position_done and heading_done
                if not position_done:
                    vx, vy = _norm_limit(dx * limits.position_gain_s, dy * limits.position_gain_s, limits.max_speed_mm_s)
                if not heading_done:
                    omega = max(-limits.max_turn_rad_s, min(limits.max_turn_rad_s, error * limits.heading_gain_s))
                previous = self.previous[rid]
                ax, ay = _norm_limit(vx - previous[0], vy - previous[1], limits.acceleration_mm_s2 * dt)
                vx, vy = previous[0] + ax, previous[1] + ay
                omega = previous[2] + max(-limits.angular_acceleration_rad_s2 * dt,
                                         min(limits.angular_acceleration_rad_s2 * dt, omega - previous[2]))
            proposals[rid] = (vx, vy, omega)
            reached[rid] = at_goal
        conflicts = predicted_conflicts(poses, proposals, self.radii_mm, limits) if poses else []
        reason = "emergency_stop" if self.estop_latched else "paused" if self.paused else reason
        reason = reason or ("predicted_collision" if conflicts else None)
        # A robot which has reached its tolerance must remain still, including
        # removal of any ramp residual. There is no integral wind-up.
        if reason:
            proposals = {rid: (0., 0., 0.) for rid in self.ids}
        else:
            proposals = {rid: (0., 0., 0.) if reached[rid] else value for rid, value in proposals.items()}
        self.previous = proposals
        self.sequence += 1
        commands = []
        for rid in self.ids:
            vx, vy, omega = proposals[rid]
            heading = poses[rid]["heading_rad"] if poses else 0.
            commands.append({"robot_id": rid, "velocity_world_mm_s": [vx, vy],
                             "angular_velocity_rad_s": omega, "pose_heading_rad": heading,
                             "wheel_velocity_rad_s": mecanum_wheels(vx, vy, omega, heading, limits),
                             "at_goal": reached[rid] if poses else False})
        return {"schema_version": 1, "session_id": self.session_id, "sequence": self.sequence,
                "issued_at_s": now_s, "ttl_s": limits.command_ttl_s,
                "status": "hold" if reason else "at_goal" if all(reached.values()) else "tracking_goal",
                "stop_reason": reason, "conflicts": conflicts, "emergency_stop": self.estop_latched,
                "localization_session_closed": self.localization_closed,
                "robots": commands, "device_io": False, "hardware_ready": False,
                "motion_permitted": False, "mock_motion_permitted": reason is None}


class MockActuatorBank:
    """In-memory actuator endpoint; tick its watchdog independently of control.

    A stopped/crashed controller cannot call its own watchdog. A real firmware
    adapter must run an independent clock/watchdog. This class models that
    boundary only when its consumer continues calling tick(). Same-process
    monotonic time is required; no cross-host clock synchronization is implied.
    """
    def __init__(self, robot_ids, *, session_id, watchdog_s=0.3, enable_hardware=False,
                 limits: ControlLimits | None = None):
        if enable_hardware is not False:
            raise ValueError("Only mock actuation exists")
        ids = tuple(robot_ids)
        if len(ids) != len(set(ids)):
            raise ValueError("Duplicate mock actuator ID")
        self.ids = tuple(validate_roles({rid: "beaver" for rid in ids}))
        if not isinstance(session_id, str) or not session_id:
            raise ValueError("Pin the expected controller process session before receiving commands")
        if not _number(watchdog_s) or not 0 < watchdog_s <= 0.3:
            raise ValueError("Independent watchdog must be in (0, 0.3] seconds")
        self.session_id, self.watchdog_s = session_id, watchdog_s
        self.limits = limits or ControlLimits()
        self.sequence = 0
        self.now = -math.inf
        self.issued_at_s = None
        self.expires_at_s = -math.inf
        self.estop_latched = False
        self.reason = "no_command"
        self.commands = self._zeros()

    def _zeros(self):
        return {rid: {"robot_id": rid, "velocity_world_mm_s": [0., 0.],
                      "angular_velocity_rad_s": 0., "wheel_velocity_rad_s": [0.] * 4} for rid in self.ids}

    def _time(self, now_s):
        if not _number(now_s) or now_s < self.now:
            self.commands, self.reason = self._zeros(), "invalid_actuator_clock"
            raise ValueError("Actuator clock must be finite and monotonic")
        self.now = now_s

    def emergency_stop(self):
        self.estop_latched = True
        self.commands, self.reason = self._zeros(), "emergency_stop"

    def reset_emergency_stop(self):
        """Does not replay the previous command; a new sequence is required."""
        self.estop_latched = False
        self.commands, self.reason = self._zeros(), "reset_requires_new_command"
        self.expires_at_s = -math.inf

    def _reject(self, reason):
        self.commands, self.reason = self._zeros(), reason
        self.expires_at_s = -math.inf
        return self.snapshot()

    def receive(self, packet, now_s):
        self._time(now_s)
        if not isinstance(packet, dict):
            return self._reject("malformed_command")
        if (type(packet.get("schema_version")) is not int or packet["schema_version"] != 1
                or type(packet.get("emergency_stop")) is not bool
                or type(packet.get("mock_motion_permitted")) is not bool):
            return self._reject("malformed_command_envelope")
        if packet.get("session_id") != self.session_id:
            return self._reject("wrong_session")
        seq, issued, ttl = packet.get("sequence"), packet.get("issued_at_s"), packet.get("ttl_s")
        if type(seq) is not int or seq <= self.sequence:
            return self._reject("out_of_order_command")
        if (not _number(issued) or issued > now_s or (self.issued_at_s is not None and issued < self.issued_at_s)
                or not _number(ttl) or not 0 < ttl <= self.watchdog_s or now_s - issued >= ttl):
            return self._reject("stale_or_invalid_command_time")
        # A valid envelope sequence is consumed even if its payload is rejected.
        self.sequence, self.issued_at_s = seq, issued
        if any(packet.get(flag) is not False for flag in ("device_io", "hardware_ready", "motion_permitted")):
            return self._reject("hardware_output_forbidden")
        if packet.get("emergency_stop") is True:
            self.emergency_stop()
        commands = packet.get("robots")
        if not isinstance(commands, list) or len(commands) != len(self.ids):
            return self._reject("incomplete_command_fleet")
        accepted = {}
        for command in commands:
            if not isinstance(command, dict):
                return self._reject("malformed_command")
            rid = command.get("robot_id")
            if not isinstance(rid, str) or rid not in self.ids or rid in accepted:
                return self._reject("unknown_or_duplicate_command_robot")
            velocity, omega, wheels, heading = (command.get(key) for key in
                ("velocity_world_mm_s", "angular_velocity_rad_s", "wheel_velocity_rad_s", "pose_heading_rad"))
            if (not isinstance(velocity, (list, tuple)) or len(velocity) != 2
                    or not isinstance(wheels, (list, tuple)) or len(wheels) != 4
                    or not all(_number(v) for v in (*velocity, omega, *wheels, heading))):
                return self._reject("malformed_command_velocity")
            if math.hypot(*velocity) > self.limits.max_speed_mm_s + 1e-8 or abs(omega) > self.limits.max_turn_rad_s + 1e-8:
                return self._reject("command_speed_limit")
            expected = mecanum_wheels(*velocity, omega, heading, self.limits)
            if any(abs(actual - wanted) > 1e-8 for actual, wanted in zip(wheels, expected)):
                return self._reject("wheel_kinematics_mismatch")
            accepted[rid] = {**command, "velocity_world_mm_s": list(velocity), "wheel_velocity_rad_s": list(wheels)}
        if self.estop_latched:
            return self._reject("emergency_stop")
        if packet.get("mock_motion_permitted") is not True or packet.get("stop_reason") is not None:
            return self._reject(packet.get("stop_reason") or "controller_hold")
        self.commands = accepted
        self.expires_at_s = issued + min(ttl, self.watchdog_s)
        self.reason = "mock_active"
        return self.snapshot()

    def tick(self, now_s):
        self._time(now_s)
        if self.estop_latched:
            self.commands, self.reason = self._zeros(), "emergency_stop"
        elif now_s >= self.expires_at_s:
            self.commands, self.reason = self._zeros(), "command_watchdog"
        return self.snapshot()

    def snapshot(self):
        return {"session_id": self.session_id, "sequence": self.sequence, "reason": self.reason,
                "emergency_stop": self.estop_latched,
                "robots": [{**c, "velocity_world_mm_s": list(c["velocity_world_mm_s"]),
                            "wheel_velocity_rad_s": list(c["wheel_velocity_rad_s"])} for c in self.commands.values()],
                "device_io": False, "hardware_ready": False, "motion_permitted": False}


def run_demo(robot_count=4, *, steps=400, dt=0.02) -> dict:
    """Reproducible feedback rehearsal, not a competition route/time prediction."""
    from .vision.tracking import PoseTracker
    if type(robot_count) is not int or not 1 <= robot_count <= 32:
        raise ValueError("Demo supports 1..32 robots on a spacious synthetic floor")
    if type(steps) is not int or steps <= 0 or not _number(dt) or not 0 < dt <= 0.1:
        raise ValueError("Positive demo steps and dt <= 0.1 are required")
    ids = list(validate_roles())
    ids = (ids + [f"extra{i + 1}" for i in range(max(0, robot_count - len(ids)))])[:robot_count]
    roles = {rid: "hamster" if rid == "H1" else "beaver" for rid in ids}
    poses = {rid: [100., 100. + i * 800., 0.] for i, rid in enumerate(ids)}
    goals = {rid: {"x_mm": 500., "y_mm": p[1] + 100., "heading_rad": 0.25} for rid, p in poses.items()}
    controller = ClosedLoopController(roles=roles, goals=goals, session_id="deterministic-mock-demo")
    bank = MockActuatorBank(ids, session_id=controller.session_id)
    tracker = PoseTracker(ids)
    samples = []
    packet = None
    for step in range(steps):
        now = round(1. + step * dt, 9)
        raw = {"status": "detected", "source_name": "synthetic-closed-loop-session", "sequence": step + 1,
               "captured_at_s": now, "is_replay": False, "hardware_verified": False, "tag_size_mm": None,
               "observation_complete": True, "unknown_tag_ids": [], "duplicate_tag_ids": [],
               "robots": [{"robot_id": rid, "robot_center_mm": pose[:2], "heading_rad": pose[2]} for rid, pose in poses.items()]}
        measured = {**raw, **tracker.update(raw, now)}
        packet = controller.tick(measured, now)
        bank.receive(packet, now)
        applied = bank.tick(now)
        for command in applied["robots"]:
            p = poses[command["robot_id"]]
            p[0] += command["velocity_world_mm_s"][0] * dt
            p[1] += command["velocity_world_mm_s"][1] * dt
            p[2] = _angle(p[2] + command["angular_velocity_rad_s"] * dt)
        if step % 25 == 0 or packet["status"] == "at_goal":
            samples.append({"elapsed_s": round(step * dt, 3), "status": packet["status"],
                            "stop_reason": packet["stop_reason"], "poses": {rid: list(p) for rid, p in poses.items()}})
        if packet["status"] == "at_goal":
            break
    watchdog = bank.tick(now + 0.301)
    return {"mode": "mock_measured_pose_closed_loop", "device_io": False, "hardware_ready": False,
            "motion_permitted": False, "robot_count": robot_count, "all_at_goal": packet["status"] == "at_goal",
            "steps": step + 1, "elapsed_s": round((step + 1) * dt, 3),
            "notice": "Spacious synthetic test floor, ideal wheels and poses; NOT arena routing or hardware validation.",
            "goals": goals, "final_poses": poses, "samples": samples, "last_command": packet,
            "after_command_loss": watchdog}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", required=True, help="run the deterministic in-memory feedback rehearsal")
    parser.add_argument("--robots", type=int, default=4)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args(argv)
    print(json.dumps(run_demo(args.robots), ensure_ascii=False, allow_nan=False, indent=None if args.compact else 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
