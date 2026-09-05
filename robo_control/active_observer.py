"""Active downward-looking viewpoint planning and conservative source selection.

Pure stdlib, no flight SDK or device commands. Geometry is a planning model:
ideal pinhole FOV, horizontal targets and finite-height axis-aligned boxes.
Calibration metadata is checked, NOT estimated here. Every coordinate/frame and
timestamp must already be expressed in the agreed field/host-clock domain.
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, replace
from itertools import combinations

from .fleet import robot_id_valid


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _coordinates(values, count):
    return (isinstance(values, (tuple, list)) and len(values) == count
            and all(_finite(v) and abs(v) <= 1e6 for v in values))


def _wrap(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class DronePose:
    x_mm: float
    y_mm: float
    z_mm: float
    yaw_rad: float = 0.

    def __post_init__(self):
        if not _coordinates((self.x_mm, self.y_mm, self.z_mm, self.yaw_rad), 4):
            raise ValueError("Finite bounded drone pose required")

    @property
    def point(self):
        return self.x_mm, self.y_mm, self.z_mm


@dataclass(frozen=True)
class ObservationTarget:
    target_id: str
    kind: str
    position_mm: tuple[float, float, float]
    weight: float = 1.
    missing: bool = False
    age_s: float = 0.
    task_state: str = "idle"
    marker_size_mm: float = 45.
    uncertainty_mm: float = 5.
    fallback_visible: bool = False

    def __post_init__(self):
        if (not isinstance(self.target_id, str) or not self.target_id or len(self.target_id) > 64
                or self.kind not in {"robot", "object", "drop_zone"}
                or not _coordinates(self.position_mm, 3)
                or self.task_state not in {"idle", "navigate", "pickup", "carry", "drop_verification"}
                or type(self.missing) is not bool or type(self.fallback_visible) is not bool
                or not _finite(self.weight) or not 0 < self.weight <= 100
                or not _finite(self.age_s) or not 0 <= self.age_s <= 1e6
                or not _finite(self.marker_size_mm) or not 0 < self.marker_size_mm <= 1000
                or not _finite(self.uncertainty_mm) or not 0 <= self.uncertainty_mm <= 1e5):
            raise ValueError("Invalid target/need hint")
        object.__setattr__(self, "position_mm", tuple(self.position_mm))

    @property
    def priority(self):
        task_bonus = {"idle": 0., "navigate": .3, "pickup": 1., "carry": .7, "drop_verification": 2.}
        need = 1. + 3. * self.missing + min(2., self.age_s * 4.) + task_bonus[self.task_state]
        # Additional camera coverage is less valuable when the fallback already
        # sees a target. It still has some value as an independent viewpoint.
        return self.weight * need * (.2 if self.fallback_visible and not self.missing else 1.)


@dataclass(frozen=True)
class Occluder:
    occluder_id: str
    min_mm: tuple[float, float, float]
    max_mm: tuple[float, float, float]

    def __post_init__(self):
        if (not isinstance(self.occluder_id, str) or not self.occluder_id
                or not _coordinates(self.min_mm, 3) or not _coordinates(self.max_mm, 3)
                or any(a >= b for a, b in zip(self.min_mm, self.max_mm))):
            raise ValueError("Occluder requires finite nonempty 3D box")
        object.__setattr__(self, "min_mm", tuple(self.min_mm))
        object.__setattr__(self, "max_mm", tuple(self.max_mm))


@dataclass(frozen=True)
class CameraModel:
    horizontal_fov_rad: float = math.radians(70.)
    vertical_fov_rad: float = math.radians(60.)
    image_size_px: tuple[int, int] = (1280, 720)
    min_marker_pixels: float = 18.

    def __post_init__(self):
        if (any(not _finite(v) or not 0.05 < v < math.pi - .05 for v in
                (self.horizontal_fov_rad, self.vertical_fov_rad))
                or not isinstance(self.image_size_px, (list, tuple)) or len(self.image_size_px) != 2
                or any(type(v) is not int or not 32 <= v <= 16384 for v in self.image_size_px)
                or not _finite(self.min_marker_pixels) or not 1 <= self.min_marker_pixels <= 1000):
            raise ValueError("Invalid pinhole/FOV model")
        object.__setattr__(self, "image_size_px", tuple(self.image_size_px))


@dataclass(frozen=True)
class FlightBounds:
    min_mm: tuple[float, float, float] = (100., 100., 650.)
    max_mm: tuple[float, float, float] = (1043., 1081., 1100.)
    horizontal_speed_mm_s: float = 300.
    vertical_speed_mm_s: float = 150.
    yaw_speed_rad_s: float = 1.
    body_clearance_mm: float = 60.

    def __post_init__(self):
        if (not _coordinates(self.min_mm, 3) or not _coordinates(self.max_mm, 3)
                or any(a >= b for a, b in zip(self.min_mm, self.max_mm))
                or self.min_mm[2] <= 0
                or any(not _finite(v) or not 0 < v <= 1e4 for v in
                    (self.horizontal_speed_mm_s, self.vertical_speed_mm_s, self.yaw_speed_rad_s, self.body_clearance_mm))):
            raise ValueError("Invalid model flight envelope")
        object.__setattr__(self, "min_mm", tuple(self.min_mm))
        object.__setattr__(self, "max_mm", tuple(self.max_mm))

    def contains(self, pose):
        return all(a <= p <= b for a, p, b in zip(self.min_mm, pose.point, self.max_mm))


def ray_box_intersects(start, end, box: Occluder, *, padding_mm=0., include_endpoints=False):
    """3D segment/slab test; a tag on a box top is not hidden by that endpoint.

    Interior tangency is conservatively occluded. Planning flight corridors use
    padded boxes and include endpoints, unlike camera-to-target sight rays.
    """
    if not _coordinates(start, 3) or not _coordinates(end, 3) or not _finite(padding_mm) or not 0 <= padding_mm <= 1e5:
        raise ValueError("Invalid 3D ray or padding")
    low, high = (0., 1.) if include_endpoints else (1e-8, 1. - 1e-8)
    for axis in range(3):
        lower, upper = box.min_mm[axis] - padding_mm, box.max_mm[axis] + padding_mm
        direction = end[axis] - start[axis]
        if abs(direction) < 1e-12:
            if start[axis] < lower or start[axis] > upper:
                return False
        else:
            a, b = (lower - start[axis]) / direction, (upper - start[axis]) / direction
            low, high = max(low, min(a, b)), min(high, max(a, b))
            if low > high:
                return False
    return low <= high


def observe_geometry(pose: DronePose, target: ObservationTarget, occluders=(), camera=None):
    """Expected visibility, not a camera detection or physical confidence value."""
    camera = camera or CameraModel()
    dz = pose.z_mm - target.position_mm[2]
    result = {"target_id": target.target_id, "visible": False, "reason": None,
              "projected_marker_pixels": 0., "occluder_id": None}
    if dz <= 0:
        return {**result, "reason": "not_below_camera"}
    dx, dy = target.position_mm[0] - pose.x_mm, target.position_mm[1] - pose.y_mm
    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    half_x, half_y = dz * math.tan(camera.horizontal_fov_rad / 2), dz * math.tan(camera.vertical_fov_rad / 2)
    radius = target.marker_size_mm * math.sqrt(2.) / 2 + target.uncertainty_mm
    pixels = min(camera.image_size_px[0] * target.marker_size_mm / (2 * half_x),
                 camera.image_size_px[1] * target.marker_size_mm / (2 * half_y))
    result["projected_marker_pixels"] = pixels
    if abs(local_x) + radius > half_x or abs(local_y) + radius > half_y:
        return {**result, "reason": "outside_fov_or_uncertainty_margin"}
    if pixels < camera.min_marker_pixels:
        return {**result, "reason": "insufficient_pixel_footprint"}
    # Centre plus square corners conservatively catch partial occlusion. This is
    # still a finite sample model, not a proof that every image pixel is clear.
    footprint = target.marker_size_mm / 2 + target.uncertainty_mm
    rays = [target.position_mm, *[
        (target.position_mm[0] + ox * footprint, target.position_mm[1] + oy * footprint, target.position_mm[2])
        for ox, oy in ((-1, -1), (-1, 1), (1, -1), (1, 1))]]
    for box in occluders:
        if any(ray_box_intersects(pose.point, point, box) for point in rays):
            return {**result, "reason": "occluded", "occluder_id": box.occluder_id}
    return {**result, "visible": True, "reason": "expected_visible"}


class ActiveObserverPlanner:
    """Choose among a bounded list, penalize travel, and avoid viewpoint chatter."""
    def __init__(self, candidates, *, bounds=None, camera=None, switch_margin=.2,
                 min_dwell_s=.4, travel_cost_per_m=.4, altitude_cost_per_m=.2):
        candidates = tuple(candidates)
        if not candidates or len(candidates) > 128 or not all(isinstance(p, DronePose) for p in candidates):
            raise ValueError("Supply 1..128 candidate viewpoints")
        if len(set(candidates)) != len(candidates):
            raise ValueError("Duplicate candidate viewpoints")
        if any(not _finite(v) or not 0 <= v <= 100 for v in
               (switch_margin, min_dwell_s, travel_cost_per_m, altitude_cost_per_m)):
            raise ValueError("Invalid selection costs")
        self.bounds, self.camera = bounds or FlightBounds(), camera or CameraModel()
        if any(not self.bounds.contains(p) for p in candidates):
            raise ValueError("A candidate lies outside the configured flight envelope")
        self.candidates = candidates
        self.switch_margin, self.min_dwell_s = switch_margin, min_dwell_s
        self.travel_cost_per_m, self.altitude_cost_per_m = travel_cost_per_m, altitude_cost_per_m
        self.selected = None
        self.last_switch_s = -math.inf
        self.now = None

    def plan(self, current: DronePose, targets, occluders=(), *, now_s):
        if not _finite(now_s) or (self.now is not None and now_s < self.now):
            raise ValueError("Planner time must be finite and monotonic")
        targets, occluders = tuple(targets), tuple(occluders)
        if (len(targets) > 256 or len(occluders) > 128
                or not all(isinstance(t, ObservationTarget) for t in targets)
                or not all(isinstance(b, Occluder) for b in occluders)
                or len({t.target_id for t in targets}) != len(targets)):
            raise ValueError("Bounded unique targets and occluders required")
        dt = .1 if self.now is None else min(.5, now_s - self.now)
        self.now = now_s
        base = {"mode": "active_viewpoint_planning_only", "physical_commands": False,
                "device_io": False, "flight_sdk_implemented": False, "dynamic_calibration_implemented": False,
                "calibration_required_per_frame": True, "target_positions_are_hints": True}
        if not targets:
            # Dwell is anti-chatter, not authority to keep pursuing a removed
            # mission. Discard the incumbent before any dwell comparison.
            self.selected, self.last_switch_s = None, -math.inf
            return {**base, "status": "hold", "reason": "no_targets",
                    "selected_pose": asdict(current), "next_pose": asdict(current),
                    "expected_visible_ids": [], "current_expected_visible_ids": [],
                    "weighted_coverage": 0., "movement_cost": 0., "net_benefit_over_current": 0., "candidates": []}
        if not self.bounds.contains(current):
            return {**base, "status": "hold", "reason": "current_pose_out_of_bounds",
                    "selected_pose": asdict(current), "next_pose": asdict(current), "candidates": []}
        evaluations = []
        for pose in (current, *[p for p in self.candidates if p != current]):
            if any(ray_box_intersects(current.point, pose.point, b,
                padding_mm=self.bounds.body_clearance_mm, include_endpoints=True) for b in occluders):
                continue
            views = [observe_geometry(pose, t, occluders, self.camera) for t in targets]
            coverage = sum(t.priority * min(1., view["projected_marker_pixels"] / (2 * self.camera.min_marker_pixels))
                           for t, view in zip(targets, views) if view["visible"])
            horizontal = math.hypot(pose.x_mm - current.x_mm, pose.y_mm - current.y_mm)
            cost = self.travel_cost_per_m * horizontal / 1000 + self.altitude_cost_per_m * abs(pose.z_mm - current.z_mm) / 1000
            evaluations.append({"pose": pose, "views": views, "coverage": coverage, "cost": cost, "score": coverage - cost})
        if not evaluations:
            return {**base, "status": "hold", "reason": "no_clear_bounded_flight_corridor",
                    "selected_pose": asdict(current), "next_pose": asdict(current), "candidates": []}
        best = max(evaluations, key=lambda e: (e["score"], -e["cost"]))
        current_view = next((e for e in evaluations if e["pose"] == current), None)
        previous = next((e for e in evaluations if e["pose"] == self.selected), None)
        reason = "weighted_information_gain"
        incumbent = previous or current_view
        if incumbent is not None and best["pose"] != incumbent["pose"]:
            if previous is not None and now_s - self.last_switch_s < self.min_dwell_s:
                best, reason = previous, "minimum_dwell"
            elif best["score"] - incumbent["score"] <= self.switch_margin:
                best, reason = incumbent, "hysteresis"
        destination = best["pose"]
        if self.selected != destination:
            self.selected, self.last_switch_s = destination, now_s
        dx, dy = destination.x_mm - current.x_mm, destination.y_mm - current.y_mm
        distance = math.hypot(dx, dy)
        fraction = min(1., self.bounds.horizontal_speed_mm_s * dt / max(distance, 1e-12))
        dz = max(-self.bounds.vertical_speed_mm_s * dt, min(self.bounds.vertical_speed_mm_s * dt, destination.z_mm - current.z_mm))
        yaw_delta = max(-self.bounds.yaw_speed_rad_s * dt,
                        min(self.bounds.yaw_speed_rad_s * dt, _wrap(destination.yaw_rad - current.yaw_rad)))
        next_pose = DronePose(current.x_mm + dx * fraction, current.y_mm + dy * fraction,
                              current.z_mm + dz, _wrap(current.yaw_rad + yaw_delta))
        # Horizontal and vertical speed limiting need not remain on the original
        # straight candidate segment. Recheck the actual proposed short segment.
        if any(ray_box_intersects(current.point, next_pose.point, b,
            padding_mm=self.bounds.body_clearance_mm, include_endpoints=True) for b in occluders):
            next_pose, reason = current, "next_step_corridor_blocked"
        return {**base, "status": "hold" if next_pose == current else "planned_move", "reason": reason,
                "selected_pose": asdict(destination), "next_pose": asdict(next_pose),
                "expected_visible_ids": [v["target_id"] for v in best["views"] if v["visible"]],
                "current_expected_visible_ids": [v["target_id"] for v in (current_view or {"views": []})["views"] if v["visible"]],
                "weighted_coverage": best["coverage"], "movement_cost": best["cost"],
                "net_benefit_over_current": best["score"] - (current_view["score"] if current_view else 0.),
                "candidates": [{**e, "pose": asdict(e["pose"])} for e in evaluations]}


def calibration_gate(frame, *, image_size_px=(1280, 720), field_frame_id="arena-mm", moving=False,
                     max_reprojection_error_mm=6., max_pose_error_mm=15.):
    """Check metadata tied to THIS frame, never compute or certify calibration."""
    if not isinstance(frame, dict):
        return "malformed_frame"
    metadata = frame.get("calibration")
    if not isinstance(metadata, dict):
        return "missing_calibration"
    if (frame.get("image_size_px") != list(image_size_px)
            or metadata.get("image_size_px") != list(image_size_px)):
        return "calibration_resolution_mismatch"
    if metadata.get("field_frame_id") != field_frame_id:
        return "wrong_field_coordinate_frame"
    if (metadata.get("frame_sequence") != frame.get("sequence")
            or type(metadata.get("frame_sequence")) is not int
            or not _finite(metadata.get("captured_at_s"))
            or metadata.get("captured_at_s") != frame.get("captured_at_s")):
        return "calibration_not_for_this_frame"
    if (metadata.get("valid") is not True or metadata.get("reference_geometry_valid") is not True
            or metadata.get("target_height_model_valid") is not True
            or type(metadata.get("reference_count")) is not int or metadata["reference_count"] < 4):
        return "invalid_calibration_references"
    if moving and metadata.get("dynamic_reference_update") is not True:
        return "moving_camera_requires_dynamic_calibration"
    reprojection, error = metadata.get("reprojection_error_mm"), metadata.get("pose_error_bound_mm")
    if (not _finite(reprojection) or not 0 <= reprojection <= max_reprojection_error_mm
            or not _finite(error) or not 0 <= error <= max_pose_error_mm):
        return "calibration_error_bound"
    return None


class MultiSourcePoseSelector:
    """Pick, do not average, one fresh usable observation per registered robot.

    Missing sources cannot resurrect an older selected pose. All source times
    must already share a clock domain; this module does not synchronize clocks.
    Source sessions are explicitly pinned/restarted, not implicitly accepted.
    """
    def __init__(self, robot_ids, *, source_sessions, moving_sources=(), source_image_sizes=None,
                 field_size_mm=(1143., 1181.), field_frame_id="arena-mm", clock_domain="observer-host",
                 max_age_s=.25, min_confidence=.6, max_error_mm=15., max_displacement_speed_mm_s=1500.):
        ids = tuple(robot_ids)
        moving_sources = tuple(moving_sources)
        if not ids or not all(robot_id_valid(r) for r in ids) or len(set(ids)) != len(ids):
            raise ValueError("Unique registered robot IDs required")
        if (not isinstance(source_sessions, dict) or not 1 <= len(source_sessions) <= 16
                or any(not isinstance(k, str) or not k or not isinstance(v, str) or not v for k, v in source_sessions.items())):
            raise ValueError("Pin 1..16 source session identities")
        if (not set(moving_sources) <= set(source_sessions) or not _coordinates(field_size_mm, 2)
                or any(v <= 0 for v in field_size_mm) or not isinstance(field_frame_id, str) or not field_frame_id
                or not isinstance(clock_domain, str) or not clock_domain
                or not _finite(max_age_s) or not 0 < max_age_s <= 5.
                or not _finite(min_confidence) or not 0 <= min_confidence <= 1.
                or not _finite(max_error_mm) or not 0 < max_error_mm <= 1000
                or not _finite(max_displacement_speed_mm_s) or not 0 <= max_displacement_speed_mm_s <= 5000.):
            raise ValueError("Invalid source selection limits")
        sizes = source_image_sizes or {source: (1280, 720) for source in source_sessions}
        if set(sizes) != set(source_sessions) or any(not isinstance(size, (tuple, list)) or len(size) != 2
            or any(type(v) is not int or not 32 <= v <= 16384 for v in size) for size in sizes.values()):
            raise ValueError("Every source needs its expected image resolution")
        self.ids, self.sessions = ids, dict(source_sessions)
        self.retired_sessions = {source: set() for source in self.sessions}
        self.moving_sources, self.image_sizes = frozenset(moving_sources), {k: tuple(v) for k, v in sizes.items()}
        self.field_size_mm, self.field_frame_id, self.clock_domain = tuple(field_size_mm), field_frame_id, clock_domain
        self.max_age_s, self.min_confidence, self.max_error_mm = max_age_s, min_confidence, max_error_mm
        self.max_displacement_speed_mm_s = float(max_displacement_speed_mm_s)
        self.states = {source: {"sequence": 0, "stamp": -math.inf, "closed": False, "poses": {}, "reason": "no_frame"} for source in self.sessions}
        self.newest = {rid: -math.inf for rid in ids}
        self.now = -math.inf

    def _clock(self, now_s):
        if not _finite(now_s) or now_s < self.now:
            raise ValueError("Selection clock must be finite and monotonic")
        self.now = now_s

    def restart_source(self, source_id, new_session_id, *, now_s):
        self._clock(now_s)
        if (source_id not in self.sessions or not isinstance(new_session_id, str) or not new_session_id
                or new_session_id == self.sessions[source_id] or new_session_id in self.retired_sessions[source_id]):
            raise ValueError("Restart requires a new never-used session for a registered source")
        self.retired_sessions[source_id].add(self.sessions[source_id])
        self.sessions[source_id] = new_session_id
        self.states[source_id] = {"sequence": 0, "stamp": -math.inf, "closed": False, "poses": {}, "reason": "restarted_awaiting_frame"}

    def ingest(self, frame, now_s):
        self._clock(now_s)
        if not isinstance(frame, dict):
            return {**self.snapshot(now_s), "ingest_rejection": "malformed_frame"}
        source = frame.get("source_id")
        if not isinstance(source, str) or source not in self.sessions:
            return {**self.snapshot(now_s), "ingest_rejection": "unknown_source"}
        state = self.states[source]
        if frame.get("session_id") != self.sessions[source]:
            return {**self.snapshot(now_s), "ingest_rejection": "wrong_source_session"}
        if state["closed"]:
            return {**self.snapshot(now_s), "ingest_rejection": "source_session_closed"}
        if frame.get("status") == "source_closed":
            state.update(closed=True, poses={}, reason="source_session_closed")
            return self.snapshot(now_s)
        sequence = frame.get("sequence")
        if type(sequence) is not int or sequence <= state["sequence"]:
            return {**self.snapshot(now_s), "ingest_rejection": "out_of_order_source_frame"}
        # New envelopes invalidate older source results even if their body or
        # calibration is unusable; duplicate/old envelopes never replace them.
        state.update(sequence=sequence, poses={})
        captured, received = frame.get("captured_at_s"), frame.get("received_at_s")
        reason = None
        if frame.get("clock_domain") != self.clock_domain:
            reason = "wrong_clock_domain"
        elif (not _finite(captured) or not _finite(received) or not state["stamp"] <= captured <= received <= now_s
                or now_s - captured >= self.max_age_s):
            reason = "stale_or_invalid_timestamp"
        elif frame.get("status") != "detected":
            state["stamp"] = captured
            reason = "source_frame_rejected"
        else:
            state["stamp"] = captured
            reason = calibration_gate(frame, image_size_px=self.image_sizes[source], field_frame_id=self.field_frame_id,
                                      moving=source in self.moving_sources, max_pose_error_mm=self.max_error_mm)
        if reason:
            state["reason"] = reason
            return {**self.snapshot(now_s), "ingest_rejection": reason}
        observations = frame.get("robots")
        if not isinstance(observations, list) or len(observations) > len(self.ids):
            state["reason"] = "malformed_robot_observations"
            return {**self.snapshot(now_s), "ingest_rejection": state["reason"]}
        poses = {}
        for row in observations:
            if not isinstance(row, dict) or not isinstance(row.get("robot_id"), str) or row["robot_id"] not in self.ids or row["robot_id"] in poses:
                reason = "unknown_or_duplicate_robot"
                break
            rid = row["robot_id"]
            point, heading, confidence, error = (row.get(key) for key in
                ("robot_center_mm", "heading_rad", "confidence", "error_bound_mm"))
            if (not _coordinates(point, 2) or not _finite(heading) or abs(heading) > 1e6
                    or any(not 0 <= p <= bound for p, bound in zip(point, self.field_size_mm))
                    or not _finite(confidence) or not self.min_confidence <= confidence <= 1.
                    or not _finite(error) or not 0 <= error <= self.max_error_mm
                    or error < frame["calibration"]["pose_error_bound_mm"]):
                reason = "invalid_pose_or_quality"
                break
            poses[rid] = {"robot_id": rid, "robot_center_mm": list(point), "heading_rad": _wrap(heading),
                          "confidence": confidence, "error_bound_mm": error, "captured_at_s": captured,
                          "source_id": source, "session_id": self.sessions[source], "source_sequence": sequence}
        if reason:
            state["reason"] = reason
            return {**self.snapshot(now_s), "ingest_rejection": reason}
        state.update(poses=poses, reason="accepted")
        for rid in poses:
            self.newest[rid] = max(self.newest[rid], captured)
        return self.snapshot(now_s)

    def snapshot(self, now_s):
        self._clock(now_s)
        selected, conflicts = [], []
        for rid in self.ids:
            fresh = [s["poses"][rid] for s in self.states.values() if not s["closed"] and rid in s["poses"]
                     and 0 <= now_s - s["poses"][rid]["captured_at_s"] < self.max_age_s]
            disagreements = []
            # Compare before the no-rewind filter: a slightly older but still
            # fresh source can reveal a coordinate-frame/calibration conflict.
            # Allow both reported error bounds and possible displacement during
            # the capture-time difference; never average inconsistent readings.
            for first, second in combinations(fresh, 2):
                elapsed = abs(first["captured_at_s"] - second["captured_at_s"])
                distance = math.dist(first["robot_center_mm"], second["robot_center_mm"])
                allowance = (first["error_bound_mm"] + second["error_bound_mm"]
                             + self.max_displacement_speed_mm_s * elapsed)
                if distance > allowance + 1e-8:
                    disagreements.append({"source_ids": [first["source_id"], second["source_id"]],
                        "distance_mm": distance, "allowed_distance_mm": allowance,
                        "capture_time_difference_s": elapsed})
            if disagreements:
                conflicts.append({"robot_id": rid, "reason": "cross_source_position_disagreement", "pairs": disagreements})
                continue
            options = [pose for pose in fresh if pose["captured_at_s"] >= self.newest[rid]]
            if options:
                best = min(options, key=lambda p: (-p["captured_at_s"], p["error_bound_mm"], -p["confidence"], p["source_id"]))
                selected.append({**best, "robot_center_mm": list(best["robot_center_mm"]),
                                 "age_ms": (now_s - best["captured_at_s"]) * 1000})
        present = {p["robot_id"] for p in selected}
        missing = [rid for rid in self.ids if rid not in present]
        return {"mode": "one_source_per_robot_selection", "selected_poses": selected,
                "selected_sources": {p["robot_id"]: p["source_id"] for p in selected}, "missing_robot_ids": missing,
                "observation_conflicts": conflicts,
                "observation_usable": not missing, "stop_required": bool(missing),
                "source_status": {source: {"session_id": self.sessions[source], "sequence": s["sequence"],
                    "closed": s["closed"], "reason": s["reason"],
                    "age_ms": None if not math.isfinite(s["stamp"]) else (now_s - s["stamp"]) * 1000}
                    for source, s in self.states.items()},
                "device_io": False, "hardware_ready": False, "motion_permitted": False,
                "dynamic_calibration_implemented": False}


def _demo_frame(source, sequence, now_s, targets, visible_ids, *, dynamic=False, reference_count=4):
    """Ideal synthetic calibration/observations, never real image evidence."""
    return {"source_id": source, "session_id": source + "-demo", "sequence": sequence,
            "captured_at_s": now_s, "received_at_s": now_s, "clock_domain": "observer-host",
            "status": "detected", "image_size_px": [1280, 720], "synthetic": True,
            "calibration": {"valid": True, "frame_sequence": sequence, "captured_at_s": now_s,
                "image_size_px": [1280, 720], "field_frame_id": "arena-mm", "reference_count": reference_count,
                "reference_geometry_valid": reference_count >= 4, "target_height_model_valid": True,
                "dynamic_reference_update": dynamic, "reprojection_error_mm": 1., "pose_error_bound_mm": 3.},
            "robots": [{"robot_id": t.target_id, "robot_center_mm": list(t.position_mm[:2]), "heading_rad": 0.,
                        "confidence": .95, "error_bound_mm": 4.} for t in targets
                       if t.kind == "robot" and t.target_id in visible_ids]}


def run_demo():
    """Same fallback dropout/3D occluder, parked vs active: observation recovery only."""
    robots = [ObservationTarget(rid, "robot", point, missing=rid == "H1", task_state="carry" if rid == "H1" else "navigate",
                                fallback_visible=rid != "H1") for rid, point in (
        ("H1", (750., 350., 30.)), ("H2", (800., 650., 30.)),
        ("B1", (300., 350., 30.)), ("B2", (300., 700., 30.)))]
    target = ObservationTarget("drop-check", "drop_zone", (750., 350., 20.), weight=1.5,
                               missing=True, task_state="drop_verification", marker_size_mm=30.)
    parked = DronePose(300., 350., 800.)
    candidates = (parked, DronePose(800., 600., 900.), DronePose(650., 850., 1000.))
    boxes = (Occluder("tall-bin", (550., 300., 0.), (650., 450., 600.)),)
    # Explicit non-collinear reference-marker hints in this fixture. Count
    # their visibility from every moving pose; do not give the drone reference
    # credit merely because the simulated image still has the same resolution.
    references = [ObservationTarget(f"reference-{i}", "object", (x, y, 0.), marker_size_mm=30.)
                  for i, (x, y) in enumerate(((450., 600.), (800., 600.), (450., 750.), (800., 750.)))]
    output = {}
    for mode in ("parked", "active"):
        pose = parked
        planner = ActiveObserverPlanner(candidates)
        selector = MultiSourcePoseSelector([t.target_id for t in robots],
            source_sessions={"fallback": "fallback-demo", "drone": "drone-demo"}, moving_sources=("drone",))
        trace, complete_frames, recovered_at = [], 0, None
        for step in range(60):
            now = round(1. + step * .1, 8)
            needs = [replace(t, age_s=step * .1 if t.missing else 0.) for t in (*robots, target)]
            decision = planner.plan(pose, needs, boxes, now_s=now) if mode == "active" else None
            if decision:
                pose = DronePose(**decision["next_pose"])
            visible = [t.target_id for t in robots if observe_geometry(pose, t, boxes)["visible"]]
            reference_count = sum(observe_geometry(pose, t, boxes)["visible"] for t in references)
            fallback = [t.target_id for t in robots if t.target_id != "H1"]
            selector.ingest(_demo_frame("fallback", step + 1, now, robots, fallback), now)
            selected = selector.ingest(_demo_frame("drone", step + 1, now, robots, visible,
                                       dynamic=True, reference_count=reference_count), now)
            if selected["observation_usable"]:
                complete_frames += 1
                if recovered_at is None:
                    recovered_at = round(step * .1, 3)
            if step % 5 == 0 or (recovered_at is not None and recovered_at == round(step * .1, 3)):
                trace.append({"elapsed_s": round(step * .1, 3), "drone_pose": asdict(pose), "drone_visible_ids": visible,
                              "visible_reference_count": reference_count, "selected_sources": selected["selected_sources"],
                              "stop_required": selected["stop_required"]})
        output[mode] = {"complete_observation_frames": complete_frames, "sampled_frames": 60,
                        "first_complete_after_dropout_s": recovered_at, "trace": trace,
                        "after_all_sources_stop": selector.snapshot(7.2)["stop_required"]}
    return {"mode": "synthetic_active_viewpoint_comparison", "same_fallback_dropout": "H1 absent for all 60 frames",
            "same_occluders": [asdict(box) for box in boxes], "results": output,
            "synthetic_reference_points_mm": [list(t.position_mm) for t in references],
            "notice": "Ideal visibility/calibration metadata, not camera trials, flight control, route completion, score or hardware validation.",
            "device_io": False, "hardware_ready": False, "flight_sdk_implemented": False,
            "dynamic_calibration_implemented": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", required=True)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args(argv)
    print(json.dumps(run_demo(), ensure_ascii=False, allow_nan=False, indent=None if args.compact else 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
