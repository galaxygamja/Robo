"""Conservative visible-object collision map, independent of motion and hardware.

Object sizes are reviewed inputs, never inferred from kind names or CAD drafts.
Missing/ambiguous objects latch a stop after activation; the last footprint is
retained for diagnostics, NOT promoted to a fresh location or silently removed.
No pickup/contact/ownership exception can be inferred from an actuator intent.
"""
from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import asdict, dataclass
from types import MappingProxyType

from .mission_bindings import label, number
from .vision.calibration import COORDINATE_SYSTEM


def validate_obstacle_plan(plan, *, field_size_mm):
    fields = {"schema_version", "coordinate_system", "reviewed", "reviewed_by", "evidence",
              "position_margin_mm", "max_route_replans", "footprints"}
    if (not isinstance(plan, dict) or set(plan) != fields
            or type(plan["schema_version"]) is not int or plan["schema_version"] != 1
            or plan["coordinate_system"] != COORDINATE_SYSTEM or plan["reviewed"] is not True
            or not label(plan["reviewed_by"]) or not label(plan["evidence"])
            or not number(plan["position_margin_mm"]) or not 0 <= plan["position_margin_mm"] <= math.hypot(*field_size_mm)
            or type(plan["max_route_replans"]) is not int or not 0 <= plan["max_route_replans"] <= 100):
        raise ValueError("Reviewed object-obstacle schema, margin and route replan limit required")
    footprints = plan["footprints"]
    if not isinstance(footprints, list) or not 1 <= len(footprints) <= 64:
        raise ValueError("Object obstacles need 1..64 explicit kind/colour footprints")
    seen = set()
    for footprint in footprints:
        if (not isinstance(footprint, dict) or set(footprint) != {"kind", "colour", "radius_mm"}
                or not isinstance(footprint["kind"], str) or footprint["kind"] not in {"disc", "cylinder", "cube"}
                or not label(footprint["colour"]) or not number(footprint["radius_mm"])
                or not 0 < footprint["radius_mm"] <= math.hypot(*field_size_mm)
                or (footprint["kind"], footprint["colour"]) in seen):
            raise ValueError("Distinct supported object classes and explicit positive conservative radii required")
        seen.add((footprint["kind"], footprint["colour"]))
    return deepcopy(plan)


@dataclass(frozen=True, slots=True)
class ObjectFootprint:
    track_id: str
    kind: str
    colour: str
    position_mm: tuple[float, float]
    radius_mm: float
    observed_at_s: float
    source_sequence: int
    confirmation_state: str

    def rectangle(self, margin_mm):
        radius = self.radius_mm + margin_mm
        return {"x_mm": self.position_mm[0]-radius, "y_mm": self.position_mm[1]-radius,
                "width_mm": 2*radius, "height_mm": 2*radius}


class ObservedObstacleMap:
    """One pinned source session; at most 64 concurrent remembered footprints."""

    def __init__(self, plan, *, field_size_mm, session_id, max_age_s=.2):
        if (not isinstance(field_size_mm, (list, tuple)) or len(field_size_mm) != 2
                or any(not number(v) or not 0 < v <= 1e6 for v in field_size_mm)):
            raise ValueError("Obstacle map needs positive field dimensions in mm")
        self.plan = validate_obstacle_plan(plan, field_size_mm=field_size_mm)
        if not label(session_id) or not number(max_age_s) or not 0 < max_age_s <= .2:
            raise ValueError("Obstacle map needs source session identity and at most 200 ms expiry")
        self.session_id, self.field_size_mm = session_id, tuple(field_size_mm)
        self.max_age_s = max_age_s
        self._radii = {(e["kind"], e["colour"]): e["radius_mm"] for e in self.plan["footprints"]}
        self._footprints = {}
        self._rectangles = ()
        self._now = self._capture = None
        self._sequence = self.revision = self.route_replans = 0
        self._provenance = None
        self.activated = False
        self._ready = False
        self.reason = "awaiting_object_obstacles"
        self.fault = None

    def _clock(self, now_s):
        if not number(now_s) or now_s < 0 or self._now is not None and now_s < self._now:
            self._reject("object_obstacle_clock_invalid", latch=True)
            raise ValueError("Obstacle map needs a monotonic host clock")
        self._now = float(now_s)

    def _reject(self, reason, *, latch=None):
        self._ready, self.reason = False, reason
        should_latch = self.activated if latch is None else latch
        if should_latch:
            self.fault = self.fault or reason

    def update(self, world, raw_objects, now_s):
        self._clock(now_s)
        if self.fault:
            return self.snapshot()
        if world.session_id != self.session_id or world.field_size_mm != self.field_size_mm:
            self._reject("object_obstacle_source_changed", latch=True)
            return self.snapshot()
        provenance = (world.source_name, world.configuration_id, world.is_replay)
        if self._provenance is not None and provenance != self._provenance:
            self._reject("object_obstacle_source_changed", latch=True)
            return self.snapshot()
        if world.generated_at_s != now_s or not world.ready:
            self._reject("object_obstacle_world_not_ready", latch=False)
            return self.snapshot()
        if (not number(world.captured_at_s) or not 0 <= now_s-world.captured_at_s < self.max_age_s-1e-12
                or type(world.source_sequence) is not int
                or world.source_sequence <= self._sequence):
            self._reject("object_obstacle_frame_not_fresh")
            return self.snapshot()
        self._sequence = world.source_sequence
        self._provenance = provenance
        if not isinstance(raw_objects, list) or len(raw_objects) > 64:
            self._reject("object_obstacle_capacity_or_payload", latch=True)
            return self.snapshot()
        # A tentative raw detection is already a possible collision hazard.
        # It need not qualify as a verified pickup identity to block a path.
        current = {o.object_id: o for o in world.objects if not o.identity_uncertain
                   and o.state in {"tentative", "confirmed"} and o.position_evidence == "vision"
                   and o.observed_at_s == world.captured_at_s}
        # WorldAdapter already cross-checks unique raw candidates. Recheck the
        # correspondence here because raw ambiguous candidates can have NO ID.
        candidates = set()
        try:
            for raw in raw_objects:
                if (not isinstance(raw, dict) or not isinstance(raw.get("kind"), str)
                        or not label(raw.get("color")) or not isinstance(raw.get("center_mm"), (list, tuple))
                        or len(raw["center_mm"]) != 2 or not all(number(v) for v in raw["center_mm"])
                        or any(not 0 <= v <= bound for v, bound in zip(raw["center_mm"], self.field_size_mm))):
                    raise ValueError("Malformed raw object")
                candidates.add((raw["kind"], raw["color"], tuple(raw["center_mm"])))
            if len(candidates) != len(raw_objects):
                raise ValueError("Duplicate raw object")
        except (ValueError, TypeError):
            self._reject("object_obstacle_raw_invalid", latch=True)
            return self.snapshot()
        if any((kind, colour) not in self._radii for kind, colour, _ in candidates):
            self._reject("object_obstacle_unknown_footprint", latch=True)
            return self.snapshot()
        tracked = {(o.kind, o.colour, o.position_mm) for o in current.values()}
        if tracked != candidates or len(current) != len(candidates):
            self._reject("object_obstacle_unconfirmed_or_ambiguous")
            return self.snapshot()
        if self.activated and not set(self._footprints) <= set(current):
            self._reject("object_obstacle_lost")
            return self.snapshot()
        pending = {}
        for oid, obj in current.items():
            previous = self._footprints.get(oid)
            if previous and (obj.kind, obj.colour) != (previous.kind, previous.colour):
                self._reject("object_obstacle_identity_changed", latch=True)
                return self.snapshot()
            pending[oid] = ObjectFootprint(oid, obj.kind, obj.colour, obj.position_mm,
                self._radii[(obj.kind, obj.colour)], obj.observed_at_s, world.source_sequence, obj.state)
        old_geometry = {oid: (o.position_mm, o.radius_mm) for oid, o in self._footprints.items()}
        new_geometry = {oid: (o.position_mm, o.radius_mm) for oid, o in pending.items()}
        self.revision += not self.activated or old_geometry != new_geometry
        self._footprints, self._capture = pending, world.captured_at_s
        self._rectangles = tuple(MappingProxyType(obj.rectangle(self.plan["position_margin_mm"]))
                                 for obj in pending.values())
        self.activated, self._ready, self.reason = True, True, None
        return self.snapshot()

    @property
    def ready(self):
        return (self._ready and not self.fault and self._capture is not None
                and 0 <= self._now-self._capture < self.max_age_s-1e-12)

    @property
    def rectangles(self):
        """Historical rectangles remain present on fault; readiness is separate."""
        return self._rectangles

    def request_replan(self):
        if self.fault:
            return False
        if self.route_replans >= self.plan["max_route_replans"]:
            self._reject("object_obstacle_replan_budget_exceeded", latch=True)
            return False
        self.route_replans += 1
        return True

    def poll(self, now_s, *, world_ready=True):
        self._clock(now_s)
        if not world_ready:
            self._ready, self.reason = False, "object_obstacle_world_not_ready"
        elif self._capture is not None and not self.ready and not self.fault:
            self._ready, self.reason = False, "object_obstacle_observation_expired"
        return self.snapshot()

    def snapshot(self):
        return {"schema_version": 1, "mode": "observed_object_obstacles", "session_id": self.session_id,
                "field_size_mm": self.field_size_mm, "max_age_s": self.max_age_s,
                "source_name": self._provenance[0] if self._provenance else None,
                "configuration_id": self._provenance[1] if self._provenance else None,
                "is_replay": self._provenance[2] if self._provenance else None,
                "status": "fault" if self.fault else "ready" if self.ready else "waiting",
                "ready": self.ready, "activated": self.activated, "fault": self.fault,
                "reason": self.fault or self.reason, "at_s": self._now,
                "captured_at_s": self._capture, "source_sequence": self._sequence, "revision": self.revision,
                "route_replans": self.route_replans, "objects": [asdict(o) for o in self._footprints.values()],
                "position_margin_mm": self.plan["position_margin_mm"],
                "unobserved_is_free": False, "contact_exemptions": [], "device_io": False,
                "motion_permitted": False, "physical_clearance_verified": False, "field_coverage_verified": False}
