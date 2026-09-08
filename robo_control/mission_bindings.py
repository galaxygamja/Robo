"""Reviewed startup regions -> session-local object IDs, never physical identity.

Regions describe the operator's identification constraints, not measurements.
Every selected region must contain exactly ONE fresh, confirmed track. A batch
is bound once; loss, colour mismatch or manual replacement never triggers a
nearest-object fallback. No file access or human input occurs in the supervisor.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
from copy import deepcopy
from pathlib import Path

from .vision.calibration import COORDINATE_SYSTEM


def number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def label(value):
    return isinstance(value, str) and bool(value.strip()) and len(value) <= 200


def profile_identity(calibration, tags, colors):
    """Stable identity of normalized observation configuration, not authentication."""
    data = {"calibration": calibration, "tags": tags, "colors": colors}
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False,
                                    separators=(",", ":")).encode("utf-8")).hexdigest()


def load_binding_plan(path):
    """Startup-only bounded JSON read; reject duplicate and nonfinite values."""
    return load_review_json(path, name="Binding")


def load_review_json(path, *, name="Review"):
    """Shared bounded startup-file boundary for explicit operator review plans."""
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ValueError(f"Duplicate {name} JSON key: " + key)
            result[key] = value
        return result

    def invalid(value):
        raise ValueError(f"Nonfinite {name} JSON number: " + value)

    def finite(value):
        result = float(value)
        return result if math.isfinite(result) else invalid(value)

    with Path(path).open("rb") as handle:
        payload = handle.read(65537)
    if len(payload) > 65536:
        raise ValueError(f"{name} plan exceeds 64 KiB")
    try:
        result = json.loads(payload.decode("utf-8-sig"), object_pairs_hook=pairs,
                            parse_constant=invalid, parse_float=finite)
    except (UnicodeError, RecursionError) as exc:
        raise ValueError(f"{name} plan must be bounded UTF-8 JSON") from exc
    if not isinstance(result, dict):
        raise ValueError(f"{name} plan must be a JSON object")  # noqa: TRY004 - external JSON schema error
    stack = [(result, 0)]
    while stack:
        item, depth = stack.pop()
        if depth > 8:
            raise ValueError(f"{name} JSON nesting exceeds eight levels")
        if isinstance(item, dict):
            stack.extend((v, depth+1) for v in item.values())
        elif isinstance(item, list):
            stack.extend((v, depth+1) for v in item)
    return result


def _region(value, bounds):
    keys = ("x_mm", "y_mm", "width_mm", "height_mm")
    if (not isinstance(value, dict) or set(value) != set(keys)
            or any(not number(value[k]) for k in keys)):
        raise ValueError("Binding region needs finite x/y/width/height in mm")
    x, y, w, h = (float(value[k]) for k in keys)
    if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > bounds[0] or y+h > bounds[1]:
        raise ValueError("Binding region must be positive and within the calibrated field")
    return x, y, w, h


def validate_binding_plan(plan, *, pieces, field_size_mm, source_name, is_replay,
                          observation_profile_id, required_piece_ids):
    """Validate and detach an explicit, reviewed startup map from caller data."""
    required = {"schema_version", "coordinate_system", "field_size_mm", "source_name", "is_replay",
                "observation_profile_id", "reviewed", "reviewed_by", "evidence", "bindings"}
    if (not isinstance(plan, dict) or set(plan) != required
            or type(plan["schema_version"]) is not int or plan["schema_version"] != 1
            or plan["coordinate_system"] != COORDINATE_SYSTEM or plan["reviewed"] is not True
            or not label(plan["reviewed_by"]) or not label(plan["evidence"])):
        raise ValueError("Explicitly reviewed binding schema 1, reviewer and evidence required")
    dimensions = plan["field_size_mm"]
    if (not isinstance(dimensions, (list, tuple)) or len(dimensions) != 2
            or any(not number(v) or v <= 0 for v in dimensions)
            or tuple(dimensions) != tuple(field_size_mm)
            or not label(source_name) or plan["source_name"] != source_name
            or type(is_replay) is not bool or plan["is_replay"] is not is_replay
            or not isinstance(observation_profile_id, str)
            or re.fullmatch(r"[0-9a-f]{64}", observation_profile_id) is None
            or plan["observation_profile_id"] != observation_profile_id):
        raise ValueError("Binding plan must match source, replay mode, field and observation profile")
    entries = plan["bindings"]
    if not isinstance(entries, list) or not 1 <= len(entries) <= 64:
        raise ValueError("Binding plan needs 1..64 explicit regions")
    catalog = {p.piece_id: p for p in pieces}
    seen, regions = set(), []
    for entry in entries:
        if (not isinstance(entry, dict) or set(entry) != {"piece_id", "kind", "colour", "region_mm"}
                or not isinstance(entry["piece_id"], str) or entry["piece_id"] not in catalog
                or entry["piece_id"] in seen or not label(entry["colour"])):
            raise ValueError("Unique known mission piece, kind, colour and region required")
        spec = catalog[entry["piece_id"]]
        if entry["kind"] != spec.kind or spec.colour not in (None, entry["colour"]):
            raise ValueError("Reviewed kind/colour contradicts the mission piece catalog")
        region = _region(entry["region_mm"], field_size_mm)
        x, y, w, h = region
        for a, b, c, d in regions:
            if x <= a+c and a <= x+w and y <= b+d and b <= y+h:
                raise ValueError("Binding regions must not overlap or share a boundary")
        regions.append(region)
        seen.add(entry["piece_id"])
    if not set(required_piece_ids) <= seen:
        raise ValueError("Every selected non-cube pickup task needs a reviewed binding region")
    return deepcopy(plan)


class MissionBindings:
    """One startup batch owned by one runtime; only current pickup gates motion."""

    def __init__(self, plan, **context):
        self.plan = validate_binding_plan(plan, **context)
        self._mappings = {}
        self._session = None
        self._applied_sequence = None
        self._candidates = []
        self.reason = "awaiting_binding_observation"
        self.fault = None

    @property
    def activated(self):
        return self._applied_sequence is not None

    def update(self, adapter, now_s):
        world = adapter.poll(now_s)
        self._session = self._session or world.session_id
        if self._session != world.session_id:
            self.fault = "binding_session_changed"
        if self.fault or self.activated or not world.ready:
            return world
        pending, candidates = [], []
        for entry in self.plan["bindings"]:
            x, y, w, h = _region(entry["region_mm"], world.field_size_mm)
            # Include tentative, ambiguous and recently missing tracks. Colour
            # must not hide a second plausible occupant of an identity region.
            occupants = [o for o in world.objects if o.position_mm is not None
                         and x <= o.position_mm[0] <= x+w and y <= o.position_mm[1] <= y+h
                         and o.state != "lost"]
            obj = occupants[0] if len(occupants) == 1 else None
            reason = ("empty_region" if not occupants else "multiple_region_occupants" if obj is None
                      else "kind_colour_mismatch" if (obj.kind, obj.colour) != (entry["kind"], entry["colour"])
                      else "track_not_pickable" if not obj.valid_for_pick else None)
            candidates.append({"piece_id": entry["piece_id"],
                               "candidate_track_ids": [o.object_id for o in occupants], "reason": reason})
            if reason is None:
                pending.append((entry["piece_id"], obj.object_id,
                                "reviewed startup region: " + self.plan["reviewed_by"][:170]))
        self._candidates = candidates
        if any(row["reason"] for row in candidates):
            self.reason = "awaiting_unique_pickable_regions"
            return world
        try:
            world = adapter.bind_pieces(pending, now_s)
        except ValueError:
            self.fault = "binding_batch_rejected"
            return adapter.poll(now_s)
        self._mappings = {pid: oid for pid, oid, _ in pending}
        self._applied_sequence = world.source_sequence
        self.reason = None
        return world

    def require_pickup(self, world, piece_id):
        """Latch loss before pickup; never use it as grip/release sensor proof."""
        if self.fault or not self.activated or piece_id is None or not world.ready:
            return self.fault
        piece = world.piece(piece_id)
        if (piece is None or piece.track_id != self._mappings.get(piece_id)
                or not piece.valid_for_pick):
            self.fault = "pickup_binding_lost:" + piece_id
        return self.fault

    def snapshot(self):
        return {"schema_version": 1, "mode": "reviewed_startup_regions",
                "status": "fault" if self.fault else "bound" if self.activated else "waiting",
                "reason": self.fault or self.reason, "session_id": self._session,
                "applied_sequence": self._applied_sequence, "mappings": dict(self._mappings),
                "candidates": deepcopy(self._candidates), "reviewed_by": self.plan["reviewed_by"],
                "evidence": self.plan["evidence"], "physical_identity_verified": False,
                "device_io": False, "motion_permitted": False}
