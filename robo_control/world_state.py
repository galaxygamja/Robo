"""Tracked observations -> immutable mission inputs, never actions or scores.

Only one explicitly identified source session/host clock is accepted. Scenario
coordinates and actuator intentions are never promoted to measurements.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, replace

from .fleet import robot_id_valid, validate_roles
from .vision.calibration import COORDINATE_SYSTEM


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _label(value):
    return isinstance(value, str) and bool(value.strip()) and len(value) <= 200


def _point(value, bounds=None):
    if (not isinstance(value, (list, tuple)) or len(value) != 2 or not all(map(_number, value))
            or (bounds is not None and any(not 0 <= v <= b for v, b in zip(value, bounds)))):
        raise ValueError("invalid_position")
    return tuple(float(v) for v in value)


def _angle(value):
    if not _number(value):
        raise ValueError("invalid_heading")
    return (float(value) + math.pi) % (2 * math.pi) - math.pi


def _rows(values, key, maximum):
    if not isinstance(values, (tuple, list)) or len(values) > maximum:
        raise ValueError("invalid_observation_list")
    result = {}
    for value in values:
        if not isinstance(value, dict) or not _label(value.get(key)) or value[key] in result:
            raise ValueError("duplicate_or_invalid_observation_id")
        result[value[key]] = value
    return result


@dataclass(frozen=True, slots=True)
class PieceSpec:
    piece_id: str
    kind: str
    colour: str | None = None

    def __post_init__(self):
        if (not robot_id_valid(self.piece_id) or self.kind not in {"disc", "cube", "cylinder"}
                or (self.colour is not None and not _label(self.colour))
                or (self.kind == "cylinder" and self.colour not in {"red", "yellow", "green"})):
            raise ValueError("Piece catalog requires ID, supported kind and cylinder colour")

    @classmethod
    def from_piece(cls, piece):
        """Copy semantics only, NOT scenario coordinates/ownership/released defaults."""
        return cls(piece.id, piece.kind, piece.colour)


@dataclass(frozen=True, slots=True)
class ObservedRobot:
    robot_id: str
    role: str
    position_mm: tuple[float, float] | None = None
    heading_rad: float | None = None
    velocity_mm_s: tuple[float, float] | None = None
    observed_at_s: float | None = None
    state: str = "missing"
    valid_for_control: bool = False
    reason: str | None = "not_observed"

    @property
    def position_m(self):
        return None if self.position_mm is None else tuple(v / 1000 for v in self.position_mm)


@dataclass(frozen=True, slots=True)
class ObservedObject:
    object_id: str
    kind: str
    colour: str
    position_mm: tuple[float, float] | None
    observed_at_s: float | None
    state: str
    identity_uncertain: bool
    owner_robot_id: str | None
    lifecycle: str
    position_evidence: str
    position_valid: bool
    valid_for_pick: bool
    reason: str | None


@dataclass(frozen=True, slots=True)
class MissionPieceState:
    piece_id: str
    kind: str
    colour: str | None
    track_id: str | None = None
    position_mm: tuple[float, float] | None = None
    # Current HSV pipeline does not measure cube orientation. Never invent 0.
    yaw_rad: float | None = None
    owner_robot_id: str | None = None
    lifecycle: str = "unknown"
    observed_at_s: float | None = None
    position_valid: bool = False
    valid_for_pick: bool = False
    reason: str | None = "unbound_piece"
    binding_evidence: str | None = None


@dataclass(frozen=True, slots=True)
class WorldState:
    session_id: str
    source_name: str
    configuration_id: str | None
    is_replay: bool
    generated_at_s: float
    source_sequence: int
    captured_at_s: float | None
    field_size_mm: tuple[float, float]
    ready: bool
    status: str
    reasons: tuple[str, ...]
    fresh_streak: int
    recovery_frames: int
    robots: tuple[ObservedRobot, ...]
    objects: tuple[ObservedObject, ...]
    pieces: tuple[MissionPieceState, ...]

    def robot(self, robot_id):
        return next((r for r in self.robots if r.robot_id == robot_id), None)

    def piece(self, piece_id):
        return next((p for p in self.pieces if p.piece_id == piece_id), None)

    def as_dict(self):
        return {"schema_version": 1, "coordinate_system": COORDINATE_SYSTEM,
                "device_io": False, "hardware_ready": False, "motion_permitted": False,
                "physical_identity_verified": False, **asdict(self)}


class ObservationWorldAdapter:
    """A mission observation boundary, not a tracker/controller replacement.

    Call update/poll every consumer tick. Saved snapshots never update themselves.
    Source-session IDs are explicit caller provenance, not security authentication.
    """

    def __init__(self, *, roles, pieces=(), source_name, session_id, field_size_mm,
                 is_replay=False, configuration_id=None, max_age_s=.2, recovery_frames=3):
        self._roles = validate_roles(roles)
        if (not _label(source_name) or not _label(session_id)
                or (configuration_id is not None and not _label(configuration_id))
                or type(is_replay) is not bool):
            raise ValueError("Pin source, session, replay mode and optional configuration identity")
        self._bounds = _point(field_size_mm)
        if any(v <= 0 or v > 1e6 for v in self._bounds):
            raise ValueError("Positive field dimensions in mm required")
        if not _number(max_age_s) or not 0 < max_age_s <= .2:
            raise ValueError("Mission observations expire within 200 ms")
        if type(recovery_frames) is not int or not 3 <= recovery_frames <= 100:
            raise ValueError("At least three fresh frames required")
        catalog = tuple(pieces)
        if (len(catalog) > 512 or any(not isinstance(p, PieceSpec) for p in catalog)
                or len({p.piece_id for p in catalog}) != len(catalog)):
            raise ValueError("Unique PieceSpec catalog, at most 512 entries, required")
        self._catalog = {p.piece_id: p for p in catalog}
        self._source, self._session, self._configuration = source_name, session_id, configuration_id
        self._replay, self._max_age, self._recovery = is_replay, float(max_age_s), recovery_frames
        self._robots = {rid: ObservedRobot(rid, role) for rid, role in self._roles.items()}
        self._objects = {}
        self._bindings, self._broken_bindings = {}, {}
        self._now, self._capture, self._capture_highwater, self._media = None, None, None, None
        self._sequence, self._streak = 0, 0
        self._reason, self._closed = "awaiting_observation", None

    def _clock(self, now_s):
        if not _number(now_s) or now_s < 0 or (self._now is not None and now_s < self._now):
            self._closed, self._reason, self._streak = "invalid_clock", "invalid_clock", 0
            raise ValueError("One finite, nonnegative, monotonic host clock required")
        self._now = float(now_s)

    def _fresh(self, stamp):
        return stamp is not None and 0 <= self._now - stamp < self._max_age - 1e-12

    def _reject(self, reason, *, close=False):
        self._reason, self._streak = reason, 0
        if close:
            self._closed = reason
        return self._snapshot()

    def update(self, record, now_s, *, source_session_id):
        self._clock(now_s)
        if self._closed:
            return self._snapshot()
        if source_session_id != self._session:
            return self._reject("source_session_changed", close=True)
        if record is None:
            return self.poll(now_s)
        if not isinstance(record, dict):
            return self._reject("malformed_record")
        if record.get("status") == "source_closed":
            return self.close(now_s)
        # These changes break both the spatial frame and temporary object IDs.
        provenance = ((record.get("source_name") != self._source, "source_changed"),
            (record.get("is_replay") is not self._replay, "replay_mode_changed"),
            (self._configuration is not None and record.get("configuration_id") != self._configuration,
             "configuration_changed"),
            (record.get("coordinate_system") != COORDINATE_SYSTEM, "coordinate_system_changed"))
        for invalid, reason in provenance:
            if invalid:
                return self._reject(reason, close=True)
        try:
            if _point(record.get("field_size_mm")) != self._bounds:
                return self._reject("field_size_changed", close=True)
            ids = record.get("registered_robot_ids")
            if (not isinstance(ids, list) or any(not isinstance(r, str) for r in ids)
                    or len(ids) != len(self._roles) or set(ids) != set(self._roles)):
                return self._reject("robot_registry_changed", close=True)
            if record.get("device_io") is not False:
                return self._reject("unexpected_device_io", close=True)
            sequence, capture, received = (record.get(k) for k in ("sequence", "captured_at_s", "received_at_s"))
            if type(sequence) is not int or sequence <= self._sequence:
                return self._reject("out_of_order_sequence")
            self._sequence = sequence  # Invalid payloads do not get to reuse a sequence.
            if (not _number(capture) or not _number(received) or not 0 <= capture <= received <= now_s
                    or (self._capture_highwater is not None and capture < self._capture_highwater)):
                return self._reject("invalid_frame_time")
            self._capture_highwater = float(capture)
            if not self._fresh(capture):
                return self._reject("stale_frame")
            media = record.get("media_time_s")
            if self._replay and media is not None:
                if not _number(media) or media < 0 or (self._media is not None and media <= self._media):
                    return self._reject("invalid_media_time")
                self._media = float(media)
            if record.get("status") != "detected":
                return self._reject("rejected_frame")
            if record.get("tracking_session_closed") is not False or record.get("object_tracking_session_closed", False) is not False:
                return self._reject("tracking_session_closed", close=True)
            robots = self._read_robots(record, capture)
            objects = self._read_objects(record, capture)
        except (ValueError, TypeError, KeyError) as exc:
            return self._reject(f"invalid_observation:{exc}")
        # Commit parsed data atomically: a malformed final row cannot partly
        # replace the world with a mix of old and new observations.
        previous_objects = self._objects
        self._robots, self._objects = robots, objects
        for piece_id, (oid, _) in self._bindings.items():
            obj = objects.get(oid)
            previous = previous_objects.get(oid)
            spec = self._catalog[piece_id]
            if (obj is None or obj.identity_uncertain or obj.state in {"lost", "ambiguous"}
                    or obj.kind != spec.kind or (spec.colour is not None and obj.colour != spec.colour)
                    or (previous is not None and (previous.kind, previous.colour) != (obj.kind, obj.colour))):
                self._broken_bindings[piece_id] = "bound_track_identity_lost"
        complete = (record.get("observation_usable") is True and record.get("observation_complete") is True
            and record.get("stop_required") is False and not record.get("tracking_frame_reason")
            and not record.get("tracking_rejections") and not record.get("unknown_tag_ids")
            and not record.get("duplicate_tag_ids") and all(r.valid_for_control for r in robots.values()))
        if not complete or not self._fresh(self._capture):
            self._streak = 0
        self._capture = float(capture)
        self._streak = min(self._recovery, self._streak + 1) if complete else 0
        self._reason = None if complete else "incomplete_robot_observation"
        return self._snapshot()

    def _read_robots(self, record, capture):
        raw = _rows(record.get("robots"), "robot_id", len(self._roles))
        tracks = _rows(record.get("tracks"), "robot_id", len(self._roles))
        if set(tracks) != set(self._roles) or not set(raw) <= set(self._roles):
            raise ValueError("robot_registry_mismatch")
        for row in raw.values():
            _point(row.get("robot_center_mm"), self._bounds)
            _angle(row.get("heading_rad"))
        robots = {}
        for rid, track in tracks.items():
            state, valid = track.get("state"), track.get("valid_for_control")
            if state not in {"observed", "missing", "stale"} or type(valid) is not bool:
                raise ValueError("invalid_robot_tracking_state")
            if state != "observed" or not valid:
                robots[rid] = replace(self._robots[rid], state=state, valid_for_control=False, reason="robot_not_observed")
                continue
            point, heading = _point(track.get("robot_center_mm"), self._bounds), _angle(track.get("heading_rad"))
            measured = raw.get(rid)
            if (measured is None or point != _point(measured.get("robot_center_mm"), self._bounds)
                    or heading != _angle(measured.get("heading_rad")) or track.get("observed_at_s") != capture
                    or type(track.get("observed_at_s")) is bool):
                raise ValueError("track_does_not_match_raw_pose")
            velocity = _point(track.get("velocity_mm_s"))
            robots[rid] = ObservedRobot(rid, self._roles[rid], point, heading, velocity,
                                       float(capture), "observed", True, None)
        return robots

    def _read_objects(self, record, capture):
        raw = record.get("objects", [])
        if not isinstance(raw, list) or len(raw) > 512:
            raise ValueError("invalid_objects")
        candidates = set()
        for row in raw:
            if (not isinstance(row, dict) or row.get("kind") not in {"cylinder", "disc", "cube"}
                    or not _label(row.get("color"))):
                raise ValueError("invalid_object_candidate")
            candidate = (row["kind"], row["color"], _point(row.get("center_mm"), self._bounds))
            if candidate in candidates:
                raise ValueError("duplicate_object_candidate")
            candidates.add(candidate)
        if raw and "object_tracks" not in record:
            raise ValueError("missing_object_tracking")
        tracks = _rows(record.get("object_tracks", []), "object_id", 512)
        if record.get("object_tracking_device_io", False) is not False:
            raise ValueError("unexpected_object_tracking_io")
        if record.get("object_tracking_frame_reason"):
            raise ValueError("object_tracking_rejected_frame")
        objects, used = {}, set()
        for oid, track in tracks.items():
            kind, colour, state = (track.get(k) for k in ("kind", "color", "state"))
            flags = [track.get(k) for k in ("confirmed", "identity_uncertain", "valid_for_pick")]
            owner, lifecycle = track.get("owner_robot_id"), track.get("lifecycle")
            if (kind not in {"cylinder", "disc", "cube"} or not _label(colour)
                    or state not in {"tentative", "confirmed", "missing", "lost", "ambiguous", "released_pending"}
                    or any(type(v) is not bool for v in flags) or lifecycle not in {"free", "gripped", "released"}
                    or (owner is not None and (not isinstance(owner, str) or owner not in self._roles))
                    or (lifecycle == "gripped") != (owner is not None)):
                raise ValueError("invalid_object_tracking_state")
            point = _point(track.get("center_mm"), self._bounds)
            stamp, evidence = track.get("observed_at_s"), track.get("position_evidence")
            if ((stamp is not None and (not _number(stamp) or not 0 <= stamp <= capture))
                    or evidence not in {"vision", "explicit_release_hint"}):
                raise ValueError("invalid_object_evidence")
            streak, required = track.get("confirmation_streak"), track.get("required_confirmation_frames")
            if type(streak) is not int or streak < 0 or type(required) is not int or not 1 <= required <= 100:
                raise ValueError("invalid_object_confirmation")
            uncertain = track["identity_uncertain"] or oid in record.get("ambiguous_object_ids", [])
            position_valid = (state == "confirmed" and track["confirmed"] and not uncertain
                and stamp == capture and self._fresh(stamp) and evidence == "vision"
                and streak >= max(required, self._recovery))
            candidate = (kind, colour, point)
            if position_valid:
                if candidate not in candidates or candidate in used:
                    raise ValueError("track_does_not_match_unique_raw_object")
                used.add(candidate)
            pick = position_valid and track["valid_for_pick"] and owner is None
            reason = None if pick else "owned_object" if position_valid and owner else "object_not_confirmed_or_observed"
            objects[oid] = ObservedObject(oid, kind, colour, point if evidence == "vision" else None,
                stamp, state, uncertain, owner, lifecycle, evidence, position_valid, pick, reason)
        return objects

    def _snapshot(self):
        fresh = self._fresh(self._capture)
        if not fresh:
            self._streak = 0
        reason = self._closed or self._reason or ("observation_expired" if not fresh else None)
        ready = reason is None and self._streak >= self._recovery
        reason = reason or (None if ready else "confirming_observation")
        robots = tuple(replace(r, valid_for_control=ready and r.valid_for_control and self._fresh(r.observed_at_s),
            reason=r.reason or (None if ready else reason)) for r in self._robots.values())
        objects = tuple(replace(o, position_valid=o.position_valid and fresh and not self._reason and not self._closed
                               and self._fresh(o.observed_at_s),
            valid_for_pick=ready and o.valid_for_pick and self._fresh(o.observed_at_s),
            reason=o.reason or (None if ready else reason)) for o in self._objects.values())
        by_id = {o.object_id: o for o in objects}
        pieces = []
        for pid, spec in self._catalog.items():
            piece = MissionPieceState(pid, spec.kind, spec.colour)
            if pid in self._bindings:
                oid, evidence = self._bindings[pid]
                obj = by_id.get(oid)
                broken = self._broken_bindings.get(pid)
                piece = replace(piece, track_id=oid, binding_evidence=evidence,
                                reason=broken or "bound_track_not_observed")
                if obj:
                    piece = replace(piece, position_mm=obj.position_mm, observed_at_s=obj.observed_at_s,
                        owner_robot_id=obj.owner_robot_id, lifecycle=obj.lifecycle,
                        position_valid=obj.position_valid and not broken,
                        valid_for_pick=obj.valid_for_pick and not broken, reason=broken or obj.reason)
            pieces.append(piece)
        return WorldState(self._session, self._source, self._configuration, self._replay, self._now,
            self._sequence, self._capture, self._bounds, ready,
            "closed" if self._closed else "ready" if ready else "confirming" if reason == "confirming_observation" else "blocked",
            () if reason is None else (reason,), self._streak, self._recovery, robots, objects, tuple(pieces))

    def poll(self, now_s):
        self._clock(now_s)
        return self._snapshot()

    def invalidate(self, now_s, *, reason):
        """A known upstream rejection is not an ordinary between-frame poll.

        For example runtime may omit its observation after rejecting a foreign
        source. Consumers must forward that rejection instead of polling old data.
        """
        self._clock(now_s)
        if not _label(reason):
            raise ValueError("Explicit upstream rejection reason required")
        return self._snapshot() if self._closed else self._reject(reason)

    def close(self, now_s):
        self._clock(now_s)
        self._closed = self._closed or "source_closed"
        self._streak = 0
        return self._snapshot()

    def bind_piece(self, piece_id, object_id, now_s, *, evidence):
        world = self.poll(now_s)
        if self._closed or not world.ready:
            raise ValueError("Binding requires a ready observation session")
        if not isinstance(piece_id, str) or piece_id not in self._catalog or not _label(evidence):
            raise ValueError("Known mission piece and explicit mapping evidence required")
        if not _label(object_id):
            raise ValueError("Object track ID required")
        obj = next((o for o in world.objects if o.object_id == object_id), None)
        if obj is None or not obj.valid_for_pick:
            raise ValueError("Binding requires a fresh, confirmed, unowned, unambiguous track")
        if piece_id in self._bindings or any(oid == object_id for oid, _ in self._bindings.values()):
            raise ValueError("Explicitly unbind before replacing an existing one-to-one binding")
        spec = self._catalog[piece_id]
        if spec.kind != obj.kind or (spec.colour is not None and spec.colour != obj.colour):
            raise ValueError("Mission piece kind/colour does not match observation")
        self._bindings[piece_id] = (object_id, evidence)
        self._broken_bindings.pop(piece_id, None)
        return self._snapshot()

    def unbind_piece(self, piece_id, now_s):
        self._clock(now_s)
        if not isinstance(piece_id, str) or piece_id not in self._catalog:
            raise ValueError("Unknown mission piece")
        self._bindings.pop(piece_id, None)
        self._broken_bindings.pop(piece_id, None)
        return self._snapshot()
