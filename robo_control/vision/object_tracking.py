"""Conservative temporal identity for colour/geometry candidates, not a gripper.

Matching uses a bounded distance in field mm and never infers pickup or release.
An equally plausible association latches identity uncertainty: affected IDs are
not silently reused when the objects separate. They expire and new IDs must be
confirmed. Colour alone cannot prove physical identity through an occlusion.

Host monotonic time always controls frame freshness and the no-frame watchdog.
For unpaced video, available media timestamps ALSO control miss expiry, so a
one-second gap in a fast-decoded recording is not mistaken for a millisecond gap.
No velocity is inferred from video decode speed. A still/replay image without a
media timestamp has host-only expiry and explicitly reports that limitation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from ..fleet import robot_id_valid


def _number(value: Any) -> bool:
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def _point(value: Any) -> tuple[float, float] | None:
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or not all(_number(v) for v in value)):
        return None
    return float(value[0]), float(value[1])


@dataclass
class _Track:
    object_id: str
    color: str
    kind: str
    center_mm: tuple[float, float]
    anchor_at_s: float
    last_seen_at_s: float | None
    last_seen_media_s: float | None
    streak: int = 1
    confirmed: bool = False
    state: str = "tentative"
    identity_uncertain: bool = False
    owner_robot_id: str | None = None
    lifecycle: str = "free"
    reason: str | None = None
    position_evidence: str = "vision"
    events: list[dict] = field(default_factory=list)


class ObjectTracker:
    """One camera/session, arbitrary registered owners, no device side effects.

    ``confirmed`` means a track passed temporal detection checks, not that its
    physical identity, grasp, destination, or score has been independently proved.
    Consumers must run ``snapshot`` on their own watchdog when input stops.
    ``source_closed``/``close`` permanently ends this instance; a new camera
    session needs a new tracker instead of reviving its old identities.
    """

    def __init__(self, robot_ids, *, confirm_frames: int = 3,
                 max_distance_mm: float = 40.0, miss_timeout_s: float = 0.5,
                 ambiguity_margin_mm: float = 5.0, max_frame_age_s: float = 0.5,
                 max_tracks: int = 512):
        ids = tuple(robot_ids)
        if (not all(robot_id_valid(rid) for rid in ids)
                or len(set(ids)) != len(ids)):
            raise ValueError("Object owners must be unique registered robot IDs")
        if type(confirm_frames) is not int or confirm_frames < 1:
            raise ValueError("confirm_frames must be a positive integer")
        if type(max_tracks) is not int or max_tracks < 1:
            raise ValueError("max_tracks must be a positive integer")
        if any(not _number(v) or v <= 0 for v in
               (max_distance_mm, miss_timeout_s, max_frame_age_s)):
            raise ValueError("Positive finite association and timeout limits required")
        if (not _number(ambiguity_margin_mm)
                or not 0 <= ambiguity_margin_mm < max_distance_mm):
            raise ValueError("ambiguity margin must be nonnegative and below association distance")
        self.robot_ids = frozenset(ids)
        self.confirm_frames = confirm_frames
        self.max_distance_mm = float(max_distance_mm)
        self.miss_timeout_s = float(miss_timeout_s)
        self.ambiguity_margin_mm = float(ambiguity_margin_mm)
        self.max_frame_age_s = float(max_frame_age_s)
        self.max_tracks = max_tracks
        self._tracks: dict[str, _Track] = {}
        self._next_id = 1
        self._source: str | None = None
        self._sequence = 0
        self._capture = -math.inf
        self._now = -math.inf
        self._replay: bool | None = None
        self._media: float | None = None
        self._latest_event_s = -math.inf
        self._closed = False

    def _clock(self, now_s: float) -> None:
        if not _number(now_s) or now_s < self._now:
            raise ValueError("Object tracker clock must be finite and monotonic")
        self._now = float(now_s)

    def _age(self, track: _Track, now_s: float) -> float:
        anchor = track.anchor_at_s if track.last_seen_at_s is None else track.last_seen_at_s
        return now_s - anchor

    def _expired(self, track: _Track, now_s: float, media: float | None) -> bool:
        return (self._age(track, now_s) >= self.miss_timeout_s
                or (media is not None and track.last_seen_media_s is not None
                    and media - track.last_seen_media_s >= self.miss_timeout_s))

    def _miss(self, track: _Track, now_s: float, media: float | None,
              reason: str = "not_observed") -> None:
        if track.state == "lost":
            return
        track.streak = 0
        if self._expired(track, now_s, media):
            # A missing image cannot release ownership. Explicit hooks must
            # resolve a carried object's disposition even after visual loss.
            track.state = "lost"
            if track.owner_robot_id is not None:
                track.identity_uncertain = True
        elif track.identity_uncertain:
            track.state = "ambiguous"
        else:
            track.state = "missing"
        track.reason = reason

    def _result(self, now_s: float, *, frame_reason=None, rejected=(), ambiguous=()):
        tracks = []
        for track in self._tracks.values():
            age = None if track.last_seen_at_s is None else (now_s - track.last_seen_at_s) * 1000
            tracks.append({
                "object_id": track.object_id, "color": track.color, "kind": track.kind,
                "center_mm": list(track.center_mm), "state": track.state,
                "confirmed": track.confirmed, "confirmation_streak": track.streak,
                "required_confirmation_frames": self.confirm_frames,
                "identity_uncertain": track.identity_uncertain,
                "owner_robot_id": track.owner_robot_id, "lifecycle": track.lifecycle,
                "observed_at_s": track.last_seen_at_s, "media_time_s": track.last_seen_media_s,
                "age_ms": age, "reason": track.reason,
                "position_evidence": track.position_evidence,
                "last_lifecycle_event": dict(track.events[-1]) if track.events else None,
                "valid_for_pick": (not self._closed and track.state == "confirmed" and track.confirmed
                    and not track.identity_uncertain and track.owner_robot_id is None
                    and track.last_seen_at_s is not None and age < self.max_frame_age_s * 1000),
                "physical_identity_verified": False,
            })
        return {"object_tracks": tracks, "object_tracking_frame_reason": frame_reason,
                "object_tracking_rejections": list(rejected),
                "ambiguous_object_ids": sorted(ambiguous),
                "object_tracking_time_basis": ("host_freshness_and_replay_media_expiry" if self._replay and self._media is not None
                    else "host_only_replay_media_unavailable" if self._replay else "host_monotonic"),
                "object_tracking_session_closed": self._closed,
                "object_tracking_device_io": False}

    def _reject(self, now_s: float, reason: str):
        for track in self._tracks.values():
            self._miss(track, now_s, None, reason)
        return self._result(now_s, frame_reason=reason)

    def update(self, record: dict, now_s: float) -> dict:
        """Consume one frame record, retaining raw detections outside this API.

        Bad frames never move a track. Bad individual candidates are reported;
        their frame cannot provide consecutive-confirmation evidence.
        """
        self._clock(now_s)
        if self._closed:
            return self._reject(now_s, "session_closed")
        if not isinstance(record, dict):
            return self._reject(now_s, "malformed_record")
        if record.get("status") == "source_closed":
            self._closed = True
            return self._reject(now_s, "source_closed")
        # A watchdog has no frame identity to consume. Rejected camera frames
        # DO: validate/consume their header before looking at detection status.
        if (record.get("status") != "detected"
                and not any(key in record for key in ("source_name", "sequence", "frame_sequence", "captured_at_s"))):
            return self._reject(now_s, str(record.get("reason", record.get("status", "no_frame"))))
        source = record.get("source_name")
        sequence = record.get("sequence", record.get("frame_sequence"))
        captured = record.get("captured_at_s")
        received = record.get("received_at_s", captured)
        if received is None:
            received = captured
        replay = record.get("is_replay", False)
        media = record.get("media_time_s") if replay is True else None
        if not isinstance(source, str) or not source or (self._source is not None and source != self._source):
            return self._reject(now_s, "source_changed")
        if type(sequence) is not int or sequence <= self._sequence:
            return self._reject(now_s, "out_of_order_sequence")
        if (not _number(captured) or not _number(received)
                or not self._capture <= captured <= received <= now_s):
            return self._reject(now_s, "stale_or_invalid_timestamp")
        # A structurally valid, ordered host header consumes the high-water
        # marks even when age, media metadata, status, or candidates fail later.
        self._source, self._sequence, self._capture = source, sequence, float(captured)
        if now_s - captured > self.max_frame_age_s:
            return self._reject(now_s, "stale_or_invalid_timestamp")
        if captured < self._latest_event_s:
            return self._reject(now_s, "frame_precedes_lifecycle_event")
        if type(replay) is not bool or (self._replay is not None and replay != self._replay):
            return self._reject(now_s, "replay_mode_changed")
        if media is not None and (not _number(media) or media < 0
                or (self._media is not None and media <= self._media)):
            return self._reject(now_s, "invalid_or_out_of_order_media_time")
        self._replay, self._media = replay, None if media is None else float(media)
        if record.get("status") != "detected":
            return self._reject(now_s, str(record.get("reason", record.get("status", "no_frame"))))
        raw = record.get("objects", [])
        if not isinstance(raw, (list, tuple)) or len(raw) > self.max_tracks:
            return self._reject(now_s, "malformed_objects")
        candidates, rejected = [], []
        for index, obj in enumerate(raw):
            if not isinstance(obj, dict):
                rejected.append({"index": index, "reason": "malformed_object"})
                continue
            point = _point(obj.get("center_mm"))
            color, kind = obj.get("color"), obj.get("kind")
            if (point is None or not isinstance(color, str) or not color.strip() or len(color) > 64
                    or not isinstance(kind, str) or kind not in {"cylinder", "disc", "cube"}):
                rejected.append({"index": index, "reason": "invalid_object"})
                continue
            candidates.append({"center_mm": point, "color": color, "kind": kind, "index": index})
        if rejected:
            for track in self._tracks.values():
                self._miss(track, now_s, media, "invalid_candidate_frame")
            return self._result(now_s, frame_reason="invalid_candidate_frame", rejected=rejected)
        # Deterministic order ensures IDs do not depend on contour enumeration.
        candidates.sort(key=lambda item: (item["kind"], item["color"], item["center_mm"]))
        for track in self._tracks.values():
            if self._expired(track, now_s, media):
                self._miss(track, now_s, media, "miss_timeout")
        # An owned item is never forgotten merely because it became invisible.
        # Keep its last-position association area quarantined until the explicit
        # release hook supplies a new hint; do not mint a free ID over it.
        active = {rid: t for rid, t in self._tracks.items()
                  if t.state != "lost" or t.owner_robot_id is not None}
        distances = {}
        for rid, track in active.items():
            for ci, candidate in enumerate(candidates):
                if (track.color, track.kind) != (candidate["color"], candidate["kind"]):
                    continue
                distance = math.dist(track.center_mm, candidate["center_mm"])
                if distance <= self.max_distance_mm:
                    distances[rid, ci] = distance

        # Two near-equal alternatives on EITHER side make the whole connected
        # association component uncertain; greedy tie-breaking would swap IDs.
        ambiguous_tracks = {rid for rid, t in active.items() if t.identity_uncertain}
        ambiguous_candidates: set[int] = set()
        for rid in active:
            options = sorted((d, ci) for (r, ci), d in distances.items() if r == rid)
            if len(options) > 1 and options[1][0] - options[0][0] <= self.ambiguity_margin_mm:
                ambiguous_tracks.add(rid)
        for ci in range(len(candidates)):
            options = sorted((d, rid) for (rid, c), d in distances.items() if c == ci)
            if len(options) > 1 and options[1][0] - options[0][0] <= self.ambiguity_margin_mm:
                ambiguous_candidates.add(ci)
        # One physical image location cannot establish two distinct identities
        # just because overlapping profiles disagree on colour or object kind.
        # Seed this BEFORE graph propagation so existing compatible tracks are
        # also latched uncertain, rather than silently revived next frame.
        for ci, a in enumerate(candidates):
            for cj in range(ci + 1, len(candidates)):
                b = candidates[cj]
                if math.dist(a["center_mm"], b["center_mm"]) <= self.ambiguity_margin_mm:
                    ambiguous_candidates.update((ci, cj))
        changed = True
        while changed:
            before = (len(ambiguous_tracks), len(ambiguous_candidates))
            for rid, ci in distances:
                if rid in ambiguous_tracks or ci in ambiguous_candidates:
                    ambiguous_tracks.add(rid)
                    ambiguous_candidates.add(ci)
            changed = before != (len(ambiguous_tracks), len(ambiguous_candidates))
        for rid in ambiguous_tracks:
            track = active[rid]
            track.identity_uncertain = True
            self._miss(track, now_s, media, "ambiguous_association")

        matched_tracks, matched_candidates = set(), set()
        for (rid, ci), _ in sorted(distances.items(), key=lambda row: (row[1], row[0])):
            if (rid in ambiguous_tracks or ci in ambiguous_candidates
                    or rid in matched_tracks or ci in matched_candidates):
                continue
            track = active[rid]
            track.center_mm = candidates[ci]["center_mm"]
            track.last_seen_at_s, track.last_seen_media_s = float(captured), media
            track.position_evidence = "vision"
            track.streak += 1
            track.confirmed = track.confirmed or track.streak >= self.confirm_frames
            track.state = "confirmed" if track.confirmed else "tentative"
            track.reason = None
            matched_tracks.add(rid)
            matched_candidates.add(ci)
        for rid, track in active.items():
            if rid not in matched_tracks and rid not in ambiguous_tracks:
                self._miss(track, now_s, media)
        for ci, candidate in enumerate(candidates):
            if ci in matched_candidates or ci in ambiguous_candidates:
                continue
            if len(self._tracks) >= self.max_tracks:
                rejected.append({"index": candidate["index"], "reason": "track_capacity_reached"})
                continue
            rid = f"O{self._next_id:04d}"
            self._next_id += 1
            confirmed = self.confirm_frames == 1
            self._tracks[rid] = _Track(rid, candidate["color"], candidate["kind"], candidate["center_mm"],
                float(captured), float(captured), media, confirmed=confirmed,
                state="confirmed" if confirmed else "tentative")
        rejected.extend({"index": candidates[ci]["index"], "reason": "ambiguous_association"}
                        for ci in sorted(ambiguous_candidates))
        return self._result(now_s, rejected=rejected, ambiguous=ambiguous_tracks)

    def snapshot(self, now_s: float) -> dict:
        """Invalidate current observation evidence when the consumer has no frame."""
        self._clock(now_s)
        return self._reject(now_s, "session_closed" if self._closed else "no_frame")

    def close(self, now_s: float) -> dict:
        """Permanently close this source/session; reconnect with a new instance."""
        return self.update({"status": "source_closed"}, now_s)

    def _event(self, object_id, robot_id, now_s, evidence):
        if self._closed:
            raise ValueError("Object tracker session is closed; create a new instance")
        if not isinstance(object_id, str) or object_id not in self._tracks:
            raise ValueError("Unknown object ID")
        if not robot_id_valid(robot_id) or robot_id not in self.robot_ids:
            raise ValueError("Unknown robot owner")
        if not isinstance(evidence, str) or not evidence.strip() or len(evidence) > 200:
            raise ValueError("An explicit nonempty ownership evidence description is required")
        if not _number(now_s) or now_s < self._now:
            raise ValueError("Lifecycle event clock must be finite and monotonic")
        return self._tracks[object_id]

    def mark_gripped(self, object_id: str, robot_id: str, now_s: float, *, evidence: str) -> dict:
        """Explicit external hook; imagery can NEVER invoke this automatically.

        ``evidence`` describes a sensor/operator/simulation acknowledgement. This
        API records that assertion; it does not itself verify a physical grasp.
        """
        track = self._event(object_id, robot_id, now_s, evidence)
        if (track.owner_robot_id is not None or not track.confirmed or track.state != "confirmed"
                or track.identity_uncertain or track.last_seen_at_s is None
                or now_s - track.last_seen_at_s >= min(self.miss_timeout_s, self.max_frame_age_s)):
            raise ValueError("Grip requires a fresh, confirmed, unowned, unambiguous object")
        self._clock(now_s)
        self._latest_event_s = float(now_s)
        track.owner_robot_id, track.lifecycle = robot_id, "gripped"
        track.events.append({"event": "gripped", "robot_id": robot_id, "at_s": float(now_s), "evidence": evidence})
        return self._result(now_s)

    def mark_released(self, object_id: str, robot_id: str, now_s: float, *,
                      center_mm, evidence: str) -> dict:
        """Release by the recorded owner, then require fresh visual confirmation.

        The supplied release position is only an association hint. It is not
        counted as a detected frame, correct placement, or a scoring event.
        """
        track = self._event(object_id, robot_id, now_s, evidence)
        point = _point(center_mm)
        if track.owner_robot_id != robot_id or track.lifecycle != "gripped":
            raise ValueError("Only the recorded gripping robot may release this object")
        if point is None:
            raise ValueError("Release position must be a finite mm coordinate pair")
        self._clock(now_s)
        self._latest_event_s = float(now_s)
        track.owner_robot_id, track.lifecycle = None, "released"
        track.center_mm, track.position_evidence = point, "explicit_release_hint"
        # The hook supplies a host event time, not a video media time. Do not
        # reuse an old pre-release frame's media timestamp for this new hint.
        track.anchor_at_s, track.last_seen_at_s, track.last_seen_media_s = float(now_s), None, None
        track.streak, track.confirmed, track.identity_uncertain = 0, False, False
        track.state, track.reason = "released_pending", "awaiting_visual_confirmation"
        track.events.append({"event": "released", "robot_id": robot_id, "at_s": float(now_s), "evidence": evidence})
        return self._result(now_s)
