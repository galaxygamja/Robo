"""Inspect a completed observation log and create an UNREVIEWED binding draft.

This offline tool never attaches to a running session. Temporary track IDs help
the operator choose a historical object, but are NOT copied to a future session.
Draft regions always require a human review of identity and starting placement.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from .mission_bindings import label, number
from .runtime_report import inspect_report


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError("Nonfinite JSON constant: " + value)


def _float(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("Nonfinite JSON value")
    return result


def load_review_world(path):
    """Only a complete validated report, never a tail of a live changing file."""
    path = Path(path)
    before = path.stat()
    result = inspect_report(path)
    header = world = None
    with path.open("r", encoding="utf-8-sig") as handle:
        while True:
            line = handle.readline(1048577)
            if not line:
                break
            if not line.endswith("\n") or len(line.encode("utf-8")) > 1048576:
                raise ValueError("Incomplete or oversized runtime record")
            row = json.loads(line, parse_constant=_reject_constant, parse_float=_float,
                             object_pairs_hook=_unique_pairs)
            if not isinstance(row, dict) or row.get("session_id") != result["session_id"]:
                raise ValueError("Report changed while preparing review")
            if header is None:
                header = row
                continue
            candidate = (row.get("mission") or {}).get("world")
            if isinstance(candidate, dict) and candidate.get("ready") is True:
                if (candidate.get("session_id") != result["session_id"]
                        or candidate.get("generated_at_s") != row.get("at_s")
                        or candidate.get("configuration_id") != header.get("configuration_id")
                        or candidate.get("source_name") != header.get("source_name")
                        or candidate.get("is_replay") is not header.get("is_replay")
                        or any(candidate.get(k) is not False for k in
                               ("device_io", "motion_permitted", "physical_identity_verified"))):
                    raise ValueError("World provenance does not match the runtime report")
                world = candidate
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError("Report changed while preparing review; use a closed report")
    if world is None:
        raise ValueError("No ready mission observation in this report")
    return header, world


def review_snapshot(path):
    header, world = load_review_world(path)
    return {"schema_version": 1, "historical_only": True,
            "session_id": world["session_id"], "source_sequence": world["source_sequence"],
            "captured_at_s": world["captured_at_s"], "source_name": world["source_name"],
            "is_replay": world["is_replay"], "observation_profile_id": header.get("observation_profile_id"),
            "field_size_mm": world["field_size_mm"], "objects": world["objects"], "pieces": world["pieces"],
            "physical_identity_verified": False, "device_io": False, "motion_permitted": False}


def create_draft(path, selections, *, half_size_mm):
    """Explicit piece=track selections only. Never choose nearest/same colour."""
    if not number(half_size_mm) or half_size_mm <= 0:
        raise ValueError("Positive finite region half-size in mm required")
    header, world = load_review_world(path)
    if not label(header.get("observation_profile_id")):
        raise ValueError("Record a new observation report with an observation profile identity")
    if not isinstance(selections, (tuple, list)) or not 1 <= len(selections) <= 64:
        raise ValueError("Explicit piece=track selections required (1..64)")
    objects = {o["object_id"]: o for o in world["objects"]}
    pieces = {p["piece_id"]: p for p in world["pieces"]}
    bound_pieces, bound_tracks, entries, regions = set(), set(), [], []
    for selection in selections:
        if not isinstance(selection, str) or selection.count("=") != 1:
            raise ValueError("Each selection must be PIECE_ID=TRACK_ID")
        pid, oid = selection.split("=")
        if pid not in pieces or oid not in objects or pid in bound_pieces or oid in bound_tracks:
            raise ValueError("Select distinct known piece IDs and observed track IDs")
        spec, obj = pieces[pid], objects[oid]
        if (spec["kind"] != obj["kind"] or spec["colour"] not in (None, obj["colour"])
                or obj.get("valid_for_pick") is not True or obj.get("position_evidence") != "vision"
                or obj.get("identity_uncertain") is not False or obj.get("owner_robot_id") is not None):
            raise ValueError("Selected track must have matching class and historical pickup evidence")
        point = obj.get("position_mm")
        if not isinstance(point, list) or len(point) != 2 or not all(number(v) for v in point):
            raise ValueError("Selected track needs a measured position")
        x, y = point
        r = half_size_mm
        if any(p-r < 0 or p+r > limit for p, limit in zip(point, world["field_size_mm"])):
            raise ValueError("Draft region leaves the field; reduce its reviewed size")
        for a, b in regions:
            if abs(x-a) <= 2*r and abs(y-b) <= 2*r:
                raise ValueError("Draft regions overlap or touch; select smaller regions")
        regions.append((x, y))
        entries.append({"piece_id": pid, "kind": obj["kind"], "colour": obj["colour"],
                        "region_mm": {"x_mm": x-r, "y_mm": y-r, "width_mm": 2*r, "height_mm": 2*r}})
        bound_pieces.add(pid)
        bound_tracks.add(oid)
    return {"schema_version": 1, "coordinate_system": world["coordinate_system"],
            "field_size_mm": world["field_size_mm"], "source_name": world["source_name"],
            "is_replay": world["is_replay"], "observation_profile_id": header["observation_profile_id"],
            "reviewed": False, "reviewed_by": "", "evidence": "", "bindings": entries}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    inspect = commands.add_parser("inspect", help="list historical object IDs/positions; no execution")
    inspect.add_argument("report", type=Path)
    draft = commands.add_parser("draft", help="save unreviewed regions from explicit piece=track choices")
    draft.add_argument("report", type=Path)
    draft.add_argument("--select", action="append", required=True, help="PIECE_ID=TRACK_ID, repeatable")
    draft.add_argument("--half-size-mm", type=float, required=True, help="explicit region half-width/height")
    draft.add_argument("--output", type=Path, required=True, help="new JSON file; never overwritten")
    args = parser.parse_args(argv)
    try:
        if args.command == "inspect":
            print(json.dumps(review_snapshot(args.report), ensure_ascii=True, allow_nan=False))
        else:
            if args.output.resolve() == args.report.resolve():
                raise ValueError("Draft output must not overwrite the source report")
            result = create_draft(args.report, args.select, half_size_mm=args.half_size_mm)
            payload = json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
            with args.output.open("x", encoding="utf-8") as handle:
                handle.write(payload)
            print(json.dumps({"status": "draft_saved_requires_review", "output": str(args.output.resolve()),
                              "reviewed": False, "device_io": False, "motion_permitted": False}))
        return 0
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        print(f"binding_tools: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
