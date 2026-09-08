"""Validate recorded conservative object-map evidence, without advancing it."""
from __future__ import annotations

from .mission_bindings import label, number
from .observed_obstacles import validate_obstacle_plan


def inspect_obstacle_snapshot(value, *, plan, world, header, at_s, previous=None):
    fields = {"schema_version", "mode", "session_id", "field_size_mm", "max_age_s", "source_name",
              "configuration_id", "is_replay", "status", "ready", "activated", "fault", "reason", "at_s",
              "captured_at_s", "source_sequence", "revision", "route_replans", "objects", "position_margin_mm",
              "unobserved_is_free", "contact_exemptions", "device_io", "motion_permitted",
              "physical_clearance_verified", "field_coverage_verified"}
    if (not isinstance(value, dict) or set(value) != fields
            or type(value["schema_version"]) is not int or value["schema_version"] != 1
            or value["mode"] != "observed_object_obstacles" or value["session_id"] != header["session_id"]
            or not isinstance(world, dict) or world.get("session_id") != header["session_id"]
            or any(value[k] is not False for k in ("unobserved_is_free", "device_io", "motion_permitted",
                                                "physical_clearance_verified", "field_coverage_verified"))
            or value["contact_exemptions"] != []
            or any(type(value[k]) is not bool for k in ("ready", "activated"))):
        raise ValueError("Invalid observed-obstacle envelope or physical/clear-space claim")
    dimensions = value["field_size_mm"]
    if (not isinstance(dimensions, (list, tuple)) or len(dimensions) != 2
            or any(not number(v) or not 0 < v <= 1e6 for v in dimensions)
            or tuple(dimensions) != tuple(world.get("field_size_mm", ()))):
        raise ValueError("Invalid obstacle-map field dimensions")
    validate_obstacle_plan(plan, field_size_mm=dimensions)
    fault, reason = value["fault"], value["reason"]
    if (not (fault is None or label(fault)) or not (reason is None or label(reason))
            or value["status"] != ("fault" if fault else "ready" if value["ready"] else "waiting")
            or fault is not None and reason != fault
            or value["ready"] and (not value["activated"] or fault is not None or reason is not None
                                   or world.get("ready") is not True)
            or not value["ready"] and reason is None):
        raise ValueError("Inconsistent obstacle-map status/readiness/fault")
    for key in ("source_sequence", "revision", "route_replans"):
        if type(value[key]) is not int or not 0 <= value[key] <= 2**53-1:
            raise ValueError("Invalid obstacle-map counter")
    if (type(world.get("source_sequence")) is not int or value["source_sequence"] > world["source_sequence"]
            or value["route_replans"] > plan["max_route_replans"]
            or value["at_s"] != at_s or not number(value["at_s"])
            or not number(value["position_margin_mm"]) or value["position_margin_mm"] != plan["position_margin_mm"]
            or not number(value["max_age_s"]) or not 0 < value["max_age_s"] <= .2):
        raise ValueError("Invalid obstacle-map clock/policy/counter")
    capture = value["captured_at_s"]
    if (capture is not None and (not number(capture) or not 0 <= capture <= at_s)
            or value["ready"] and (capture is None or at_s-capture >= value["max_age_s"]-1e-12)):
        raise ValueError("Expired obstacle geometry cannot be ready")
    if value["source_name"] is not None and (
            value["source_name"] != header.get("source_name")
            or value["configuration_id"] != header.get("configuration_id")
            or value["is_replay"] is not header.get("is_replay")):
        raise ValueError("Obstacle-map source provenance changed")
    if value["activated"] and (value["revision"] == 0 or capture is None or value["source_name"] is None):
        raise ValueError("Activated obstacle map needs recorded source evidence")
    objects = value["objects"]
    if not isinstance(objects, list) or len(objects) > 64:
        raise ValueError("Obstacle map exceeds its object capacity")
    radii = {(p["kind"], p["colour"]): p["radius_mm"] for p in plan["footprints"]}
    by_id = {}
    for obj in objects:
        if (not isinstance(obj, dict) or set(obj) != {"track_id", "kind", "colour", "position_mm", "radius_mm",
                "observed_at_s", "source_sequence", "confirmation_state"}
                or not label(obj["track_id"]) or obj["track_id"] in by_id
                or not label(obj["kind"]) or not label(obj["colour"])
                or (obj["kind"], obj["colour"]) not in radii
                or not number(obj["radius_mm"]) or obj["radius_mm"] != radii[(obj["kind"], obj["colour"])]
                or not isinstance(obj["confirmation_state"], str)
                or obj["confirmation_state"] not in {"confirmed", "tentative"}):
            raise ValueError("Invalid obstacle object identity/footprint")
        point = obj["position_mm"]
        if (not isinstance(point, (list, tuple)) or len(point) != 2
                or any(not number(v) or not 0 <= v <= bound for v, bound in zip(point, dimensions))
                or not number(obj["observed_at_s"]) or not 0 <= obj["observed_at_s"] <= at_s
                or type(obj["source_sequence"]) is not int or not 1 <= obj["source_sequence"] <= value["source_sequence"]
                or value["ready"] and (obj["source_sequence"] != value["source_sequence"]
                                       or obj["observed_at_s"] != capture)):
            raise ValueError("Invalid or stale obstacle position evidence")
        by_id[obj["track_id"]] = obj
    if objects and not value["activated"]:
        raise ValueError("Unactivated obstacle map cannot contain an applied object batch")
    if value["ready"]:
        if (value["source_sequence"] != world["source_sequence"] or capture != world.get("captured_at_s")
                or world.get("generated_at_s") != at_s
                or world.get("source_name") != header.get("source_name")
                or world.get("configuration_id") != header.get("configuration_id")
                or world.get("is_replay") is not header.get("is_replay")):
            raise ValueError("Ready obstacle evidence must use the current matching world frame")
        observed = world.get("objects")
        if (not isinstance(observed, list) or len(observed) > 512
                or any(not isinstance(o, dict) or not label(o.get("object_id")) for o in observed)
                or len({o["object_id"] for o in observed}) != len(observed)):
            raise ValueError("Ready obstacle world needs unique bounded object evidence")
        current = {o["object_id"]: o for o in observed if o.get("identity_uncertain") is False
                   and o.get("state") in ("tentative", "confirmed") and o.get("position_evidence") == "vision"
                   and o.get("observed_at_s") == capture}
        if set(current) != set(by_id):
            raise ValueError("Ready obstacle IDs do not cover current world object evidence")
        for oid, obj in by_id.items():
            measured = current[oid]
            if ((obj["kind"], obj["colour"], obj["confirmation_state"]) != (
                    measured.get("kind"), measured.get("colour"), measured.get("state"))
                    or not isinstance(measured.get("position_mm"), (list, tuple))
                    or tuple(obj["position_mm"]) != tuple(measured["position_mm"])):
                raise ValueError("Ready obstacle geometry/class must match current world evidence")
    if previous is not None:
        if (any(value[k] < previous[k] for k in ("revision", "source_sequence", "route_replans"))
                or previous["activated"] and not value["activated"]
                or previous["fault"] is not None and fault != previous["fault"]):
            raise ValueError("Obstacle-map history counters/fault changed")
        if previous["fault"] is not None and any(value[k] != previous[k] for k in
                ("objects", "captured_at_s", "source_sequence", "revision", "route_replans")):
            raise ValueError("Latched obstacle-map evidence cannot be rewritten")
        for old in previous["objects"]:
            new = by_id.get(old["track_id"])
            if (new is None or (new["kind"], new["colour"], new["radius_mm"]) != (old["kind"], old["colour"], old["radius_mm"])
                    or new["source_sequence"] < old["source_sequence"] or new["observed_at_s"] < old["observed_at_s"]
                    or value["revision"] == previous["revision"] and new["position_mm"] != old["position_mm"]):
                raise ValueError("Obstacle footprint disappeared, changed identity or rewound")
    return value
