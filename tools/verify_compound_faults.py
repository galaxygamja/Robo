"""Generated pixels + deterministic clocks + existing runtime/fake wire.

This complements, never replaces, the process-isolated video CLI verifier.
There is no physical IO, pose integrator, sensor feedback or real-time claim.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robo_control.adapters import CameraFrame
from robo_control.fleet import roles_from_scenario
from robo_control.runtime_report import inspect_report
from robo_control.runtime_session import LiveControlSession
from robo_control.vision.calibration import FieldCalibration, vision_dependencies
from robo_control.vision.colors import ColorDetector
from robo_control.vision.detection import DetectionPipeline
from robo_control.vision.tags import TagDetectorConfig
from tools.verify_mission_operations import prepare_fixture, write_json

CASES = {
    "multi_clear": (None, None, "operator_stop"),
    "occlusion_request_delay": ("occlusion", "request_delay_s", "object_obstacle_lost"),
    "occlusion_ack_delay": ("occlusion", "response_delay_s", "object_obstacle_lost"),
    "crossing_request_delay": ("crossing", "request_delay_s", "object_obstacle_unconfirmed_or_ambiguous"),
    "crossing_ack_delay": ("crossing", "response_delay_s", "object_obstacle_unconfirmed_or_ambiguous"),
    "envelope_request_delay": ("envelope", "request_delay_s", "mission_obstacle_envelope"),
    "envelope_ack_delay": ("envelope", "response_delay_s", "mission_obstacle_envelope"),
    "occlusion_stop_ack_loss": ("occlusion", "drop_responses", "object_obstacle_lost"),
}


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def robot_wire(row, rid="H1"):
    return next(r for r in row["wire"]["robots"] if r["robot_id"] == rid)


def render_frame(base, case, frame_index):
    """Fixture coordinates only; no location depends on calculated commands."""
    cv2, _ = vision_dependencies()
    frame = base.copy()
    # Remove only the two source-fixture objects, leaving all measured tags.
    frame[200:281, 270:551] = 255
    shift = min(15, max(0, frame_index - 9) * 3) if case == "crossing" else 0
    cv2.circle(frame, (300 + shift, 240), 10, (0, 0, 255), -1)
    if case == "crossing":
        cv2.circle(frame, (330 - shift, 240), 10, (0, 0, 255), -1)
    if case != "occlusion" or frame_index < 10 or frame_index >= 20:
        cv2.circle(frame, (460, 240), 10, (0, 255, 0), -1)
    cv2.circle(frame, (520, 240), 10, (0, 255, 0), -1)
    if case == "envelope" and 10 <= frame_index < 20:
        cv2.circle(frame, (184, 135), 10, (0, 255, 0), -1)
    return frame


def check_trace(rows, *, case, expected_reason, lost_stop_ack):
    """Audit the trace itself, not a caller-provided 'passed' bit."""
    ticks = rows[1:]
    final = ticks[-1]
    require(final["closed_reason"] == expected_reason, "Wrong final reason")
    before = next(r for r in ticks if r["tick_sequence"] == 9)
    require(before["object_obstacles"]["ready"] and before["bindings"]["status"] == "bound", "Missing ready baseline")
    expected_objects = 4 if case == "crossing" else 3
    require(len(before["object_obstacles"]["objects"]) == expected_objects, "Did not observe every baseline object")
    require(any(robot_wire(r)["receiver"]["v_mm_s"] or robot_wire(r)["receiver"]["omega_rad_s"]
                for r in ticks[:8]), "No nonzero fake command before fault")
    require(any(r["mission"]["observed_pickup_in_use"] for r in ticks[:8]), "Observed pickup not exercised")
    if case is not None:
        require(robot_wire(before)["sender"]["pending_request"] == "drive", "No drive waiting during hazard")
    closed = [r for r in ticks if r["closed_reason"] is not None]
    first = closed[0]
    if case is not None:
        prior = ticks[ticks.index(first) - 1]
        for rid in ("H1", "H2", "B1", "B2"):
            count = robot_wire(prior, rid)["receiver"]["accepted_drive_count"]
            require(all(robot_wire(r, rid)["receiver"]["accepted_drive_count"] == count for r in closed),
                    "Drive executed on or after hazard frame")
        if case in {"crossing", "occlusion"}:
            require(first["object_obstacles"]["objects"] == prior["object_obstacles"]["objects"],
                    "Lost/ambiguous obstacle history was cleared or replaced")
    require(all(r["stop_requested"] for r in first["wire"]["robots"]), "Fleet stop not requested")
    for row in ticks:
        if row["tick_sequence"] >= 9:
            require(row["bindings"]["mappings"] == before["bindings"]["mappings"], "Target silently rebound")
        require(row["device_io"] is False and row["motion_permitted"] is False, "Physical output claim")
        require(not row["mission"]["completed_tasks"] and row["mission"]["physical_mission_verified"] is False,
                "Invented mission completion")
        observation = row.get("observation")
        if observation and observation.get("status") == "detected":
            measured = {r["robot_id"]: r["robot_center_mm"] for r in observation["robots"]}
            require(len(measured) == 4, "Fixture unexpectedly lost robot localization")
            for track in observation["tracks"]:
                if track["state"] == "observed":
                    require(track["robot_center_mm"] == measured[track["robot_id"]], "Pose was not image-measured")
        if row in closed:
            require(all(r["forward_velocity_mm_s"] == 0 and r["angular_velocity_rad_s"] == 0
                        for r in row["actuator"]["robots"]), "Nonzero local output after stop")
    require(all(r["receiver"]["v_mm_s"] == 0 and r["receiver"]["omega_rad_s"] == 0
                for r in final["wire"]["robots"]), "Final modeled receiver output not zero")
    unconfirmed = final["wire"]["unconfirmed_stop_robot_ids"]
    require(unconfirmed == (["H1"] if lost_stop_ack else []), "Stop ACK certainty was misstated")
    return {"ticks": len(ticks), "first_stop_tick": first["tick_sequence"],
            "baseline_objects": expected_objects, "hazard_injected": case is not None,
            "no_post_hazard_drive": True if case is not None else None,
            "unconfirmed_stop_robot_ids": unconfirmed, "no_pose_integrator": True,
            "no_sensor_feedback_injected": True, "physical_stop_verified": False}


def run_case(root, name, *, delay_s=.12):
    case, delay_option, expected = CASES[name]
    require(type(delay_s) in (int, float) and .08 <= delay_s <= .16, "Test delay must be 80..160 ms")
    cv2, _ = vision_dependencies()
    directory = root / name
    directory.mkdir(exist_ok=False)
    calibration = FieldCalibration.load(root / "camera.json")
    tags = TagDetectorConfig.load(root / "tags.json")
    fleet = json.loads((root / "fleet.json").read_text(encoding="utf-8"))
    mission = json.loads((root / "mission.json").read_text(encoding="utf-8"))
    # Larger fixture envelope makes a visible, non-tag-overlapping new object
    # intersect the body boundary. This is NOT a measured robot dimension.
    mission["radii_mm"] = dict.fromkeys(mission["radii_mm"], 80.)
    binding = json.loads((root / "steady-bindings.json").read_text(encoding="utf-8"))
    source = "video:synthetic-compound-" + name
    binding["source_name"] = source
    obstacles = json.loads((root / "obstacles.json").read_text(encoding="utf-8"))
    session = LiveControlSession(roles=roles_from_scenario(fleet), field_size_mm=calibration.field_size_mm,
        source_name=source, is_replay=True, mission_plan=mission, fleet=fleet, wire_fake=True,
        track_objects=True, binding_plan=binding, observation_profile_id=binding["observation_profile_id"],
        obstacle_plan=obstacles)
    clock = [10.]
    detector = DetectionPipeline(calibration, tags, colors=ColorDetector.load(calibration, root / "colors.json"),
                                 registry_checked=True, clock=lambda: clock[0])
    reader = cv2.VideoCapture(str(root / "steady.avi"))
    try:
        ok, base = reader.read()
    finally:
        reader.release()
    require(ok, "Cannot decode source fixture")
    header = {"schema_version": 1, "event": "session_started", "session_id": session.controller.session_id,
        "configuration_id": None, "calibration": calibration.as_dict(), "tags": tags.as_dict(),
        "roles": roles_from_scenario(fleet), "drive_model": "differential_body", "mission_plan": mission,
        "binding_plan": binding, "observation_profile_id": binding["observation_profile_id"],
        "mission_observe_only": False, "obstacle_plan": obstacles, "source_name": source, "is_replay": True,
        "input_mode": "video_replay", "output_mode": "dry_run_commands", "transport_mode": "fake_wire",
        "device_io": False, "motion_permitted": False}
    rows, detections, first_fault = [header], [], None
    injections = []
    try:
        for index in range(1, 41):
            clock[0] = 10. + index * .02
            if index == 9 and delay_option:
                options = {delay_option: 100 if delay_option == "drop_responses" else delay_s}
                session.wire.configure_link("H1", session.now, **options)
                injections.append({"before_frame": index, "host_at_s": session.now, "robot_id": "H1", "options": options})
            if first_fault is None and case is not None and index == 10:
                injections.append({"before_frame": index, "pixel_fault": case})
            frame = render_frame(base, case, index)
            record = detector.process(CameraFrame(frame, clock[0], index, source,
                received_at_s=clock[0], media_time_s=index*.02, is_replay=True,
                timestamp_basis="synthetic_test_clock")).record
            detections.append(record)
            event = session.advance(record, clock[0])
            rows.append(event)
            if index in (8, 20) or event["closed_reason"] is not None and first_fault is None:
                require(cv2.imwrite(str(directory / f"frame-{index:03d}.png"), frame), "Cannot save pixel evidence")
            if event["closed_reason"] is not None and first_fault is None:
                first_fault = index
            # Continue feeding separating/reappearing pixels after a fault.
            # No explicit recovery: the closed mission must stay closed.
            if index == 20 and case is None:
                rows.append(session.close(clock[0] + .001, "operator_stop"))
                first_fault = index
        require(first_fault is not None, "Expected safety closure never occurred")
    finally:
        if session.closed_reason is None:
            rows.append(session.close(clock[0] + .001, "verifier_cleanup"))
        # The normal report deliberately ends at the FIRST session closure.
        # Later wire drains are a separate trace, never backdated stop ACKs.
        for filename, stop_at_close in (("runtime.jsonl", True), ("shutdown-trace.jsonl", False)):
            with (directory / filename).open("x", encoding="utf-8") as handle:
                for row in rows:
                    handle.write(json.dumps(row, allow_nan=False) + "\n")
                    if stop_at_close and row.get("closed_reason") is not None:
                        break
        with (directory / "detections.jsonl").open("x", encoding="utf-8") as handle:
            for record in detections:
                handle.write(json.dumps(record, allow_nan=False) + "\n")
        write_json(directory / "injections.json", {"synthetic": True, "clock_mode": "deterministic_not_realtime",
            "case": name, "delay_s": delay_s, "events": injections})
    checks = check_trace(rows, case=case, expected_reason=expected, lost_stop_ack=delay_option == "drop_responses")
    audit = inspect_report(directory / "runtime.jsonl")
    require(audit["closed_reason"] == expected and audit["physical_stop_verified"] is False,
            "Strict report analysis disagrees with fixture oracle")
    first_closed = next(r for r in rows[1:] if r["closed_reason"] is not None)
    require(audit["unconfirmed_stop_robot_ids"] == first_closed["wire"]["unconfirmed_stop_robot_ids"],
            "Report hid missing stop ACK at closure")
    result = {"case": name, "expected_reason": expected, "status": "passed", "checks": checks,
              "report_analysis": audit}
    write_json(directory / "verification.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path, help="New evidence directory; never overwrite")
    args = parser.parse_args(argv)
    root = args.output_dir.resolve()
    try:
        root.mkdir(parents=True, exist_ok=False)
        versions = prepare_fixture(root, 6.)
        results = [run_case(root, name) for name in CASES]
        require(len({r["report_analysis"]["session_id"] for r in results}) == len(CASES), "Session ID reused")
        summary = {"status": "passed", "schema_version": 1, "clock_mode": "deterministic_not_realtime",
            "physical_camera_used": False, "physical_robot_used": False, "physical_stop_verified": False,
            "geometry_is_fixture_only": True, **versions, "cases": results}
        write_json(root / "verification.json", summary)
        print(json.dumps({"status": "passed", "cases": len(results), "output_dir": str(root)}, indent=2))
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, AssertionError) as exc:
        print(f"verify-compound-faults: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
