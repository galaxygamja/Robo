"""Repeat the image -> binding -> goal -> obstacle -> fake-wire safety pipeline.

Generated pixels and fixture dimensions ONLY; no physical camera, robot, sensor
feedback, pose integrator or network. Real user plans are never auto-reviewed.
Each run creates a NEW evidence directory and executes the normal runtime CLI.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robo_control.mission_bindings import profile_identity
from robo_control.runtime_report import inspect_report
from robo_control.vision.calibration import (
    COORDINATE_SYSTEM,
    FieldCalibration,
    vision_dependencies,
)
from robo_control.vision.tags import TagDetectorConfig

SCENARIOS = {"steady": "video_eof", "obstacle_lost": "object_obstacle_lost",
             "target_drift": "pickup_anchor_drift_exceeded"}
REVIEW = {"reviewed": True, "reviewed_by": "SYNTHETIC-VERIFIER",
          "evidence": "Generated pixel fixtures ONLY; no physical geometry or identity reviewed"}
FIXTURE_TOOL_FORWARD_MM = 150.


def write_json(path, value):
    with path.open("x", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write("\n")


def prepare_fixture(root, seconds):
    cv2, np = vision_dependencies()
    calibration = FieldCalibration((640, 480), ((0, 0), (639, 0), (639, 479), (0, 479)), (1143, 1181))
    calibration.save(root/"camera.json")
    mapping = {0: "H1", 1: "H2", 2: "B1", 3: "B2"}
    tags = TagDetectorConfig(tag_to_robot=mapping)
    tags.save(root/"tags.json")
    fleet = json.loads((ROOT/"config/qualifier_senior.json").read_text(encoding="utf-8-sig"))
    write_json(root/"fleet.json", fleet)
    colors = {"schema_version": 1, "profiles": [
        {"color": "red", "kind": "disc", "hsv_ranges": [
            [[0, 120, 100], [10, 255, 255]], [[170, 120, 100], [179, 255, 255]]],
            "min_area_mm2": 100, "max_area_mm2": 3000, "min_circularity": .3},
        {"color": "green", "kind": "cylinder", "hsv_ranges": [[[45, 120, 100], [85, 255, 255]]],
            "min_area_mm2": 100, "max_area_mm2": 3000, "min_circularity": .3}]}
    write_json(root/"colors.json", colors)
    # Every dimension below belongs to this generated test, NOT to a real robot.
    mission = {"schema_version": 2, "coordinate_system": COORDINATE_SYSTEM,
        "radii_mm": {rid: 20. for rid in mapping.values()}, "cell_mm": 40., "obstacles_mm": [],
        "tasks": [{"task_id": "move-D1", "pickup": {"mode": "observed_piece", "heading_rad": 0.,
            "tool_forward_mm": FIXTURE_TOOL_FORWARD_MM, "tool_left_mm": 0., "max_anchor_drift_mm": 20.,
            "replan_distance_mm": 5., "max_replans": 3},
            "drop": {"x_mm": 400., "y_mm": 840., "heading_rad": 0.},
            "retreat": {"x_mm": 300., "y_mm": 840., "heading_rad": math.pi}}]}
    write_json(root/"mission.json", mission)
    obstacle_plan = {"schema_version": 1, "coordinate_system": COORDINATE_SYSTEM, **REVIEW,
        "position_margin_mm": 3., "max_route_replans": 3,
        "footprints": [{"kind": p["kind"], "colour": p["color"], "radius_mm": 30.}
                       for p in colors["profiles"]]}
    write_json(root/"obstacles.json", obstacle_plan)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    base = np.full((480, 640), 255, dtype=np.uint8)
    for tag, (x, y) in enumerate(((100, 100), (420, 100), (100, 320), (420, 320))):
        base[y:y+70, x:x+70] = cv2.aruco.generateImageMarker(dictionary, tag, 70)
    base = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
    x_mm, y_mm = map(float, calibration.pixel_to_field_mm([(300, 240)])[0])
    profile = profile_identity(calibration.as_dict(), tags.as_dict(), colors)
    for scenario in SCENARIOS:
        video = root/f"{scenario}.avi"
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 20., (640, 480))
        if not writer.isOpened():
            raise RuntimeError("MJPG encoder required for synthetic verification")
        try:
            for index in range(math.ceil(seconds*20)):
                stamp = index/20
                frame = base.copy()
                # Gradual displacement preserves association until the reviewed
                # total anchor-drift limit is crossed; there is no pose feedback.
                shift = min(20, max(0, round((stamp-3.)*20))) if scenario == "target_drift" else 0
                cv2.circle(frame, (300+shift, 240), 10, (0, 0, 255), -1)
                if scenario != "obstacle_lost" or stamp < 3.:
                    cv2.circle(frame, (460, 240), 10, (0, 255, 0), -1)
                writer.write(frame)
        finally:
            writer.release()
        write_json(root/f"{scenario}-bindings.json", {
            "schema_version": 1, "coordinate_system": COORDINATE_SYSTEM,
            "field_size_mm": list(calibration.field_size_mm), "source_name": f"video:{video}",
            "is_replay": True, "observation_profile_id": profile, **REVIEW,
            "bindings": [{"piece_id": "D1", "kind": "disc", "colour": "red",
                "region_mm": {"x_mm": x_mm-25., "y_mm": y_mm-25., "width_mm": 50., "height_mm": 50.}}]})
    return {"opencv_version": cv2.__version__, "numpy_version": np.__version__,
            "python_version": sys.version, "fixture_seconds": seconds, "fixture_fps": 20.}


def verify_rows(path):
    """Extra cross-layer assertions in addition to the strict report reader."""
    moving_ticks = target_ticks = map_ticks = measured_frames = 0
    bound_tracks = set()
    with path.open(encoding="utf-8") as handle:
        header = json.loads(next(handle))
        for line in handle:
            row = json.loads(line)
            if row["event"] != "runtime_tick":
                continue
            wire_moving = any(r["receiver"]["v_mm_s"] or r["receiver"]["omega_rad_s"]
                              for r in row["wire"]["robots"])
            mission, bindings, obstacles = row["mission"], row["bindings"], row["object_obstacles"]
            if wire_moving:
                if (bindings["status"] != "bound" or obstacles["ready"] is not True
                        or mission["world"]["ready"] is not True or row["last_observation_age_ms"] >= 200):
                    raise AssertionError("Fake motion crossed an unready binding/map/observation gate")
                moving_ticks += 1
            map_ticks += obstacles["ready"] is True
            if obstacles["ready"] and len(obstacles["objects"]) != 2:
                raise AssertionError("Generated fixture must yield both distinct object footprints")
            if mission["observed_pickup_in_use"]:
                target = mission["observed_pickup"]
                point, goal = target["object_position_mm"], target["robot_goal"]
                if (not math.isclose(goal["x_mm"], point[0]-FIXTURE_TOOL_FORWARD_MM, abs_tol=1e-9)
                        or not math.isclose(goal["y_mm"], point[1], abs_tol=1e-9)
                        or goal["heading_rad"] != 0. or target["physical_pickup_verified"] is not False):
                    raise AssertionError("Measured pickup offset derivation changed")
                bound_tracks.add(target["track_id"])
                target_ticks += 1
            observation = row.get("observation")
            if observation and observation["status"] == "detected":
                measured = {r["robot_id"]: r["robot_center_mm"] for r in observation["robots"]}
                for track in observation["tracks"]:
                    if track["state"] == "observed" and track["robot_center_mm"] != measured[track["robot_id"]]:
                        raise AssertionError("Image-measured pose was replaced by generated motion")
                measured_frames += 1
    if not (moving_ticks and target_ticks and map_ticks and measured_frames and len(bound_tracks) == 1):
        raise AssertionError("The entire bound pickup/map/fake-drive pipeline was not exercised")
    return {"session_id": header["session_id"], "fake_motion_ticks": moving_ticks,
            "observed_target_ticks": target_ticks, "ready_obstacle_ticks": map_ticks,
            "image_measured_frames": measured_frames, "bound_target_track_count": len(bound_tracks),
            "no_pose_integrator": True, "no_sensor_feedback_injected": True}


def run_scenario(root, scenario, seconds):
    report = root/f"{scenario}.jsonl"
    command = [sys.executable, "-m", "robo_control.runtime", "--video", str(root/f"{scenario}.avi"),
        "--calibration", str(root/"camera.json"), "--tags", str(root/"tags.json"),
        "--fleet", str(root/"fleet.json"), "--mission", str(root/"mission.json"),
        "--colors", str(root/"colors.json"), "--bindings", str(root/f"{scenario}-bindings.json"),
        "--object-obstacles", str(root/"obstacles.json"), "--wire-fake",
        "--report", str(report), "--duration-s", str(seconds+10)]
    process = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
        encoding="utf-8", timeout=seconds+25, check=False)
    # A runtime safety stop deliberately exits 1. The verifier accepts ONLY the
    # scenario's exact expected reason, not any arbitrary error/nonzero status.
    with (root/f"{scenario}-process.json").open("x", encoding="utf-8") as handle:
        json.dump({"command": command, "returncode": process.returncode,
                   "stdout": process.stdout, "stderr": process.stderr}, handle, indent=2)
    try:
        runtime = json.loads(process.stdout)
    except ValueError as exc:
        raise AssertionError(f"{scenario}: runtime did not return a summary: {process.stderr[-2000:]}") from exc
    expected = SCENARIOS[scenario]
    expected_exit = 0 if scenario == "steady" else 1
    if process.returncode != expected_exit or runtime["status"] != expected:
        raise AssertionError(f"{scenario}: expected exit {expected_exit}/{expected}, got "
                             f"{process.returncode}/{runtime.get('status')}")
    audit = inspect_report(report)
    if (audit["closed_reason"] != expected or audit["final_wire_stop_acknowledged"] is not True
            or audit["final_wire_zero"] is not True or audit["unconfirmed_stop_robot_ids"]
            or audit["physical_stop_verified"] is not False or audit["mission"]["completed_tasks"]):
        raise AssertionError(f"{scenario}: invalid stop evidence or invented task/physical success")
    checks = verify_rows(report)
    if scenario == "obstacle_lost" and len(audit["object_obstacles"]["objects"]) != 2:
        raise AssertionError("Lost obstacle history was silently cleared")
    return {"scenario": scenario, "expected_reason": expected, "runtime": runtime,
            "report_analysis": audit, "checks": checks}


def run_observation_soak(root, seconds):
    """Longer streaming check with NO mission clock or movement authorization.

    Static tags cannot complete navigation, so a match-length observation soak
    is explicitly separate from the short bound-command scenarios above.
    """
    cv2, _ = vision_dependencies()
    reader = cv2.VideoCapture(str(root/"steady.avi"))
    try:
        success, frame = reader.read()
    finally:
        reader.release()
    if not success:
        raise RuntimeError("Cannot read the generated observation fixture")
    video, report = root/"observation-soak.avi", root/"observation-soak.jsonl"
    if video.exists():
        raise FileExistsError(video)
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 20., (640, 480))
    if not writer.isOpened():
        raise RuntimeError("MJPG encoder required for observation soak")
    try:
        for _ in range(math.ceil((seconds+3)*20)):
            writer.write(frame)
    finally:
        writer.release()
    command = [sys.executable, "-m", "robo_control.runtime", "--video", str(video),
        "--calibration", str(root/"camera.json"), "--tags", str(root/"tags.json"),
        "--fleet", str(root/"fleet.json"), "--mission", str(root/"mission.json"),
        "--colors", str(root/"colors.json"), "--object-obstacles", str(root/"obstacles.json"),
        "--mission-observe-only", "--report", str(report), "--duration-s", str(seconds)]
    process = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
        encoding="utf-8", timeout=seconds+25, check=False)
    write_json(root/"observation-soak-process.json", {"command": command, "returncode": process.returncode,
        "stdout": process.stdout, "stderr": process.stderr})
    try:
        runtime = json.loads(process.stdout)
    except ValueError as exc:
        raise AssertionError("Observation soak returned no summary: "+process.stderr[-2000:]) from exc
    audit = inspect_report(report)
    if (process.returncode != 0 or runtime["status"] != "duration_elapsed"
            or audit["mission_observe_only"] is not True or audit["transport_mode"] != "mock"
            or audit["mission"]["active_task_id"] is not None or audit["mission"]["elapsed_s"] != 0.
            or audit["mission"]["completed_tasks"] or audit["object_obstacles"]["fault"] is not None):
        raise AssertionError("Observation soak did not finish with unstarted mission and valid map")
    ready_ticks = 0
    with report.open(encoding="utf-8") as handle:
        next(handle)
        for line in handle:
            row = json.loads(line)
            if row["mission"]["active_task_id"] is not None or row["wire"] is not None:
                raise AssertionError("Observation-only soak started a task or wire session")
            if row["object_obstacles"]["ready"]:
                if len(row["object_obstacles"]["objects"]) != 2:
                    raise AssertionError("Observation soak lost a generated object footprint")
                ready_ticks += 1
    if ready_ticks == 0:
        raise AssertionError("Observation soak never obtained a ready object map")
    result = {"requested_duration_s": seconds, "runtime": runtime, "report_analysis": audit,
        "ready_obstacle_ticks": ready_ticks, "movement_authorized": False,
        "physical_camera_used": False, "physical_robot_used": False}
    write_json(root/"observation-soak-verification.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True, help="NEW directory; never overwrite")
    parser.add_argument("--seconds", type=float, default=12., help="each generated clip, 6..20 seconds")
    parser.add_argument("--observe-only-soak-s", type=float, default=0.,
                        help="optional separate stationary observation soak, 5..600 seconds; default off")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 6 <= args.seconds <= 20:
        parser.error("seconds must be finite and in [6, 20]; stationary tags do not complete navigation")
    if (not math.isfinite(args.observe_only_soak_s)
            or args.observe_only_soak_s != 0 and not 5 <= args.observe_only_soak_s <= 600):
        parser.error("observe-only-soak-s must be 0 (off) or finite and in [5, 600]")
    root = args.output_dir.resolve()
    try:
        root.mkdir(parents=True, exist_ok=False)
        versions = prepare_fixture(root, args.seconds)
        results = []
        for scenario in SCENARIOS:
            result = run_scenario(root, scenario, args.seconds)
            write_json(root/f"{scenario}-verification.json", result)
            results.append(result)
        if len({r["checks"]["session_id"] for r in results}) != len(SCENARIOS):
            raise AssertionError("Independent executions reused a runtime session ID")
        soak = run_observation_soak(root, args.observe_only_soak_s) if args.observe_only_soak_s else None
        evidence = {"schema_version": 1, "status": "passed", "fixture": "generated_static_tags_and_objects",
            "physical_camera_used": False, "physical_robot_used": False, "physical_stop_verified": False,
            "geometry_is_fixture_only": True, **versions, "scenarios": results, "observation_soak": soak}
        write_json(root/"verification.json", evidence)
        print(json.dumps({"status": "passed", "output_dir": str(root), **versions,
            "observe_only_soak_s": args.observe_only_soak_s,
            "scenarios": [{"scenario": r["scenario"], "closed_reason": r["expected_reason"],
                           **r["checks"]} for r in results]}, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, AssertionError, subprocess.TimeoutExpired) as exc:
        print(f"verify-mission-operations: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
