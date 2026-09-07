"""Generate a measured-image fixture and run the real runtime for one match.

This exercises video decoding, AprilTag detection, process isolation, tracking,
goal calculations, report writing and stop records. It does NOT simulate robot
movement and does not verify physical camera accuracy or motor operation.
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

from robo_control.runtime_report import inspect_report
from robo_control.vision.calibration import (
    FieldCalibration,
    vision_dependencies,
)
from robo_control.vision.lens import LensCalibration
from robo_control.vision.tags import TagDetectorConfig


def check_observation(observation, lens, moving):
    """A rejected frame has metadata but deliberately has no measured robots."""
    if lens and (observation.get("lens_correction_applied") is not True
                 or observation.get("lens_calibration_id") != lens.fingerprint):
        raise AssertionError("Lens profile was omitted or changed between process boundaries")
    if observation["status"] == "rejected_frame":
        if moving or observation.get("observation_usable") is True:
            raise AssertionError("Rejected camera input retained a usable/moving command")
        return
    if observation["status"] != "detected":
        raise AssertionError("Unexpected camera observation status")
    measured = {robot["robot_id"]: robot["robot_center_mm"] for robot in observation["robots"]}
    for track in observation["tracks"]:
        if track["state"] == "observed" and track["robot_center_mm"] != measured[track["robot_id"]]:
            raise AssertionError("Runtime replaced an image measurement with a generated pose")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True, help="new directory for reproducible evidence")
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--lens-fixture", action="store_true", help="nonlinear Brown5 distortion with embedded field profile")
    parser.add_argument("--colors-fixture", action="store_true", help="also exercise real HSV/object tracking on a red image marker")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 600:
        parser.error("seconds must be in [2, 600]")
    root = args.output_dir.resolve()
    root.mkdir(parents=True, exist_ok=False)
    cv2, np = vision_dependencies()
    calibration = root / "camera.json"
    tags = root / "tags.json"
    fleet = root / "fleet.json"
    goals = root / "goals.json"
    video = root / "four-tags-with-loss.avi"
    report = root / "runtime.jsonl"
    lens = (LensCalibration((640, 480), ((500., 0., 320.), (0., 510., 240.), (0., 0., 1.)),
            (-.15, .03, .001, -.001, 0.), "GENERATED-lens-fixture-NOT-physical") if args.lens_fixture else None)
    corners = ((0, 0), (639, 0), (639, 479), (0, 479))
    if lens:
        lens.save(root / "lens-fixture.json")
        corners = lens.distort_points(corners)
    FieldCalibration((640, 480), corners, (1143, 1181), lens).save(calibration)
    mapping = {0: "H1", 1: "H2", 2: "B1", 3: "B2"}
    TagDetectorConfig(tag_to_robot=mapping).save(tags)
    fleet.write_text(json.dumps({"ground_robots": [
        {"id": rid, "tag_id": tag, "role": "hamster" if rid == "H1" else "beaver"}
        for tag, rid in mapping.items()]}), encoding="utf-8")
    goals.write_text(json.dumps({"schema_version": 1,
        "coordinate_system": "bottom_left_x_right_y_up_mm",
        "goals": {"H1": {"x_mm": 400, "y_mm": 840}},
        "radii_mm": {rid: 40 for rid in mapping.values()}}), encoding="utf-8")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    complete = np.full((480, 640), 255, dtype=np.uint8)
    for tag, (x, y) in enumerate(((100, 100), (420, 100), (100, 320), (420, 320))):
        complete[y:y + 70, x:x + 70] = cv2.aruco.generateImageMarker(dictionary, tag, 70)
    lost = complete.copy()
    lost[100:170, 100:170] = 255
    complete, lost = [cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) for image in (complete, lost)]
    color_path = root / "colors.json"
    if args.colors_fixture:
        for frame in (complete, lost):
            cv2.circle(frame, (320, 240), 8, (0, 0, 255), -1)
        color_path.write_text(json.dumps({"schema_version": 1, "profiles": [{"color": "red", "kind": "cylinder",
            # Red straddles the hue boundary; MJPG can turn pure red into H=179.
            "hsv_ranges": [[[0, 120, 100], [10, 255, 255]], [[170, 120, 100], [179, 255, 255]]], "min_area_mm2": 100,
            "max_area_mm2": 3000, "min_circularity": .5}]}), encoding="utf-8")
    if lens:
        x, y = np.meshgrid(np.arange(640), np.arange(480))
        points = lens.undistort_points(np.column_stack((x.ravel(), y.ravel()))).reshape(480, 640, 2)
        complete, lost = [cv2.remap(frame, points[:, :, 0].astype(np.float32), points[:, :, 1].astype(np.float32),
            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(255, 255, 255)) for frame in (complete, lost)]
    fps = 20.
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), fps, (640, 480))
    if not writer.isOpened():
        raise RuntimeError("MJPG encoder is required")
    try:
        # Longer than requested runtime, so shutdown interrupts a still-open
        # source. H1 disappears for 0.6 seconds every 10 seconds.
        for index in range(math.ceil((args.seconds + 3) * fps)):
            stamp = index / fps
            missing = 4.0 <= stamp % 10.0 < 4.6
            writer.write(lost if missing else complete)
    finally:
        writer.release()
    command = [sys.executable, "-m", "robo_control.runtime", "--video", str(video),
               "--calibration", str(calibration), "--tags", str(tags), "--fleet", str(fleet),
               "--goals", str(goals), "--report", str(report), "--duration-s", str(args.seconds)]
    if args.colors_fixture:
        command.extend(("--colors", str(color_path)))
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, encoding="utf-8",
                            timeout=args.seconds + 20, check=False)
    if result.returncode != 0:
        print(result.stderr or result.stdout, file=sys.stderr)
        return 1
    summary = json.loads(result.stdout)
    audit = inspect_report(report)
    checked = checked_motion = loss_events = recovery_events = 0
    previous_streak = 0
    confirmed_object_ticks = 0
    with report.open(encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            if row["event"] != "runtime_tick":
                continue
            moving = any(any(value != 0 for value in (*robot["velocity_world_mm_s"],
                         robot["angular_velocity_rad_s"], *robot["wheel_velocity_rad_s"]))
                         for robot in row["actuator"]["robots"])
            streak = row["fresh_streak"]
            if moving and (streak < row["recovery_frames"] or row["last_observation_age_ms"] >= 200):
                raise AssertionError("Unconfirmed or expired input retained a nonzero command")
            observation = row.get("observation")
            if observation:
                check_observation(observation, lens, moving)
                loss_events += observation["observation_usable"] is not True
            if row.get("object_tracking"):
                confirmed_object_ticks += any(track["valid_for_pick"] for track in row["object_tracking"]["object_tracks"])
            recovery_events += previous_streak < row["recovery_frames"] <= streak
            previous_streak = streak
            checked += 1
            checked_motion += moving
    checks = {"ticks_checked": checked, "nonzero_dry_run_ticks": checked_motion,
              "unusable_observations": loss_events, "confirmed_recoveries": recovery_events,
              "confirmed_object_ticks": confirmed_object_ticks,
              "all_motion_fresh_and_confirmed": True, "no_synthetic_pose_replacement": True}
    evidence = {"fixture": "generated_static_AprilTags_with_periodic_H1_loss",
                "physical_camera_used": False, "physical_robot_used": False,
                "lens_fixture": args.lens_fixture, "colors_fixture": args.colors_fixture,
                "requested_duration_s": args.seconds, "runtime": summary,
                "report_analysis": audit, "invariant_checks": checks}
    (root / "verification.json").write_text(json.dumps(evidence, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"output_dir": str(root), "status": summary["status"], "analysis": audit, "checks": checks},
                     ensure_ascii=True, indent=2, allow_nan=False))
    return 0 if (summary["status"] == "duration_elapsed" and checked_motion > 0
                 and (not args.colors_fixture or confirmed_object_ticks > 0)
                 and (args.seconds < 8 or (loss_events > 0 and recovery_events >= 2))) else 1


if __name__ == "__main__":
    raise SystemExit(main())
