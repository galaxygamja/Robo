"""Run real camera measurements through tracking and dry-run command calculation.

The camera/CV worker is isolated from host supervision. No simulated ground
truth, robot motion integrator, network transmitter or motor driver is used.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import multiprocessing
import sys
import time
from pathlib import Path

from .adapters import OpenCVCameraSource, VideoFileSource
from .fleet import validate_tag_registry
from .mission_bindings import load_binding_plan, load_review_json, profile_identity
from .runtime_io import AsyncJsonlReport, LatestRecordMailbox
from .runtime_session import LiveControlSession
from .vision.calibration import COORDINATE_SYSTEM, FieldCalibration
from .vision.colors import ColorDetector
from .vision.detection import DetectionPipeline
from .vision.tags import TagDetectorConfig, default_tag_config_path


def _positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--camera", type=int, help="fixed USB camera index")
    source.add_argument("--video", type=Path, help="local video; paced replay, no physical output")
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--tags", type=Path, default=default_tag_config_path())
    parser.add_argument("--fleet", type=Path, required=True, help="mission ground_robots registry")
    execution = parser.add_mutually_exclusive_group()
    execution.add_argument("--goals", type=Path, help="reviewed field-mm goals JSON; omitted: all hold position")
    execution.add_argument("--mission", type=Path, help="reviewed qualifier task/robot-pose plan; differential dry-run")
    parser.add_argument("--wire-fake", action="store_true",
                        help="connect --mission to framed fake receivers; no network or hardware")
    parser.add_argument("--colors", type=Path, help="optional HSV profile, uses existing object tracker")
    parser.add_argument("--bindings", type=Path,
                        help="reviewed startup object identity regions; requires --mission and --colors")
    parser.add_argument("--mission-observe-only", action="store_true",
                        help="record mission world/object IDs without starting tasks or fake-wire")
    parser.add_argument("--object-obstacles", type=Path,
                        help="reviewed object footprints for live collision map; requires --mission and --colors")
    parser.add_argument("--report", type=Path, required=True, help="NEW JSONL file, never overwritten")
    parser.add_argument("--duration-s", type=_positive, default=120.0)
    parser.add_argument("--tick-hz", type=_positive, default=50.0)
    parser.add_argument("--startup-timeout-s", type=_positive, default=10.0)
    parser.add_argument("--recovery-frames", type=int, default=3)
    parser.add_argument("--moving-camera", action="store_true", help="unsupported: requires dynamic calibration")
    return parser


def load_goals(path, roles, calibration):
    if path is None:
        return {}, None
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if (not isinstance(data, dict) or type(data.get("schema_version")) is not int
            or data["schema_version"] != 1 or data.get("coordinate_system") != COORDINATE_SYSTEM
            or not {"schema_version", "coordinate_system", "goals"} <= set(data)
            or not set(data) <= {"schema_version", "coordinate_system", "goals", "radii_mm"}):
        raise ValueError("Goals require schema_version=1, coordinate_system and goals; optional radii_mm")
    if not isinstance(data["goals"], dict):
        raise TypeError("goals must be an object mapping registered IDs to field-mm targets")
    # The same constructor used at runtime checks finite coordinates, complete
    # radii, registered IDs and body clearance from all four field boundaries.
    from .control_loop import ClosedLoopController
    checked = ClosedLoopController(roles=roles, goals=data["goals"],
        radii_mm=data.get("radii_mm"), field_size_mm=calibration.field_size_mm)
    return checked.goals, checked.radii_mm


def camera_worker(options, mailbox, stop, worker_status):
    """Spawn entry point; a stuck read/detector can be terminated independently."""
    source = None
    try:
        # Receive the already-validated snapshot, not mutable configuration
        # paths. Editing a JSON file after launch cannot change this session.
        calibration = FieldCalibration.from_dict(options["calibration"])
        tags = TagDetectorConfig(**options["tags"])
        validate_tag_registry(options["fleet"], tags.tag_to_robot)
        colors = ColorDetector(calibration, options["colors"]) if options.get("colors") else None
        pipeline = DetectionPipeline(calibration, tags, colors=colors, registry_checked=True)
        source = VideoFileSource(options["video"]) if options.get("video") else OpenCVCameraSource(options["camera"])
        period = 0.0
        expected_frames = None
        if options.get("video"):
            fps = source.nominal_fps
            expected_frames = source.nominal_frame_count
            if fps is None:
                raise ValueError("Paced video runtime requires valid nominal FPS metadata")
            period = 1.0 / fps
        worker_status.value = 1  # source opened; detector/capture can still block
        while not stop.is_set():
            iteration_at = time.monotonic()
            frame = source.read()
            if frame is None:
                if not options.get("video"):
                    code, reason = 3, "camera_read_failed"
                elif expected_frames is not None and source.sequence >= expected_frames:
                    code, reason = 2, "video_eof"
                else:
                    code, reason = 5, "video_decode_failed_or_unverified_end"
                worker_status.value = code
                mailbox.publish({"status": "source_closed", "reason": reason,
                                 "decoded_frames": source.sequence, "nominal_frames": expected_frames})
                return
            result = pipeline.process(frame)
            result.record["configuration_id"] = options["configuration_id"]
            mailbox.publish(result.record)
            # Replay pacing waits BEFORE the next read, preserving honest host
            # decode timestamps. Media time remains the kinematic clock.
            if period:
                stop.wait(max(0.0, period - (time.monotonic() - iteration_at)))
    except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - worker failure must stop the parent
        worker_status.value = 3
        try:
            mailbox.publish({"status": "source_closed", "reason": "camera_worker_failed",
                             "error": f"{type(exc).__name__}: {exc}"[:1000]})
        except (OSError, ValueError, TypeError):
            worker_status.value = 4  # parent detects death even when reporting failed
    finally:
        if source is not None:
            source.close()


def supervise(session, worker, mailbox, stop, worker_status, report, *, duration_s,
              tick_hz=50.0, startup_timeout_s=10.0):
    """Finite supervisor loop. Never calls read(), CV, GUI or file writes."""
    period = 1.0 / tick_hz
    started = time.monotonic()
    next_tick = started
    revision = delivered = replaced = detected = usable = commanded = 0
    final = None
    error = None
    reason = "duration_elapsed"
    try:
        while True:
            now = time.monotonic()
            if now >= started + duration_s:
                break
            if report.error is not None:
                reason = "report_failed"
                break
            if session.closed_reason:
                reason = session.closed_reason
                break
            if now >= started + startup_timeout_s and delivered == 0:
                reason = "camera_startup_timeout"
                break
            value = mailbox.poll(revision)
            record = None
            if value is not None:
                new_revision, record = value
                replaced += new_revision - revision - 1
                revision = new_revision
                if record.get("status") == "source_closed":
                    reason = record.get("reason", "source_closed")
                    error = record.get("error")
                    break
                delivered += 1
                detected += record.get("status") == "detected"
            if worker_status.value in (2, 3, 4, 5):
                # A driver can also hang in close()/release(), after EOF or
                # failure was reported. Do not wait for process exit to stop.
                reason = {2: "video_eof", 5: "video_decode_failed_or_unverified_end"}.get(
                    worker_status.value, "camera_worker_failed")
                break
            # Check death regardless of mailbox state, including a producer
            # that died while holding its lock or before publishing an error.
            if not worker.is_alive():
                reason = "video_eof" if worker_status.value == 2 else "camera_worker_exited"
                break
            event = session.advance(record, time.monotonic())
            finished_at = time.monotonic()
            # Check the computation itself too. A slow controller must not
            # publish a movement record using only its start-of-tick age.
            if finished_at - event["at_s"] > session.max_tick_gap_s:
                reason = "supervisor_deadline_missed"
                break
            if (session.last_capture_s is not None
                    and finished_at >= session.last_capture_s + session.controller.limits.max_pose_age_s
                    and event["actuator"]["reason"] == "mock_active"):
                event = session.advance(None, finished_at)
            if session.closed_reason is not None:
                # advance() can latch shutdown on a scheduler gap. Emit the
                # single terminal event in finally, not two closed records.
                reason = session.closed_reason
                break
            if event["observation"] is not None:
                usable += event["observation"].get("observation_usable") is True
            commanded += event["command"] is not None
            if not report.submit(event):
                reason = "report_failed"
                break
            next_tick += period
            now = time.monotonic()
            next_tick = max(next_tick, now)  # never manufacture missed control ticks
            stop.wait(max(0.0, next_tick - now))
            if stop.is_set():
                reason = "operator_stop"
                break
    except KeyboardInterrupt:
        reason = "operator_stop"
    except Exception as exc:  # noqa: BLE001 - stop before any error leaves supervision
        reason, error = "runtime_failed", f"{type(exc).__name__}: {exc}"
    finally:
        # Stop the actuator before process cleanup, logging or stdout can wait.
        final = session.close(max(session.now or 0.0, time.monotonic()), reason)
        stop.set()
        report.submit(final)
        worker.join(timeout=0.5)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=0.5)
        if worker.is_alive():
            worker.kill()
            worker.join(timeout=0.5)
        report_complete = report.finish()
    return {"status": reason, "error": error, "elapsed_s": time.monotonic() - started,
            "delivered_frames": delivered, "replaced_frames": replaced,
            "detected_frames": detected, "usable_observation_frames": usable,
            "command_packets": commanded,
            "ticks": session.tick_sequence, "worker_stopped": not worker.is_alive(),
            "report_complete": report_complete, "report_error": report.error,
            "configuration_id": session.configuration_id,
            "input_mode": "video_replay" if session.is_replay else "live_camera",
            "output_mode": "dry_run_commands",
            "transport_mode": session.transport_mode,
            "wire_fault": session.wire.fault if session.wire else None,
            "wire_stop_acknowledged": (all(r["stop_acknowledged"] for r in final["wire"]["robots"])
                                       if session.wire else None),
            "final": final, "device_io": False, "hardware_ready": False, "motion_permitted": False}


def main(argv=None):
    args = _parser().parse_args(argv)
    worker = report = None
    try:
        if args.moving_camera:
            raise ValueError("Moving camera needs per-frame dynamic calibration; static runtime is fixed-camera only")
        if args.wire_fake and args.mission is None:
            raise ValueError("--wire-fake requires --mission; legacy mecanum output is not a wire command")
        if args.bindings is not None and (args.mission is None or args.colors is None):
            raise ValueError("--bindings requires --mission and --colors")
        if args.object_obstacles is not None and (args.mission is None or args.colors is None):
            raise ValueError("--object-obstacles requires --mission and --colors")
        if args.camera is not None and args.camera < 0:
            raise ValueError("Camera index must be nonnegative")
        if not 20 <= args.tick_hz <= 100:
            raise ValueError("Supervisor tick-hz must be in [20, 100]")
        inputs = [args.calibration, args.tags, args.fleet, args.goals, args.mission, args.colors,
                  args.bindings, args.object_obstacles, args.video]
        if args.report.resolve() in {p.resolve() for p in inputs if p is not None}:
            raise ValueError("Report must be distinct from every input")
        calibration = FieldCalibration.load(args.calibration)
        tags = TagDetectorConfig.load(args.tags)
        fleet_data = json.loads(args.fleet.read_text(encoding="utf-8-sig"))
        roles = validate_tag_registry(fleet_data, tags.tag_to_robot)
        colors = None
        if args.colors:
            colors = json.loads(args.colors.read_text(encoding="utf-8-sig"))
            ColorDetector(calibration, colors)
        if args.video and not args.video.is_file():
            raise ValueError("Video must be an existing local file")
        goals, radii = load_goals(args.goals, roles, calibration)
        mission_plan = load_review_json(args.mission, name="Mission") if args.mission else None
        binding_plan = load_binding_plan(args.bindings) if args.bindings else None
        obstacle_plan = load_review_json(args.object_obstacles, name="Object obstacle") if args.object_obstacles else None
        observation_profile_id = profile_identity(calibration.as_dict(), tags.as_dict(), colors)
        options = {"calibration": calibration.as_dict(),
                   "tags": {"dictionary_name": tags.dictionary_name, "tag_to_robot": dict(tags.tag_to_robot),
                       "heading_offsets_rad": dict(tags.heading_offsets_rad),
                       "robot_center_from_tag_mm": dict(tags.robot_center_from_tag_mm),
                       "allowed_margin_mm": tags.allowed_margin_mm, "tag_size_mm": tags.tag_size_mm,
                       "tag_size_tolerance_fraction": tags.tag_size_tolerance_fraction,
                       "hardware_verified": tags.hardware_verified},
                   "fleet": fleet_data, "colors": colors, "mission": mission_plan,
                   "bindings": binding_plan,
                   "object_obstacles": obstacle_plan,
                   "mission_observe_only": args.mission_observe_only,
                   "transport_mode": "fake_wire" if args.wire_fake else "mock",
                   "video": str(args.video.resolve()) if args.video else None, "camera": args.camera}
        configuration_id = hashlib.sha256(json.dumps(options, sort_keys=True, allow_nan=False).encode()).hexdigest()
        options["configuration_id"] = configuration_id
        session = LiveControlSession(roles=roles, field_size_mm=calibration.field_size_mm,
            source_name=f"video:{args.video.resolve()}" if args.video else f"webcam:{args.camera}",
            is_replay=args.video is not None, goals=goals, radii_mm=radii,
            recovery_frames=args.recovery_frames, track_objects=args.colors is not None,
            configuration_id=configuration_id, mission_plan=mission_plan, fleet=fleet_data,
            wire_fake=args.wire_fake, binding_plan=binding_plan,
            observation_profile_id=observation_profile_id, mission_observe_only=args.mission_observe_only,
            obstacle_plan=obstacle_plan)
        if session.bindings:
            configured_classes = {(p["kind"], p["color"]) for p in colors["profiles"]}
            if any((e["kind"], e["colour"]) not in configured_classes
                   for e in session.bindings.plan["bindings"]):
                raise ValueError("Every binding kind/colour needs an enabled colour detector profile")
        if session.object_obstacles:
            configured_classes = {(p["kind"], p["color"]) for p in colors["profiles"]}
            obstacle_classes = {(p["kind"], p["colour"]) for p in session.object_obstacles.plan["footprints"]}
            if not configured_classes <= obstacle_classes:
                raise ValueError("Every enabled detector class needs a reviewed obstacle footprint")
        context = multiprocessing.get_context("spawn")
        mailbox, stop = LatestRecordMailbox(context), context.Event()
        worker_status = context.RawValue("i", 0)
        report = AsyncJsonlReport(args.report)
        report.submit({"schema_version": 1, "event": "session_started",
            "session_id": session.controller.session_id, "configuration_id": configuration_id,
            "calibration": calibration.as_dict(), "tags": tags.as_dict(), "roles": roles,
            "goals": session.controller.goals, "radii_mm": session.controller.radii_mm,
            "drive_model": session.controller.drive_model, "mission_plan": mission_plan,
            "binding_plan": binding_plan, "observation_profile_id": observation_profile_id,
            "mission_observe_only": args.mission_observe_only,
            "obstacle_plan": obstacle_plan,
            "control_limits": vars(session.controller.limits), "tick_hz": args.tick_hz,
            "max_tick_gap_s": session.max_tick_gap_s, "recovery_frames": args.recovery_frames,
            "source_name": session.source_name, "is_replay": session.is_replay,
            "input_mode": "video_replay" if session.is_replay else "live_camera",
            "output_mode": "dry_run_commands", "transport_mode": session.transport_mode,
            "device_io": False, "motion_permitted": False})
        worker = context.Process(target=camera_worker, args=(options, mailbox, stop, worker_status),
                                 name="robo-camera-detector", daemon=True)
        worker.start()
        summary = supervise(session, worker, mailbox, stop, worker_status, report,
            duration_s=args.duration_s, tick_hz=args.tick_hz, startup_timeout_s=args.startup_timeout_s)
        print(json.dumps(summary, ensure_ascii=True, allow_nan=False))
        successful_end = summary["status"] in {"duration_elapsed", "operator_stop", "video_eof", "mission_completed"}
        transport_ok = session.wire is None or (not summary["wire_fault"] and summary["wire_stop_acknowledged"])
        return 0 if (successful_end and summary["detected_frames"] > 0 and summary["report_complete"]
                     and summary["worker_stopped"] and transport_ok) else 1
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        if report is not None:
            report.finish()
        print(f"runtime: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
