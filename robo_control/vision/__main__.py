"""Real image calibration CLI; no robot transport is created here."""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from contextlib import ExitStack
from pathlib import Path

from ..adapters import CameraFrame, OpenCVCameraSource, VideoFileSource
from ..fleet import validate_tag_registry
from .calibration import CORNER_NAMES, FieldCalibration, vision_dependencies
from .pipeline import FrameProcessor, FrameRejected
from .tags import AprilTagDetector, TagDetectorConfig, TagDetectionError, default_tag_config_path
from .tracking import PoseTracker
from .colors import ColorDetector
from .map_view import draw_position_map


def _positive_int(value: str) -> int:
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be >= 1")
    return number


def _source_options(parser: argparse.ArgumentParser, images: bool = False) -> None:
    source = parser.add_mutually_exclusive_group(required=True)
    if images:
        source.add_argument("--image", type=Path, help="local still image")
    source.add_argument("--video", type=Path, help="local recording (offline replay)")
    source.add_argument("--camera", type=int, help="USB camera index, e.g. 0")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Camera input and planar field calibration in mm")
    commands = parser.add_subparsers(dest="command", required=True)
    calibrate = commands.add_parser("calibrate", help="label four corners and save calibration JSON")
    _source_options(calibrate, images=True)
    calibrate.add_argument("--corners", nargs=8, type=float, metavar="PX",
                           help="TLx TLy TRx TRy BRx BRy BLx BLy; omitted: click in a window")
    calibrate.add_argument("--field-size-mm", nargs=2, type=float, default=(1143.0, 1181.0), metavar=("WIDTH", "HEIGHT"))
    calibrate.add_argument("--output", type=Path, required=True, help="calibration JSON")
    calibrate.add_argument("--preview-output", type=Path, help="save rectified PNG/JPEG")

    run = commands.add_parser("run", help="rectify camera/video frames and check freshness")
    _source_options(run)
    run.add_argument("--calibration", type=Path, required=True)
    run.add_argument("--frames", type=_positive_int, default=100, help="maximum input frames (default: 100)")
    run.add_argument("--max-age-ms", type=float, default=200.0)
    run.add_argument("--pixels-per-mm", type=float, default=1.0)
    run.add_argument("--preview", action="store_true", help="display rectified frames; Q/Esc to stop")
    run.add_argument("--output", type=Path, help="save last accepted rectified image")
    run.add_argument("--report", type=Path, help="write accepted/rejected frame metadata as JSONL")

    detect = commands.add_parser("detect", help="detect configured robot AprilTags in real images")
    _source_options(detect, images=True)
    detect.add_argument("--calibration", type=Path, required=True)
    detect.add_argument("--tags", type=Path, default=default_tag_config_path(), help="robot tag JSON")
    detect.add_argument("--fleet", type=Path, help="cross-check mission ground_robots IDs/roles/tag_id against tag config")
    detect.add_argument("--frames", type=_positive_int, default=100)
    detect.add_argument("--max-age-ms", type=float, default=200.0)
    detect.add_argument("--preview", action="store_true", help="display annotated original frames")
    detect.add_argument("--output", type=Path, help="save the last annotated frame")
    detect.add_argument("--report", type=Path, help="write one detection record per frame as JSONL")
    detect.add_argument("--require-complete-observation", action="store_true",
                        help="exit 1 unless every processed frame sees all configured robots exactly once")
    detect.add_argument("--track", action="store_true", help="append measured-pose history and fail-closed freshness gates")
    detect.add_argument("--colors", type=Path, help="HSV/metric contour profile JSON; enables colour candidates")
    detect.add_argument("--moving-camera", action="store_true", help="reject static calibration for moving cameras (dynamic calibration not yet implemented)")

    check = commands.add_parser("check", help="measure independently surveyed field landmarks")
    check.add_argument("--calibration", type=Path, required=True)
    check.add_argument("--points", type=Path, required=True,
                       help='JSON object with "pixel_points" and "expected_mm" arrays')
    check.add_argument("--max-error-mm", type=float, default=15.0)
    return parser


def _distinct_paths(args: argparse.Namespace) -> None:
    inputs = [getattr(args, key, None) for key in ("image", "video", "calibration", "points", "tags", "colors", "fleet")]
    outputs = [getattr(args, key, None) for key in ("output", "preview_output", "report")]
    input_paths = {p.resolve() for p in inputs if p is not None}
    output_paths = [p.resolve() for p in outputs if p is not None]
    if len(set(output_paths)) != len(output_paths) or input_paths.intersection(output_paths):
        raise ValueError("input and output paths must be distinct")


def _read_image(path: Path):
    cv2, np = vision_dependencies()
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"could not decode image: {path}")
    return image


def _write_image(path: Path, image) -> None:
    cv2, _ = vision_dependencies()
    if path.suffix.lower() not in (".png", ".jpg", ".jpeg"):
        raise ValueError("image output must end in .png, .jpg or .jpeg")
    ok, encoded = cv2.imencode(path.suffix.lower(), image)
    if not ok:
        raise RuntimeError(f"image encoder failed: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded.tobytes())


def _open_source(args: argparse.Namespace):
    if args.video is not None:
        return VideoFileSource(args.video)
    if args.camera is None or args.camera < 0:
        raise ValueError("camera index must be >= 0")
    return OpenCVCameraSource(args.camera)


def _pick_corners(image):
    cv2, _ = vision_dependencies()
    points: list[tuple[int, int]] = []
    window = "Field corners: TL TR BR BL | Enter=save R=reset Esc=cancel"
    height, width = image.shape[:2]
    scale = min(1.0, 1000 / width, 650 / height)
    shown_width, shown_height = max(2, round(width * scale)), max(2, round(height * scale))
    reference = cv2.resize(image, (shown_width, shown_height), interpolation=cv2.INTER_AREA)

    def clicked(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((x, y))

    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, clicked)
    try:
        while True:
            display = reference.copy()
            for index, point in enumerate(points):
                cv2.circle(display, point, 5, (0, 255, 255), -1)
                cv2.putText(display, CORNER_NAMES[index], point,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imshow(window, display)
            key = cv2.waitKey(20) & 0xFF
            if key in (10, 13) and len(points) == 4:
                # OpenCV resize maps pixel centres, not endpoint coordinates.
                return tuple(((x + 0.5) * width / shown_width - 0.5,
                              (y + 0.5) * height / shown_height - 0.5) for x, y in points)
            if key in (ord("r"), ord("R")):
                points.clear()
            if key in (27, ord("q")) or _window_closed(window):
                raise ValueError("calibration cancelled")
    finally:
        try:
            cv2.destroyWindow(window)
        except cv2.error:
            pass  # The user may already have closed the window.


def _window_closed(window: str) -> bool:
    cv2, _ = vision_dependencies()
    try:
        return cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def _calibrate(args: argparse.Namespace) -> int:
    if args.image is not None:
        image = _read_image(args.image)
    else:
        with _open_source(args) as source:
            frame = source.read()
            if frame is None:
                raise RuntimeError("source returned no calibration frame")
            image = frame.image
    corners = (tuple(zip(args.corners[::2], args.corners[1::2]))
               if args.corners is not None else _pick_corners(image))
    calibration = FieldCalibration((image.shape[1], image.shape[0]), corners, tuple(args.field_size_mm))
    # Generate the preview first; malformed images/geometry must not save a config.
    rectified = calibration.warp(image)
    if args.preview_output is not None:
        _write_image(args.preview_output, rectified)
    calibration.save(args.output)
    print(json.dumps({"status": "calibrated", "output": str(args.output),
                      "calibration": calibration.as_dict(), "physical_accuracy_verified": False}))
    return 0


def _run(args: argparse.Namespace) -> int:
    cv2, _ = vision_dependencies()
    calibration = FieldCalibration.load(args.calibration)
    processor = FrameProcessor(calibration, args.max_age_ms / 1000.0, args.pixels_per_mm)
    accepted = 0
    rejected: Counter[str] = Counter()
    last = None
    status = "frame_limit"
    preview_window = "Rectified field (top-left pixels, bottom-left field mm)"
    preview_created = False
    with ExitStack() as stack:
        source = stack.enter_context(_open_source(args))
        report = None
        if args.report is not None:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            report = stack.enter_context(args.report.open("w", encoding="utf-8"))
        if args.preview:
            stack.callback(cv2.destroyAllWindows)
        for _ in range(args.frames):
            frame = source.read()
            if frame is None:
                status = "eof_or_decode_failure" if args.video is not None else "camera_read_failed"
                break
            record = {"sequence": frame.sequence, "source_name": frame.source_name,
                      "captured_at_s": frame.captured_at_s, "received_at_s": frame.received_at_s,
                      "media_time_s": frame.media_time_s, "timestamp_basis": frame.timestamp_basis,
                      "is_replay": frame.is_replay}
            try:
                result = processor.process(frame)
            except FrameRejected as exc:
                rejected[exc.reason] += 1
                record.update(status="rejected", reason=exc.reason)
            else:
                accepted += 1
                last = result.image
                record.update(status="accepted", processed_at_s=result.processed_at_s,
                              host_age_ms=result.age_s * 1000.0)
                if args.preview:
                    if not preview_created:
                        cv2.namedWindow(preview_window, cv2.WINDOW_NORMAL)
                        display_scale = min(1.0, 1000 / last.shape[1], 650 / last.shape[0])
                        cv2.resizeWindow(preview_window, max(2, round(last.shape[1] * display_scale)),
                                         max(2, round(last.shape[0] * display_scale)))
                        preview_created = True
                    cv2.imshow(preview_window, last)
            if report is not None:
                report.write(json.dumps(record, allow_nan=False) + "\n")
            if args.preview:
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")) or (preview_created and _window_closed(preview_window)):
                    status = "user_stopped"
                    break
    if args.output is not None and last is not None:
        _write_image(args.output, last)
    print(json.dumps({"status": status, "accepted_frames": accepted,
                      "rejected_frames": sum(rejected.values()), "rejection_reasons": dict(rejected),
                      "is_replay": args.video is not None}))
    return 0 if accepted and not rejected and status != "camera_read_failed" else 1


def _check(args: argparse.Namespace) -> int:
    import math

    if not math.isfinite(args.max_error_mm) or args.max_error_mm <= 0:
        raise ValueError("max-error-mm must be finite and positive")
    points = json.loads(args.points.read_text(encoding="utf-8-sig"))
    if not isinstance(points, dict) or not {"pixel_points", "expected_mm"} <= points.keys():
        raise ValueError("points JSON requires pixel_points and expected_mm")
    result = FieldCalibration.load(args.calibration).check_points(points["pixel_points"], points["expected_mm"])
    passed = result["max_error_mm"] <= args.max_error_mm
    print(json.dumps({**result, "max_allowed_mm": args.max_error_mm, "passed": passed}))
    return 0 if passed else 1


def _still_frame(path: Path) -> CameraFrame:
    before = time.monotonic()
    image = _read_image(path)
    after = time.monotonic()
    return CameraFrame(
        image=image,
        captured_at_s=before,
        received_at_s=after,
        sequence=1,
        source_name=f"image:{path.resolve()}",
        timestamp_basis="host_file_decode_start",
        is_replay=True,
    )


def _detect(args: argparse.Namespace) -> int:
    if args.moving_camera:
        raise ValueError("Moving camera requires per-frame field references; saved static calibration is unsafe. Drone simulation is not a live camera adapter.")
    cv2, _ = vision_dependencies()
    calibration = FieldCalibration.load(args.calibration)
    tag_config = TagDetectorConfig.load(args.tags)
    if args.fleet is not None:
        validate_tag_registry(json.loads(args.fleet.read_text(encoding="utf-8-sig")), tag_config.tag_to_robot)
    detector = AprilTagDetector(tag_config, calibration)
    processor = FrameProcessor(calibration, args.max_age_ms / 1000.0)
    tracker = PoseTracker(tag_config.tag_to_robot.values()) if args.track else None
    color_detector = ColorDetector.load(calibration, args.colors) if args.colors else None
    processed = complete = detected = total = 0
    unknown = duplicates = missing_frames = rejected_frames = 0
    rejection_reasons: Counter[str] = Counter()
    last = None
    status = "image_complete" if args.image is not None else "frame_limit"
    preview_window = "Robot AprilTags | marker +X arrow shows measured heading"
    preview_created = False
    with ExitStack() as stack:
        source = None if args.image is not None else stack.enter_context(_open_source(args))
        report = None
        if args.report is not None:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            report = stack.enter_context(args.report.open("w", encoding="utf-8"))
        if args.preview:
            stack.callback(cv2.destroyAllWindows)
        frame_limit = 1 if args.image is not None else args.frames
        for _ in range(frame_limit):
            frame = _still_frame(args.image) if args.image is not None else source.read()
            if frame is None:
                status = "eof_or_decode_failure" if args.video is not None else "camera_read_failed"
                break
            record: dict[str, object] = {
                "sequence": frame.sequence,
                "source_name": frame.source_name,
                "captured_at_s": frame.captured_at_s,
                "received_at_s": frame.received_at_s,
                "media_time_s": frame.media_time_s,
                "timestamp_basis": frame.timestamp_basis,
                "is_replay": frame.is_replay,
                "dictionary_name": tag_config.dictionary_name,
                "tag_size_mm": tag_config.tag_size_mm,
                "hardware_verified": tag_config.hardware_verified,
                "coordinate_system": "bottom_left_x_right_y_up_mm",
                "field_size_mm": list(calibration.field_size_mm),
                "registered_robot_ids": sorted(tag_config.tag_to_robot.values()),
                "mission_registry_checked": args.fleet is not None,
                "device_io": False,
            }
            try:
                processor.begin_frame(frame)
                batch = detector.detect(frame)
                objects = color_detector.detect(frame, batch.observations) if color_detector else []
                processed_at_s, age_s = processor.finish_frame(frame)
            except (FrameRejected, TagDetectionError) as exc:
                processor.abandon_frame()
                rejected_frames += 1
                rejection_reasons[exc.reason] += 1
                record.update(status="rejected_frame", reason=exc.reason)
            except Exception:
                processor.abandon_frame()
                raise
            else:
                processed += 1
                complete += int(batch.observation_complete)
                detected += int(bool(batch.observations))
                total += len(batch.observations)
                unknown += len(batch.unknown_tag_ids)
                duplicates += len(batch.duplicate_tag_ids)
                missing_frames += int(bool(batch.missing_robot_ids))
                record.update(status="detected", processed_at_s=processed_at_s,
                              host_age_ms=age_s * 1000.0, objects=objects, **batch.as_dict())
                last = detector.annotate(frame.image, batch)
                if color_detector:
                    color_detector.annotate(last, objects)
                if args.preview and not args.track:
                    if not preview_created:
                        cv2.namedWindow(preview_window, cv2.WINDOW_NORMAL)
                        scale = min(1.0, 1000 / last.shape[1], 650 / last.shape[0])
                        cv2.resizeWindow(preview_window, max(2, round(last.shape[1] * scale)),
                                         max(2, round(last.shape[0] * scale)))
                        preview_created = True
                    cv2.imshow(preview_window, last)
            if tracker:
                record.update(tracker.update(record, time.monotonic()))
                if args.preview:
                    if not preview_created:
                        cv2.namedWindow(preview_window, cv2.WINDOW_NORMAL)
                        preview_created = True
                    cv2.imshow(preview_window, draw_position_map(record, calibration.field_size_mm))
            if report is not None:
                report.write(json.dumps(record, allow_nan=False) + "\n")
                report.flush()
            if args.preview:
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")) or (preview_created and _window_closed(preview_window)):
                    status = "user_stopped"
                    break
        if tracker and report is not None:
            # Consumers ALSO need a watchdog if this process blocks or dies.
            terminal_at = time.monotonic()
            report.write(json.dumps({"status": "source_closed", "source_name": tracker.source or "unopened",
                "sequence": tracker.sequence + 1, "captured_at_s": terminal_at, "reason": status,
                "field_size_mm": list(calibration.field_size_mm), "is_replay": args.camera is None,
                **tracker.snapshot(terminal_at)}, allow_nan=False) + "\n")
            report.flush()
    if args.output is not None and last is not None:
        _write_image(args.output, last)
    summary = {
        "status": status,
        "processed_frames": processed,
        "detected_frames": detected,
        "complete_observation_frames": complete,
        "total_robot_detections": total,
        "unknown_tag_detections": unknown,
        "duplicate_tag_detections": duplicates,
        "missing_robot_frames": missing_frames,
        "rejected_frames": rejected_frames,
        "rejection_reasons": dict(rejection_reasons),
        "is_replay": args.image is not None or args.video is not None,
        "dictionary_name": tag_config.dictionary_name,
        "configured_robot_ids": sorted(tag_config.tag_to_robot.values()),
        "tag_size_mm": tag_config.tag_size_mm,
        "hardware_verified": tag_config.hardware_verified,
    }
    print(json.dumps(summary))
    success = processed > 0 and rejected_frames == 0 and status != "camera_read_failed"
    if args.require_complete_observation:
        success = success and complete == processed
    return 0 if success else 1


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    cv2 = None
    try:
        _distinct_paths(args)
        cv2, _ = vision_dependencies()
        return {"calibrate": _calibrate, "run": _run, "check": _check,
                "detect": _detect}[args.command](args)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"vision: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        if cv2 is None or not isinstance(exc, cv2.error):
            raise
        print(f"vision: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
