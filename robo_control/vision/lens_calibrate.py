"""Fit a fixed camera lens from diverse chessboard photographs; no device output."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

from .calibration import CalibrationError, _points, vision_dependencies
from .lens import LensCalibration, _finite


def board_points(inner_corners, square_size_mm):
    _, np = vision_dependencies()
    if (not isinstance(inner_corners, (tuple, list)) or len(inner_corners) != 2
            or any(type(v) is not int or not 3 <= v <= 30 for v in inner_corners)
            or not _finite(square_size_mm) or not 1 <= square_size_mm <= 500):
        raise CalibrationError("Board needs 3..30 inner corners per axis and 1..500 mm measured square size")
    cols, rows = inner_corners
    result = np.zeros((cols * rows, 3), np.float32)
    result[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_mm
    return result


def fit_lens(image_points, image_size_px, inner_corners, square_size_mm, camera_label, max_error_px=1.0):
    """Use >=10 training views plus held-out views; do not silently remove bad data."""
    cv2, np = vision_dependencies()
    objects = board_points(inner_corners, square_size_mm)
    if (not isinstance(image_size_px, (tuple, list)) or len(image_size_px) != 2
            or any(type(v) is not int or v < 2 for v in image_size_px)
            or math.prod(image_size_px) > 16_000_000):
        raise CalibrationError("One exact image resolution <=16 MP is required")
    if not _finite(max_error_px) or not 0 < max_error_px <= 2:
        raise CalibrationError("Reprojection threshold must be in (0,2] px")
    if not isinstance(image_points, (tuple, list)) or not 12 <= len(image_points) <= 100:
        raise CalibrationError("Use 12..100 diverse views: at least 10 fit views and 2 held-out views")
    width, height = image_size_px
    views = []
    for index, values in enumerate(image_points):
        points = _points(values)
        if (points.shape != (len(objects), 2) or (points < 0).any()
                or (points[:, 0] >= width).any() or (points[:, 1] >= height).any()):
            raise CalibrationError(f"View {index} must contain every board corner inside the same image")
        if any(float(np.sqrt(np.mean(np.sum((points - other)**2, axis=1)))) < 2. for other in views):
            raise CalibrationError(f"View {index} repeats a nearly identical board pose; move and tilt the board")
        if np.prod(np.ptp(points, axis=0)) < width * height * .015:
            raise CalibrationError(f"View {index}: board is too small for this image")
        views.append(points.astype(np.float32))
    coverage = np.ptp(np.concatenate(views), axis=0) / np.array((width, height))
    if (coverage < .5).any():
        raise CalibrationError("Board views must span at least half of both image axes, not just the centre")
    # Every sixth view is held out, independent of image content/errors. Lens
    # parameters are never refit to the held-out images after this check.
    held_out = [i for i in range(len(views)) if i % 6 == 5]
    training = [i for i in range(len(views)) if i not in held_out]
    try:
        rms, matrix, coefficients, rotations, translations = cv2.calibrateCamera(
            [objects.copy() for _ in training], [views[i] for i in training], tuple(image_size_px),
            None, None, criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-9))
        errors, maximum_errors, normals = {}, {}, []
        for index, rotation, translation in zip(training, rotations, translations):
            projected, _ = cv2.projectPoints(objects, rotation, translation, matrix, coefficients)
            distances = np.linalg.norm(projected.reshape(-1, 2) - views[index], axis=1)
            errors[index] = float(np.sqrt(np.mean(distances**2)))
            maximum_errors[index] = float(distances.max())
            normals.append(cv2.Rodrigues(rotation)[0][:, 2])
        max_tilt = max(math.degrees(math.acos(float(np.clip(np.dot(a, b), -1, 1))))
                       for a in normals for b in normals)
        if max_tilt < 10:
            raise CalibrationError("Board orientations lack tilt diversity; parallel translated photos are insufficient")
        for index in held_out:
            found, rotation, translation = cv2.solvePnP(objects, views[index], matrix, coefficients,
                                                       flags=cv2.SOLVEPNP_ITERATIVE)
            if not found:
                raise CalibrationError(f"Held-out view {index} could not be evaluated")
            projected, _ = cv2.projectPoints(objects, rotation, translation, matrix, coefficients)
            distances = np.linalg.norm(projected.reshape(-1, 2) - views[index], axis=1)
            errors[index] = float(np.sqrt(np.mean(distances**2)))
            maximum_errors[index] = float(distances.max())
    except cv2.error as exc:
        raise CalibrationError(f"OpenCV could not fit/evaluate this board dataset: {exc}") from exc
    if (not math.isfinite(rms) or any(not math.isfinite(value) or value > max_error_px for value in errors.values())
            or any(not math.isfinite(value) or value > 3 * max_error_px for value in maximum_errors.values())):
        raise CalibrationError("Training/held-out reprojection threshold failed; inspect/refilm, do not discard errors silently")
    lens = LensCalibration(tuple(image_size_px), tuple(map(tuple, matrix.tolist())),
                           tuple(coefficients.ravel().tolist()), camera_label)
    report = {"schema_version": 1, "status": "lens_fit_completed", "opencv_version": cv2.__version__,
        "lens_calibration_id": lens.fingerprint, "training_rms_px": float(rms), "max_error_px": max_error_px,
        "training_view_indices": training, "held_out_view_indices": held_out,
        "per_view_rms_px": [errors[i] for i in range(len(views))],
        "per_view_max_corner_error_px": [maximum_errors[i] for i in range(len(views))],
        "max_corner_error_limit_px": 3 * max_error_px,
        "image_axis_coverage_fraction": coverage.tolist(), "max_fitted_normal_angle_deg": max_tilt,
        "board_inner_corners": list(inner_corners), "square_size_mm": square_size_mm,
        "image_size_px": list(image_size_px), "camera_label": camera_label,
        "physical_accuracy_verified": False, "device_io": False, "motion_permitted": False}
    return lens, report


def calibrate_images(paths, inner_corners, square_size_mm, camera_label, max_error_px=1.0):
    cv2, np = vision_dependencies()
    board_points(inner_corners, square_size_mm)
    if not 12 <= len(paths) <= 100:
        raise CalibrationError("Supply 12..100 photographs from one fixed camera/lens configuration")
    size, views, records, digests = None, [], [], set()
    for path in paths:
        path = Path(path)
        if path.stat().st_size > 50_000_000:
            raise CalibrationError(f"Image exceeds 50 MB input limit: {path}")
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        if digest in digests:
            raise CalibrationError("Duplicate image content does not provide another calibration view")
        digests.add(digest)
        gray = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None or gray.size > 16_000_000:
            raise CalibrationError(f"Image is undecodable or exceeds 16 MP: {path}")
        actual_size = (gray.shape[1], gray.shape[0])
        if size is not None and actual_size != size:
            raise CalibrationError("Mixed image sizes: changing resolution/crop invalidates the camera model")
        size = actual_size
        found, corners = cv2.findChessboardCornersSB(gray, tuple(inner_corners), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        record = {"file": path.name, "sha256": digest, "board_found": bool(found)}
        if found:
            record["view_index"] = len(views)
            record["corners_raw_px"] = corners.reshape(-1, 2).tolist()
            views.append(record["corners_raw_px"])
        records.append(record)
    lens, report = fit_lens(views, size, inner_corners, square_size_mm, camera_label, max_error_px)
    report["images"] = records
    report["undetected_images"] = sum(not record["board_found"] for record in records)
    return lens, report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, nargs="+", required=True, help="files or one folder (non-recursive)")
    parser.add_argument("--inner-corners", type=int, nargs=2, required=True, metavar=("COLUMNS", "ROWS"))
    parser.add_argument("--square-size-mm", type=float, required=True, help="measured printed square edge, not corner spacing guesses")
    parser.add_argument("--camera-label", required=True, help="camera/lens serial or fixed identifier plus focus/zoom settings")
    parser.add_argument("--max-view-rms-px", "--max-error-px", dest="max_error_px", type=float, default=1.0,
                        help="maximum RMS per view (<=2 px); each individual corner must also be <=3x this")
    parser.add_argument("--output", type=Path, required=True, help="NEW lens profile JSON")
    parser.add_argument("--report", type=Path, required=True, help="NEW fit/held-out diagnostics JSON")
    args = parser.parse_args(argv)
    try:
        paths = args.images
        if len(paths) == 1 and paths[0].is_dir():
            paths = sorted(p for p in paths[0].iterdir() if p.is_file() and p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"})
        resolved = [p.resolve(strict=True) for p in paths]
        if len(set(resolved)) != len(resolved):
            raise CalibrationError("Duplicate image paths")
        if (args.output.resolve() == args.report.resolve() or args.output.exists() or args.report.exists()
                or {args.output.resolve(), args.report.resolve()}.intersection(resolved)):
            raise CalibrationError("Output/report must be distinct NEW files and must not overwrite input images")
        lens, report = calibrate_images(resolved, args.inner_corners, args.square_size_mm,
                                       args.camera_label, args.max_error_px)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        with args.report.open("x", encoding="utf-8") as handle:
            handle.write(json.dumps(report, indent=2, allow_nan=False) + "\n")
        lens.save(args.output)
        print(json.dumps({"status": report["status"], "output": str(args.output),
            "report": str(args.report), "training_rms_px": report["training_rms_px"],
            "lens_calibration_id": lens.fingerprint, "physical_accuracy_verified": False}, allow_nan=False))
        return 0
    except (OSError, ValueError, RuntimeError, TypeError) as exc:
        print(f"lens-calibrate: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
