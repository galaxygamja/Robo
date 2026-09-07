"""Collect RAW USB-camera chessboard photographs with explicit operator saves."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from ..adapters import OpenCVCameraSource
from .calibration import CalibrationError, _points, vision_dependencies
from .lens import _finite
from .lens_calibrate import board_points


class ChessboardCapture:
    """New folder only; no resized screenshots, duplicate poses or stale saves."""

    def __init__(self, directory, *, source_name, camera_label, inner_corners, max_age_s=1.):
        board_points(inner_corners, 1.)  # pattern validation; square size is measured at fitting
        if not isinstance(camera_label, str) or not camera_label.strip() or len(camera_label) > 200:
            raise CalibrationError("Use a nonempty camera/lens/focus label <=200 characters")
        if not isinstance(source_name, str) or not source_name.startswith("webcam:"):
            raise CalibrationError("Capture requires an explicitly selected USB camera source")
        if not _finite(max_age_s) or not 0 < max_age_s <= 2:
            raise CalibrationError("Offline photo age limit must be in (0,2] seconds; this is not a control-loop deadline")
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=False)
        self.handle = (self.root / "capture.jsonl").open("x", encoding="utf-8")
        self.source_name, self.camera_label = source_name, camera_label
        self.max_age_s = max_age_s
        self.inner_corners = tuple(inner_corners)
        self.image_size = None
        self.last_sequence = 0
        self.views = []
        self.closed = False
        self._write({"event": "capture_started", "camera_label": camera_label,
            "source_name": source_name, "board_inner_corners": list(inner_corners),
            "pixel_geometry": "raw_camera_pixels", "max_photo_age_s": max_age_s})

    def _write(self, row):
        row = {"schema_version": 1, "physical_accuracy_verified": False,
               "device_io": False, "motion_permitted": False, **row}
        self.handle.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        self.handle.flush()
        return row

    def save_frame(self, frame, corners, now_s):
        cv2, np = vision_dependencies()
        if self.closed:
            raise CalibrationError("Capture session is closed")
        if frame.source_name != self.source_name or frame.is_replay is not False:
            self.close("source_or_mode_changed")
            raise CalibrationError("Camera source/replay mode changed; start a new capture session")
        image = frame.image
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim not in (2, 3)
                or (image.ndim == 3 and image.shape[2] not in (1, 3, 4))):
            raise CalibrationError("Expected a raw uint8 camera image")
        size = (image.shape[1], image.shape[0])
        if (min(size) < 2 or max(size) >= 32767 or math.prod(size) > 16_000_000
                or (self.image_size is not None and size != self.image_size)):
            self.close("resolution_changed_or_unsupported")
            raise CalibrationError("Capture resolution changed or exceeds supported size; do not rescale images")
        self.image_size = size
        reason = None
        received = frame.received_at_s if frame.received_at_s is not None else frame.captured_at_s
        if (not all(_finite(value) for value in (frame.captured_at_s, received, now_s))
                or not 0 <= frame.captured_at_s <= received <= now_s):
            reason = "invalid_frame_time"
        elif now_s - frame.captured_at_s > self.max_age_s:
            reason = "stale_frame"
        if type(frame.sequence) is not int or frame.sequence <= self.last_sequence:
            reason = reason or "repeated_or_out_of_order_frame"
        else:
            self.last_sequence = frame.sequence
        points = None if corners is None else _points(corners)
        if points is None or points.shape != (math.prod(self.inner_corners), 2):
            reason = reason or "board_not_found"
        elif ((points < 0).any() or (points[:, 0] >= size[0]).any() or (points[:, 1] >= size[1]).any()):
            reason = reason or "board_outside_image"
        elif np.prod(np.ptp(points, axis=0)) < math.prod(size) * .015:
            reason = reason or "board_too_small"
        elif any(float(np.sqrt(np.mean(np.sum((points - old)**2, axis=1)))) < 2. for old in self.views):
            reason = reason or "duplicate_board_pose"
        if len(self.views) >= 100:
            reason = reason or "dataset_limit"
        if reason:
            return self._write({"event": "save_rejected", "reason": reason, "saved_count": len(self.views)})
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            raise CalibrationError("Raw PNG encoding failed")
        name = f"board-{len(self.views) + 1:03}.png"
        with (self.root / name).open("xb") as handle:
            handle.write(encoded.tobytes())
        self.views.append(points.copy())
        return self._write({"event": "image_saved", "file": name, "saved_count": len(self.views),
            "image_size_px": list(size), "frame_sequence": frame.sequence,
            "captured_at_s": frame.captured_at_s, "received_at_s": received, "selected_at_s": now_s,
            "timestamp_basis": frame.timestamp_basis, "corners_raw_px": points.tolist()})

    def close(self, reason="operator_finished"):
        if not self.closed:
            self.closed = True
            try:
                self._write({"event": "capture_closed", "reason": reason, "saved_count": len(self.views)})
            finally:
                self.handle.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, required=True)
    parser.add_argument("--camera-label", required=True)
    parser.add_argument("--inner-corners", type=int, nargs=2, required=True, metavar=("COLUMNS", "ROWS"))
    parser.add_argument("--output-dir", type=Path, required=True, help="NEW directory; raw photos remain on early exit")
    parser.add_argument("--count", type=int, default=24, help="requested saves, 12..100; press SPACE for each")
    parser.add_argument("--duration-s", type=float, default=600.)
    parser.add_argument("--max-photo-age-ms", type=float, default=1000., help="offline capture only; never changes runtime deadlines")
    args = parser.parse_args(argv)
    collector = source = None
    cv2 = None
    status = "capture_failed"
    window = "Raw chessboard capture | SPACE save | Q finish"
    try:
        if args.camera < 0 or not 12 <= args.count <= 100 or not _finite(args.duration_s) or not 0 < args.duration_s <= 1800:
            raise CalibrationError("Use a nonnegative camera, 12..100 saves and duration in (0,1800] seconds")
        cv2, _ = vision_dependencies()
        collector = ChessboardCapture(args.output_dir, source_name=f"webcam:{args.camera}",
            camera_label=args.camera_label, inner_corners=args.inner_corners, max_age_s=args.max_photo_age_ms / 1000.)
        source = OpenCVCameraSource(args.camera)
        started, message = time.monotonic(), "Move + tilt the board; SPACE saves the RAW image"
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        status = "duration_elapsed"
        while time.monotonic() - started < args.duration_s:
            frame = source.read()
            if frame is None:
                status = "camera_read_failed"
                break
            gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY) if frame.image.ndim == 3 else frame.image
            found, corners = cv2.findChessboardCornersSB(gray, tuple(args.inner_corners), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            shown = frame.image.copy()
            if found:
                cv2.drawChessboardCorners(shown, tuple(args.inner_corners), corners, True)
            scale = min(1., 960 / shown.shape[1], 680 / shown.shape[0])
            shown = cv2.resize(shown, (round(shown.shape[1] * scale), round(shown.shape[0] * scale)))
            cv2.putText(shown, f"{len(collector.views)}/{args.count} | {message}", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 255), 1)
            cv2.imshow(window, shown)
            key = cv2.waitKey(10) & 0xFF
            if key == 32:
                row = collector.save_frame(frame, corners.reshape(-1, 2) if found else None, time.monotonic())
                message = row.get("reason", "Saved. Move + tilt to a DIFFERENT position")
                if len(collector.views) >= args.count:
                    status = "requested_count_saved"
                    break
            if key in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                status = "operator_finished"
                break
    except KeyboardInterrupt:
        status = "operator_finished"
    except Exception as exc:  # noqa: BLE001 - camera/GUI backend failures still need cleanup and an actionable error
        print(f"lens-capture: {exc}", file=sys.stderr)
        return 2
    finally:
        try:
            if collector is not None:
                collector.close(status)
        finally:
            try:
                if source is not None:
                    source.close()
            finally:
                if cv2 is not None:
                    try:
                        cv2.destroyAllWindows()
                    except cv2.error:
                        pass  # A manually closed window/headless backend may have no GUI state.
    saved = len(collector.views) if collector else 0
    print(json.dumps({"status": status, "saved_images": saved, "output_dir": str(args.output_dir),
        "physical_accuracy_verified": False, "device_io": False, "motion_permitted": False}))
    return 0 if status in {"requested_count_saved", "operator_finished"} and saved >= 12 else 1


if __name__ == "__main__":
    raise SystemExit(main())
