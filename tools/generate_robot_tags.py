"""Generate the exact OpenCV AprilTag images expected by robo_control.vision."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from robo_control.vision.calibration import vision_dependencies
from robo_control.vision.tags import (
    MARKER_FORWARD,
    TagDetectorConfig,
    default_tag_config_path,
)


def positive_integer(value: str) -> int:
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def generate(config: TagDetectorConfig, output_dir: Path, side_px: int,
             quiet_zone_px: int, overwrite: bool = False) -> dict:
    cv2, np = vision_dependencies()
    if side_px < 64:
        raise ValueError("side-px must be >= 64")
    minimum_quiet_zone = max(16, math.ceil(side_px / 8))
    if quiet_zone_px < minimum_quiet_zone:
        raise ValueError(
            f"quiet-zone-px must be >= {minimum_quiet_zone} for side-px {side_px}"
        )
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, config.dictionary_name)
    )
    assets = []
    for tag_id, robot_id in config.tag_to_robot.items():
        stem = f"{robot_id}_tag_{tag_id}"
        assets.append((tag_id, robot_id, output_dir / f"{stem}_printable.png",
                       output_dir / f"{stem}_front_reference.png"))
    manifest_path = output_dir / "manifest.json"
    targets = [manifest_path, *(path for item in assets for path in item[2:])]
    existing = [str(path) for path in targets if path.exists()]
    if existing and not overwrite:
        raise FileExistsError(f"refusing to overwrite generated asset: {existing[0]}")
    output_dir.mkdir(parents=True, exist_ok=True)
    files = []
    for tag_id, robot_id, printable_path, reference_path in assets:
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, side_px)
        printable = np.full(
            (side_px + 2 * quiet_zone_px, side_px + 2 * quiet_zone_px), 255, dtype=np.uint8
        )
        printable[
            quiet_zone_px:quiet_zone_px + side_px,
            quiet_zone_px:quiet_zone_px + side_px,
        ] = marker
        ok, encoded = cv2.imencode(".png", printable)
        if not ok:
            raise RuntimeError(f"PNG encoder failed: {printable_path}")
        printable_path.write_bytes(encoded.tobytes())

        arrow_band = max(60, quiet_zone_px)
        label_band = 55
        canvas = np.full(
            (arrow_band + side_px + 2 * quiet_zone_px + label_band,
             side_px + 2 * quiet_zone_px, 3),
            255,
            dtype=np.uint8,
        )
        x, y = quiet_zone_px, arrow_band + quiet_zone_px
        canvas[y:y + side_px, x:x + side_px] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        arrow_y = max(18, arrow_band // 2)
        cv2.arrowedLine(canvas, (x, arrow_y), (x + side_px, arrow_y),
                        (0, 0, 220), max(2, side_px // 200), tipLength=0.06)
        cv2.putText(canvas, "ROBOT FRONT / MARKER +X", (x, max(16, arrow_y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, max(0.45, side_px / 1500), (0, 0, 180), 2)
        cv2.putText(canvas, f"{robot_id}   TAG {tag_id}", (x, y + side_px + quiet_zone_px + 35),
                    cv2.FONT_HERSHEY_SIMPLEX, max(0.55, side_px / 1200), (0, 0, 0), 2)
        ok, encoded = cv2.imencode(".png", canvas)
        if not ok:
            raise RuntimeError(f"PNG encoder failed: {reference_path}")
        reference_path.write_bytes(encoded.tobytes())
        files.append({
            "robot_id": robot_id,
            "tag_id": tag_id,
            "printable_marker_png": printable_path.name,
            "front_reference_png": reference_path.name,
        })
    manifest = {
        "schema_version": 1,
        "dictionary_name": config.dictionary_name,
        "marker_forward": MARKER_FORWARD,
        "tag_size_mm": config.tag_size_mm,
        "marker_square_side_px": side_px,
        "quiet_zone_px": quiet_zone_px,
        "files": files,
        "printing": (
            "Print each printable_marker_png with its white quiet zone intact. "
            "Scale the black marker square (not the full image) to tag_size_mm and disable page scaling. "
            "If tag_size_mm is null, measure and update the configuration before physical use."
        ),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate compatible robot AprilTag PNG files")
    parser.add_argument("--config", type=Path, default=default_tag_config_path())
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--side-px", type=positive_integer, default=800)
    parser.add_argument("--quiet-zone-px", type=positive_integer, default=100)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(float(args.side_px)) or not math.isfinite(float(args.quiet_zone_px)):
            raise ValueError("pixel sizes must be finite")
        result = generate(TagDetectorConfig.load(args.config), args.output_dir,
                          args.side_px, args.quiet_zone_px, args.overwrite)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"generate-tags: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": "generated", "output_dir": str(args.output_dir),
                      "tag_count": len(result["files"]), "tag_size_mm": result["tag_size_mm"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
