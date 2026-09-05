"""HSV + metric contour gates. Colour is a class, NOT persistent object identity."""
from __future__ import annotations

import json
import math
from pathlib import Path

from .calibration import FieldCalibration, vision_dependencies


class ColorDetector:
    def __init__(self, calibration: FieldCalibration, config: dict):
        self.calibration = calibration
        self.profiles = config.get("profiles", [])
        if config.get("schema_version") != 1 or not self.profiles:
            raise ValueError("Colour config requires schema_version=1 and profiles")
        names = set()
        for p in self.profiles:
            if p["color"] in names or p["kind"] not in {"cylinder", "disc", "cube"}:
                raise ValueError("Unique colour names and supported kinds required")
            names.add(p["color"])
            if not p["hsv_ranges"]:
                raise ValueError("At least one HSV interval required")
            for lo, hi in p["hsv_ranges"]:
                if len(lo) != 3 or len(hi) != 3 or any(type(a) is not int or type(b) is not int
                        or not 0 <= a <= b <= limit for a, b, limit in zip(lo, hi, (179, 255, 255))):
                    raise ValueError("HSV intervals must use H 0..179, S/V 0..255")
            a, b, c = p["min_area_mm2"], p["max_area_mm2"], p["min_circularity"]
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (a,b,c)) or not 0 < a < b or not 0 <= c <= 1:
                raise ValueError("Invalid contour limits")

    @classmethod
    def load(cls, calibration, path):
        return cls(calibration, json.loads(Path(path).read_text(encoding="utf-8-sig")))

    def detect(self, frame, tags=()):
        cv2, np = vision_dependencies()
        rectified = self.calibration.warp(frame.image, 1.0)
        if rectified.ndim != 3 or rectified.shape[2] not in (3, 4):
            raise ValueError("Colour detection requires a BGR/BGRA frame")
        bgr = rectified[:,:,:3]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        exclude = np.zeros(hsv.shape[:2], np.uint8)
        for obs in tags:
            corners = self.calibration.field_mm_to_rectified_px(obs.corners_mm)
            cv2.fillConvexPoly(exclude, np.rint(corners).astype(np.int32), 255)
        exclude = cv2.dilate(exclude, np.ones((9,9), np.uint8))
        objects = []
        for profile in self.profiles:
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in profile["hsv_ranges"]:
                mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask[exclude > 0] = 0
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area, perimeter = cv2.contourArea(contour), cv2.arcLength(contour, True)
                if not profile["min_area_mm2"] <= area <= profile["max_area_mm2"] or perimeter <= 0:
                    continue
                circularity = 4 * math.pi * area / perimeter**2
                if circularity < profile["min_circularity"]:
                    continue
                moment = cv2.moments(contour)
                x, y = moment["m10"]/moment["m00"], self.calibration.field_size_mm[1] - moment["m01"]/moment["m00"]
                width, height = self.calibration.field_size_mm
                if not 0 <= x <= width or not 0 <= y <= height:
                    continue
                objects.append({"color": profile["color"], "kind": profile["kind"],
                                "center_mm": [x,y], "area_mm2": area, "circularity": circularity,
                                "identity": None, "classification": "color_geometry_candidate"})
        return sorted(objects, key=lambda o: (o["color"], o["center_mm"]))

    def annotate(self, image, objects):
        cv2, _ = vision_dependencies()
        for obs in objects:
            x,y = self.calibration.field_mm_to_pixel([obs["center_mm"]])[0]
            cv2.circle(image, (round(x),round(y)), 12, (255,0,255), 2)
            cv2.putText(image, f'{obs["color"]} {obs["center_mm"][0]:.0f},{obs["center_mm"][1]:.0f}mm',
                        (round(x)+14,round(y)), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,0,255),1)
        return image
