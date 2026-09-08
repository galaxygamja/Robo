"""Synthetic pixels through real OpenCV/video/process/CLI boundaries, no devices."""
from __future__ import annotations

import json
import subprocess
import sys
import unittest
from unittest.mock import patch

import test_runtime_process as video_tests
from test_mission_runtime import ROOT, fixture

from robo_control.binding_tools import create_draft, review_snapshot
from robo_control.runtime import main
from robo_control.runtime_report import inspect_report


@unittest.skipIf(video_tests.cv2 is None, "OpenCV vision extra is not installed")
class BindingVideoTests(unittest.TestCase):
    def setUp(self):
        self.video = video_tests.ActualVideoRuntimeTests()
        self.video.setUp()
        self.addCleanup(self.video.doCleanups)
        v, cv2 = self.video, video_tests.cv2
        reader = cv2.VideoCapture(str(v.video))
        try:
            success, frame = reader.read()
        finally:
            reader.release()
        self.assertTrue(success)
        cv2.circle(frame, (300, 240), 10, (0, 0, 255), -1)
        v.video = v.root / "observed-disc.avi"
        writer = cv2.VideoWriter(str(v.video), cv2.VideoWriter_fourcc(*"MJPG"), 20., (640, 480))
        self.assertTrue(writer.isOpened())
        try:
            for _ in range(24):
                writer.write(frame)
        finally:
            writer.release()
        fleet, _, plan = fixture()
        v.fleet.write_text(json.dumps(fleet), encoding="utf-8")
        plan["tasks"][0]["pickup"].update(x_mm=400., y_mm=840.)
        self.mission, self.colors = v.root/"mission.json", v.root/"colors.json"
        self.mission.write_text(json.dumps(plan), encoding="utf-8")
        self.colors.write_text(json.dumps({"schema_version": 1, "profiles": [
            {"color": "red", "kind": "disc", "hsv_ranges": [[[0, 100, 100], [10, 255, 255]],
              [[170, 100, 100], [179, 255, 255]]], "min_area_mm2": 100,
             "max_area_mm2": 3000, "min_circularity": .2}]}), encoding="utf-8")

    def arguments(self):
        return [*self.video.arguments(), "--mission", str(self.mission), "--colors", str(self.colors)]

    def run_cli(self, *extra):
        process = subprocess.run([sys.executable, "-m", "robo_control.runtime", *self.arguments(), *extra],
                                 cwd=ROOT, text=True, encoding="utf-8", capture_output=True, timeout=25, check=False)
        self.assertEqual(0, process.returncode, process.stdout+process.stderr)
        return json.loads(process.stdout)

    def test_observation_review_draft_new_session_binding_wire_pipeline(self):
        self.run_cli("--mission-observe-only")
        view = review_snapshot(self.video.report)
        self.assertEqual(1, len(view["objects"]))
        self.assertEqual("disc", view["objects"][0]["kind"])
        report = inspect_report(self.video.report)
        self.assertTrue(report["mission_observe_only"])
        self.assertIsNone(report["mission"]["active_task_id"])
        self.assertFalse(report["physical_stop_verified"])
        draft = create_draft(self.video.report, ["D1="+view["objects"][0]["object_id"]], half_size_mm=20.)
        self.assertFalse(draft["reviewed"])
        # Only a TEST marks synthetic fixture review; the tool never does so.
        draft.update(reviewed=True, reviewed_by="synthetic-test", evidence="generated pixels; no physical review")
        path = self.video.root/"bindings.json"
        path.write_text(json.dumps(draft), encoding="utf-8")
        original_report = self.video.report.read_bytes()
        self.video.report = self.video.root/"bound-runtime.jsonl"
        self.run_cli("--bindings", str(path), "--wire-fake")
        report = inspect_report(self.video.report)
        self.assertEqual("bound", report["bindings"]["status"])
        self.assertNotEqual(view["session_id"], report["session_id"])
        self.assertFalse(report["bindings"]["physical_identity_verified"])
        self.assertEqual("fake_wire", report["transport_mode"])
        self.assertTrue(report["final_wire_stop_acknowledged"])
        self.assertEqual([], report["mission"]["completed_tasks"])
        rows = [json.loads(line) for line in self.video.report.read_text(encoding="utf-8").splitlines()]
        moving = [r for r in rows[1:] if any(c["receiver"]["v_mm_s"] or c["receiver"]["omega_rad_s"]
                                           for c in r["wire"]["robots"])]
        self.assertTrue(moving)
        self.assertTrue(all(r["bindings"]["status"] == "bound" for r in moving))
        self.assertTrue(all(r["mission"]["world"]["ready"] for r in moving))
        self.assertEqual(original_report, (self.video.root/"runtime.jsonl").read_bytes())

    def test_binding_file_cannot_be_report_output(self):
        path = self.video.root/"bindings.json"
        path.write_text("preserve this input", encoding="utf-8")
        args = self.arguments()
        args[args.index("--report")+1] = str(path)
        with patch("sys.stderr"):
            self.assertEqual(2, main([*args, "--bindings", str(path)]))
        self.assertEqual("preserve this input", path.read_text(encoding="utf-8"))

    def test_schema_two_video_uses_measured_pickup_position_and_explicit_offset(self):
        self.run_cli("--mission-observe-only")
        view = review_snapshot(self.video.report)
        draft = create_draft(self.video.report, ["D1="+view["objects"][0]["object_id"]], half_size_mm=20.)
        draft.update(reviewed=True, reviewed_by="synthetic-test", evidence="synthetic video fixture only")
        path = self.video.root/"bindings.json"
        path.write_text(json.dumps(draft), encoding="utf-8")
        plan = json.loads(self.mission.read_text(encoding="utf-8"))
        plan["schema_version"] = 2
        plan["tasks"][0]["pickup"] = {"mode": "observed_piece", "heading_rad": 0.,
            "tool_forward_mm": 80., "tool_left_mm": 0., "max_anchor_drift_mm": 20.,
            "replan_distance_mm": 5., "max_replans": 3}
        self.mission.write_text(json.dumps(plan), encoding="utf-8")
        self.video.report = self.video.root/"observed-pickup-runtime.jsonl"
        self.run_cli("--bindings", str(path), "--wire-fake")
        rows = [json.loads(line) for line in self.video.report.read_text(encoding="utf-8").splitlines()]
        targets = [r["mission"]["observed_pickup"] for r in rows[1:] if r["mission"]["observed_pickup_in_use"]]
        self.assertTrue(targets)
        for target in targets:
            point = target["object_position_mm"]
            self.assertAlmostEqual(point[0]-80., target["robot_goal"]["x_mm"])
            self.assertAlmostEqual(point[1], target["robot_goal"]["y_mm"])
            self.assertEqual(0., target["robot_goal"]["heading_rad"])
            self.assertFalse(target["physical_pickup_verified"])
        audit = inspect_report(self.video.report)
        self.assertTrue(audit["final_wire_stop_acknowledged"])
        self.assertEqual([], audit["mission"]["completed_tasks"])

    def test_video_objects_feed_collision_map_without_hardware_or_scene_defaults(self):
        obstacle_path = self.video.root/"obstacles.json"
        obstacle_path.write_text(json.dumps({"schema_version": 1,
            "coordinate_system": "bottom_left_x_right_y_up_mm", "reviewed": True,
            "reviewed_by": "synthetic-test", "evidence": "generated pixel footprint only",
            "position_margin_mm": 2., "max_route_replans": 3,
            "footprints": [{"kind": "disc", "colour": "red", "radius_mm": 15.}]}), encoding="utf-8")
        self.run_cli("--object-obstacles", str(obstacle_path), "--wire-fake")
        audit = inspect_report(self.video.report)
        self.assertEqual(1, len(audit["object_obstacles"]["objects"]))
        self.assertFalse(audit["object_obstacles"]["physical_clearance_verified"])
        self.assertTrue(audit["final_wire_stop_acknowledged"])
        rows = [json.loads(line) for line in self.video.report.read_text(encoding="utf-8").splitlines()]
        moving = [r for r in rows[1:] if any(c["receiver"]["v_mm_s"] or c["receiver"]["omega_rad_s"]
                                           for c in r["wire"]["robots"])]
        self.assertTrue(moving)
        for row in moving:
            self.assertTrue(row["object_obstacles"]["ready"])
            self.assertEqual(15., row["object_obstacles"]["objects"][0]["radius_mm"])
            self.assertEqual(row["mission"]["world"]["source_sequence"], row["object_obstacles"]["source_sequence"])

    def test_incomplete_footprint_profiles_are_rejected_before_camera_launch(self):
        obstacle_path = self.video.root/"obstacles.json"
        obstacle_path.write_text(json.dumps({"schema_version": 1,
            "coordinate_system": "bottom_left_x_right_y_up_mm", "reviewed": True,
            "reviewed_by": "synthetic-test", "evidence": "bad class test",
            "position_margin_mm": 2., "max_route_replans": 3,
            "footprints": [{"kind": "cylinder", "colour": "red", "radius_mm": 15.}]}), encoding="utf-8")
        with patch("sys.stderr"):
            self.assertEqual(2, main([*self.arguments(), "--object-obstacles", str(obstacle_path)]))
        self.assertFalse(self.video.report.exists())

    def test_ambiguous_or_unbounded_mission_json_is_rejected_before_worker_creation(self):
        original = self.mission.read_text(encoding="utf-8")
        cases = [original[:-1]+', "cell_mm": 40}', '{"cell_mm": NaN}',
                 '{"cell_mm": 1e9999}', " "*65537, '{"a": '+"["*12+"0"+"]"*12+"}"]
        for contents in cases:
            self.mission.write_text(contents, encoding="utf-8")
            with self.subTest(contents=contents[:80]), patch("sys.stderr"):
                with patch("robo_control.runtime.multiprocessing.get_context") as create_context:
                    self.assertEqual(2, main(self.arguments()))
                create_context.assert_not_called()
            self.assertFalse(self.video.report.exists())


if __name__ == "__main__":
    unittest.main()
