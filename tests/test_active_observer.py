from __future__ import annotations

import copy
import io
import json
import math
import unittest
from contextlib import redirect_stdout
from dataclasses import replace

from robo_control.active_observer import (ActiveObserverPlanner, CameraModel, DronePose, FlightBounds,
    MultiSourcePoseSelector, ObservationTarget, Occluder, calibration_gate, main, observe_geometry,
    ray_box_intersects, run_demo)


def frame(source="fallback", seq=1, stamp=10., ids=("H1", "H2", "B1", "B2"), *, error=4., confidence=.9):
    return {"source_id": source, "session_id": source + "-one", "sequence": seq,
            "status": "detected", "captured_at_s": stamp, "received_at_s": stamp,
            "clock_domain": "observer-host", "image_size_px": [1280, 720],
            "calibration": {"valid": True, "frame_sequence": seq, "captured_at_s": stamp,
                "image_size_px": [1280, 720], "field_frame_id": "arena-mm", "reference_count": 4,
                "reference_geometry_valid": True, "target_height_model_valid": True,
                "dynamic_reference_update": source == "drone", "reprojection_error_mm": 1., "pose_error_bound_mm": 3.},
            "robots": [{"robot_id": rid, "robot_center_mm": [100. + i * 40., 200.], "heading_rad": 0.,
                        "confidence": confidence, "error_bound_mm": error} for i, rid in enumerate(ids)]}


def selector(ids=("H1", "H2", "B1", "B2")):
    return MultiSourcePoseSelector(ids, source_sessions={"fallback": "fallback-one", "drone": "drone-one"}, moving_sources=("drone",))


class VisibilityTests(unittest.TestCase):
    def test_finite_height_occlusion_clears_with_higher_viewpoint(self):
        target = ObservationTarget("H1", "robot", (800., 400., 30.))
        box = Occluder("bin", (480., 300., 0.), (550., 500., 400.))
        camera = CameraModel(horizontal_fov_rad=math.radians(110))
        low = observe_geometry(DronePose(200., 400., 600.), target, [box], camera)
        high = observe_geometry(DronePose(200., 400., 1100.), target, [box], camera)
        self.assertEqual(low["reason"], "occluded")
        self.assertTrue(high["visible"])

    def test_downward_fov_and_marker_pixel_limits(self):
        pose = DronePose(500., 500., 800.)
        self.assertTrue(observe_geometry(pose, ObservationTarget("one", "robot", (500., 500., 30.)))["visible"])
        self.assertFalse(observe_geometry(pose, ObservationTarget("one", "robot", (500., 500., 900.)))["visible"])
        outside = ObservationTarget("one", "robot", (1100., 500., 30.))
        self.assertEqual(observe_geometry(pose, outside)["reason"], "outside_fov_or_uncertainty_margin")
        tiny = ObservationTarget("one", "object", (500., 500., 20.), marker_size_mm=5.)
        self.assertEqual(observe_geometry(pose, tiny)["reason"], "insufficient_pixel_footprint")

    def test_yaw_rotates_rectangular_fov_and_uncertainty_reduces_visibility(self):
        camera = CameraModel(horizontal_fov_rad=math.radians(90.), vertical_fov_rad=math.radians(30.))
        target = ObservationTarget("one", "robot", (500., 950., 0.))
        self.assertFalse(observe_geometry(DronePose(500., 500., 800.), target, camera=camera)["visible"])
        self.assertTrue(observe_geometry(DronePose(500., 500., 800., math.pi / 2), target, camera=camera)["visible"])
        uncertain = replace(target, uncertainty_mm=500.)
        self.assertFalse(observe_geometry(DronePose(500., 500., 800., math.pi / 2), uncertain, camera=camera)["visible"])

    def test_ray_slab_parallel_tangent_endpoint_and_padding(self):
        box = Occluder("bin", (0., 0., 0.), (100., 100., 100.))
        self.assertTrue(ray_box_intersects((-100., 50., 50.), (200., 50., 50.), box))
        self.assertFalse(ray_box_intersects((-100., 101., 50.), (200., 101., 50.), box))
        self.assertTrue(ray_box_intersects((-100., 101., 50.), (200., 101., 50.), box, padding_mm=2.))
        self.assertFalse(ray_box_intersects((50., 50., 300.), (50., 50., 100.), box))
        self.assertTrue(ray_box_intersects((50., 50., 300.), (50., 50., 100.), box, include_endpoints=True))

    def test_partial_tag_occlusion_is_not_treated_as_clear_center(self):
        target = ObservationTarget("one", "robot", (500., 500., 30.), marker_size_mm=100., uncertainty_mm=0.)
        box = Occluder("edge", (545., 545., 0.), (565., 565., 200.))
        pose = DronePose(550., 550., 800.)
        self.assertFalse(ray_box_intersects(pose.point, target.position_mm, box))
        self.assertEqual(observe_geometry(pose, target, [box])["reason"], "occluded")

    def test_invalid_geometry_and_camera_models_fail_early(self):
        for point in ((math.nan, 1., 1.), (1., True, 1.)):
            with self.assertRaises(ValueError):
                DronePose(*point)
        with self.assertRaises(ValueError):
            CameraModel(horizontal_fov_rad=math.pi)
        with self.assertRaises(ValueError):
            Occluder("bad", (0., 0., 0.), (1., 1., 0.))
        with self.assertRaises(ValueError):
            ObservationTarget("bad", "robot", (0., 0., 0.), missing="true")


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.left, self.right = DronePose(300., 400., 700.), DronePose(800., 400., 700.)
        self.camera = CameraModel(horizontal_fov_rad=math.radians(40.), vertical_fov_rad=math.radians(40.))
        self.targets = [ObservationTarget("left", "robot", (250., 400., 30.), fallback_visible=True),
                        ObservationTarget("right", "robot", (850., 400., 30.), missing=True, task_state="drop_verification")]

    def planner(self, **kwargs):
        return ActiveObserverPlanner((self.left, self.right), camera=self.camera, **kwargs)

    def test_mission_missing_and_verification_priorities_choose_needed_viewpoint(self):
        p = self.planner()
        result = p.plan(self.left, self.targets, now_s=1.)
        self.assertEqual(result["selected_pose"], {"x_mm": 800., "y_mm": 400., "z_mm": 700., "yaw_rad": 0.})
        self.assertEqual(result["expected_visible_ids"], ["right"])
        self.assertFalse(result["physical_commands"])
        self.assertFalse(result["dynamic_calibration_implemented"])
        self.assertTrue(result["calibration_required_per_frame"])
        next_pose = DronePose(**result["next_pose"])
        self.assertLessEqual(math.dist(next_pose.point[:2], self.left.point[:2]), 30. + 1e-8)

    def test_hysteresis_and_minimum_dwell_avoid_viewpoint_chatter(self):
        stay = self.planner(switch_margin=100.)
        self.assertEqual(stay.plan(self.left, self.targets, now_s=1.)["reason"], "hysteresis")
        p = self.planner()
        first = p.plan(self.left, self.targets, now_s=1.)
        reversed_needs = [replace(self.targets[0], missing=True, fallback_visible=False, weight=10.),
                          replace(self.targets[1], missing=False, fallback_visible=True, task_state="idle")]
        immediate = p.plan(self.left, reversed_needs, now_s=1.1)
        self.assertEqual(immediate["reason"], "minimum_dwell")
        self.assertEqual(immediate["selected_pose"], first["selected_pose"])
        later = p.plan(self.left, reversed_needs, now_s=1.5)
        self.assertEqual(later["selected_pose"]["x_mm"], 300.)

    def test_bounds_speed_and_elapsed_gap_do_not_teleport_camera(self):
        p = self.planner()
        first = p.plan(self.left, self.targets, now_s=1.)
        current = DronePose(**first["next_pose"])
        second = p.plan(current, self.targets, now_s=100.)
        self.assertLessEqual(math.dist(current.point[:2], DronePose(**second["next_pose"]).point[:2]), 150. + 1e-8)
        invalid = p.plan(DronePose(-10., 100., 800.), self.targets, now_s=101.)
        self.assertEqual(invalid["status"], "hold")
        with self.assertRaises(ValueError):
            self.planner().plan(self.left, self.targets * 2, now_s=1.)
        with self.assertRaises(ValueError):
            ActiveObserverPlanner([DronePose(0., 0., 100.)])

    def test_padded_flight_corridor_not_just_camera_sightline_is_checked(self):
        p = self.planner()
        wall = Occluder("high-divider", (500., 0., 0.), (600., 1000., 1000.))
        result = p.plan(self.left, self.targets, [wall], now_s=1.)
        self.assertEqual(result["next_pose"]["x_mm"], self.left.x_mm)
        self.assertTrue(all(e["pose"]["x_mm"] != self.right.x_mm for e in result["candidates"]))

    def test_no_targets_yields_no_motion(self):
        result = self.planner().plan(self.left, [], now_s=1.)
        self.assertEqual(result["status"], "hold")

    def test_removing_targets_during_dwell_holds_and_discards_incumbent(self):
        p = self.planner()
        first = p.plan(self.left, self.targets, now_s=1.)
        self.assertEqual(first["status"], "planned_move")
        current = DronePose(**first["next_pose"])
        removed = p.plan(current, [], now_s=1.1)
        self.assertEqual(removed["status"], "hold")
        self.assertEqual(removed["reason"], "no_targets")
        self.assertEqual(removed["next_pose"], first["next_pose"])
        self.assertIsNone(p.selected)
        reversed_need = [replace(self.targets[0], missing=True, fallback_visible=False, weight=10.)]
        resumed = p.plan(current, reversed_need, now_s=1.2)
        self.assertNotEqual(resumed["reason"], "minimum_dwell")
        self.assertNotEqual(resumed["selected_pose"], first["selected_pose"])


class CalibrationTests(unittest.TestCase):
    def test_per_frame_dynamic_reference_and_quality_gate(self):
        good = frame("drone")
        self.assertIsNone(calibration_gate(good, moving=True))
        changes = [{"frame_sequence": 0}, {"captured_at_s": 9.}, {"captured_at_s": True},
                   {"valid": False}, {"reference_count": 3}, {"reference_count": True},
                   {"dynamic_reference_update": False}, {"target_height_model_valid": False},
                   {"reference_geometry_valid": False}, {"reprojection_error_mm": 20.},
                   {"pose_error_bound_mm": math.nan}, {"field_frame_id": "another-field"}]
        for update in changes:
            with self.subTest(update=update):
                bad = copy.deepcopy(good)
                bad["calibration"].update(update)
                self.assertIsNotNone(calibration_gate(bad, moving=True))
        bad = copy.deepcopy(good)
        bad["image_size_px"] = [640, 480]
        self.assertEqual(calibration_gate(bad), "calibration_resolution_mismatch")


class SelectionTests(unittest.TestCase):
    def test_moving_source_generator_keeps_dynamic_calibration_requirement(self):
        s = MultiSourcePoseSelector(["H1"], source_sessions={"drone": "drone-one"},
                                    moving_sources=(source for source in ("drone",)))
        self.assertEqual(s.moving_sources, frozenset({"drone"}))
        bad = frame("drone", ids=("H1",))
        bad["calibration"]["dynamic_reference_update"] = False
        rejected = s.ingest(bad, 10.)
        self.assertEqual(rejected["ingest_rejection"], "moving_camera_requires_dynamic_calibration")
        self.assertTrue(rejected["stop_required"])
        good = frame("drone", 2, 10.1, ids=("H1",))
        self.assertFalse(s.ingest(good, 10.1)["stop_required"])

    def test_one_source_per_id_with_complementary_partial_frames(self):
        s = selector()
        first = s.ingest(frame(ids=("H2", "B1", "B2")), 10.)
        self.assertTrue(first["stop_required"])
        second = s.ingest(frame("drone", ids=("H1",)), 10.)
        self.assertFalse(second["stop_required"])
        self.assertEqual(len(second["selected_poses"]), 4)
        self.assertEqual(second["selected_sources"]["H1"], "drone")
        self.assertFalse(second["hardware_ready"])
        self.assertFalse(second["motion_permitted"])

    def test_quality_threshold_then_freshness_then_error_and_confidence(self):
        s = selector(("H1",))
        s.ingest(frame(ids=("H1",), confidence=.95, error=4.), 10.)
        same = s.ingest(frame("drone", ids=("H1",), confidence=.99, error=5.), 10.)
        self.assertEqual(same["selected_sources"]["H1"], "fallback")
        newer = s.ingest(frame("drone", 2, 10.1, ids=("H1",), error=5., confidence=.65), 10.1)
        self.assertEqual(newer["selected_sources"]["H1"], "drone")
        self.assertEqual(len(newer["selected_poses"]), 1)

    def test_simultaneous_source_contradiction_is_not_decided_by_confidence(self):
        s = selector()
        s.ingest(frame(confidence=.7), 10.)
        contradictory = frame("drone", confidence=.99)
        contradictory["robots"][0]["robot_center_mm"] = [1100., 200.]
        result = s.ingest(contradictory, 10.)
        self.assertEqual(result["missing_robot_ids"], ["H1"])
        self.assertNotIn("H1", result["selected_sources"])
        self.assertEqual(len(result["selected_poses"]), 3)
        conflict = result["observation_conflicts"][0]
        self.assertEqual(conflict["robot_id"], "H1")
        self.assertEqual(conflict["pairs"][0]["distance_mm"], 1000.)
        self.assertEqual(conflict["pairs"][0]["allowed_distance_mm"], 8.)
        self.assertTrue(result["stop_required"])
        self.assertFalse(s.snapshot(10.01)["observation_usable"])
        # Agreement in a subsequent fresh observation restores selection;
        # conflicting coordinates were never blended into a fake midpoint.
        recovered = s.ingest(frame("drone", 2, 10.1), 10.1)
        self.assertEqual(recovered["observation_conflicts"], [])
        self.assertFalse(recovered["stop_required"])

    def test_compatible_asynchronous_observations_allow_bounded_displacement(self):
        def configured():
            return MultiSourcePoseSelector(["H1"], source_sessions={"fallback": "fallback-one", "drone": "drone-one"},
                moving_sources=("drone",), max_displacement_speed_mm_s=300.)
        s = configured()
        s.ingest(frame(ids=("H1",)), 10.)
        moving = frame("drone", stamp=10.1, ids=("H1",))
        moving["robots"][0]["robot_center_mm"] = [130., 200.]
        compatible = s.ingest(moving, 10.1)
        self.assertFalse(compatible["stop_required"])
        self.assertEqual(compatible["observation_conflicts"], [])
        self.assertEqual(compatible["selected_poses"][0]["robot_center_mm"], [130., 200.])
        too_far = configured()
        too_far.ingest(frame(ids=("H1",)), 10.)
        moving["robots"][0]["robot_center_mm"] = [139., 200.]
        inconsistent = too_far.ingest(moving, 10.1)
        self.assertTrue(inconsistent["stop_required"])
        self.assertAlmostEqual(inconsistent["observation_conflicts"][0]["pairs"][0]["allowed_distance_mm"], 38.)

    def test_expired_source_is_not_reused_as_consistency_evidence(self):
        s = selector(("H1",))
        s.ingest(frame(ids=("H1",)), 10.)
        later = frame("drone", stamp=10.1, ids=("H1",))
        later["robots"][0]["robot_center_mm"] = [1100., 200.]
        self.assertTrue(s.ingest(later, 10.1)["stop_required"])
        result = s.snapshot(10.26)
        self.assertEqual(result["observation_conflicts"], [])
        self.assertEqual(result["selected_sources"], {"H1": "drone"})
        self.assertTrue(s.snapshot(10.36)["stop_required"])

    def test_no_fallback_to_older_measurement_after_newer_source_fails(self):
        s = selector(("H1",))
        s.ingest(frame(ids=("H1",)), 10.)
        s.ingest(frame("drone", 1, 10.1, ids=("H1",)), 10.1)
        rejected = frame("drone", 2, 10.15, ids=("H1",))
        rejected["calibration"]["valid"] = False
        result = s.ingest(rejected, 10.15)
        self.assertTrue(result["stop_required"])
        self.assertEqual(result["selected_poses"], [])
        recovery = s.ingest(frame(seq=2, stamp=10.2, ids=("H1",)), 10.2)
        self.assertFalse(recovery["stop_required"])

    def test_stale_source_cannot_revive_or_overwrite_newer_pose(self):
        s = selector(("H1",))
        s.ingest(frame(seq=2, stamp=10.1, ids=("H1",)), 10.1)
        old = frame(seq=1, stamp=10., ids=("H1",))
        old["robots"][0]["robot_center_mm"] = [900., 900.]
        result = s.ingest(old, 10.12)
        self.assertEqual(result["ingest_rejection"], "out_of_order_source_frame")
        self.assertEqual(result["selected_poses"][0]["robot_center_mm"], [100., 200.])
        self.assertTrue(s.snapshot(10.35)["stop_required"])
        stale = s.ingest(frame("drone", stamp=10., ids=("H1",)), 10.36)
        self.assertTrue(stale["stop_required"])

    def test_source_close_latches_and_restart_requires_new_session(self):
        s = selector(("H1",))
        s.ingest(frame(ids=("H1",)), 10.)
        closed = s.ingest({"source_id": "fallback", "session_id": "fallback-one", "status": "source_closed"}, 10.1)
        self.assertTrue(closed["stop_required"])
        self.assertEqual(s.ingest(frame(seq=2, stamp=10.2, ids=("H1",)), 10.2)["ingest_rejection"], "source_session_closed")
        s.restart_source("fallback", "fallback-two", now_s=10.21)
        good = frame(stamp=10.22, ids=("H1",))
        self.assertEqual(s.ingest(good, 10.22)["ingest_rejection"], "wrong_source_session")
        good["session_id"] = "fallback-two"
        self.assertFalse(s.ingest(good, 10.22)["stop_required"])
        with self.assertRaises(ValueError):
            s.restart_source("fallback", "fallback-one", now_s=10.23)

    def test_wrong_clock_source_frame_resolution_and_quality_never_contribute(self):
        mutations = [lambda f: f.update(clock_domain="another-host"), lambda f: f.update(source_id="unknown"),
                     lambda f: f.update(session_id="other"), lambda f: f.update(image_size_px=[640, 480]),
                     lambda f: f["robots"][0].update(confidence=.1),
                     lambda f: f["robots"][0].update(error_bound_mm=1.),
                     lambda f: f["robots"][0].update(error_bound_mm=50.),
                     lambda f: f["robots"][0].update(robot_center_mm=[math.inf, 0.]),
                     lambda f: f["robots"][0].update(robot_center_mm=[-1., 0.]),
                     lambda f: f["robots"][0].update(heading_rad=True),
                     lambda f: f.update(robots=None)]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                bad = frame(ids=("H1",))
                mutate(bad)
                self.assertTrue(selector(("H1",)).ingest(bad, 10.)["stop_required"])

    def test_unknown_duplicate_or_invalid_robot_invalidates_that_whole_source_frame(self):
        for mutate in (lambda f: f["robots"][0].update(robot_id="unknown"),
                       lambda f: f["robots"][0].update(robot_id="H2"),
                       lambda f: f["robots"][0].update(confidence=True)):
            s = selector()
            s.ingest(frame(), 10.)
            bad = frame(seq=2, stamp=10.1)
            mutate(bad)
            result = s.ingest(bad, 10.1)
            self.assertTrue(result["stop_required"])
            self.assertEqual(result["selected_poses"], [])

    def test_rejected_frame_consumes_header_order_and_stops_old_source_cache(self):
        s = selector(("H1",))
        s.ingest(frame(ids=("H1",)), 10.)
        rejected = frame(seq=3, stamp=10.2, ids=("H1",))
        rejected["status"] = "rejected_frame"
        self.assertTrue(s.ingest(rejected, 10.2)["stop_required"])
        old = s.ingest(frame(seq=2, stamp=10.1, ids=("H1",)), 10.21)
        self.assertEqual(old["ingest_rejection"], "out_of_order_source_frame")
        self.assertTrue(old["stop_required"])

    def test_configurable_5_6_12_fleet_and_no_double_count(self):
        for n in (5, 6, 12):
            ids = tuple(f"robot{i}" for i in range(n))
            s = selector(ids)
            s.ingest(frame(ids=ids), 10.)
            result = s.ingest(frame("drone", ids=ids), 10.)
            self.assertEqual(len(result["selected_poses"]), n)
            self.assertEqual(len(result["selected_sources"]), n)
            self.assertFalse(result["stop_required"])

    def test_returned_coordinates_do_not_alias_cached_pose(self):
        s = selector(("H1",))
        result = s.ingest(frame(ids=("H1",)), 10.)
        result["selected_poses"][0]["robot_center_mm"][0] = 900.
        self.assertEqual(s.snapshot(10.01)["selected_poses"][0]["robot_center_mm"][0], 100.)

    def test_invalid_selector_configuration_and_clock(self):
        with self.assertRaises(ValueError):
            selector(("H1", "H1"))
        with self.assertRaises(ValueError):
            MultiSourcePoseSelector(["H1"], source_sessions={})
        with self.assertRaises(ValueError):
            MultiSourcePoseSelector(["H1"], source_sessions={"a": "s"}, moving_sources=("b",))
        s = selector()
        s.snapshot(10.)
        with self.assertRaises(ValueError):
            s.snapshot(9.)
        for speed in (-1., True, math.inf, 5001.):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                MultiSourcePoseSelector(["H1"], source_sessions={"a": "s"}, max_displacement_speed_mm_s=speed)


class DemoTests(unittest.TestCase):
    def test_identical_dropout_and_occlusion_recover_only_with_active_viewpoint(self):
        result = run_demo()
        parked, active = (result["results"][name] for name in ("parked", "active"))
        self.assertEqual(parked["complete_observation_frames"], 0)
        self.assertIsNone(parked["first_complete_after_dropout_s"])
        self.assertGreater(active["complete_observation_frames"], 0)
        self.assertIsNotNone(active["first_complete_after_dropout_s"])
        self.assertTrue(parked["after_all_sources_stop"])
        self.assertTrue(active["after_all_sources_stop"])
        self.assertFalse(result["dynamic_calibration_implemented"])
        self.assertFalse(result["device_io"])
        self.assertNotIn("score", result)
        self.assertEqual(len(result["synthetic_reference_points_mm"]), 4)
        self.assertTrue(all(row["visible_reference_count"] >= 4 for row in active["trace"] if not row["stop_required"]))

    def test_demo_cli_is_deterministic_json(self):
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertEqual(main(["--demo", "--compact"]), 0)
        self.assertEqual(run_demo(), run_demo())
        self.assertEqual(json.loads(output.getvalue()), json.loads(json.dumps(run_demo())))


if __name__ == "__main__":
    unittest.main()
