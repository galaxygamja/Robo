from __future__ import annotations

import copy
import unittest
from dataclasses import replace

from test_mission_runtime import MissionHarness, fixture
from test_world_state import (
    SESSION,
    SOURCE,
    TrackedFrames,
    make_adapter,
    object_candidate,
)

from robo_control.observed_obstacles import ObservedObstacleMap
from robo_control.runtime_session import LiveControlSession


def obstacle_plan(**changes):
    result = {"schema_version": 1, "coordinate_system": "bottom_left_x_right_y_up_mm",
        "reviewed": True, "reviewed_by": "synthetic-test", "evidence": "test fixtures; no physical sizes verified",
        "position_margin_mm": 2., "max_route_replans": 3,
        "footprints": [{"kind": "cylinder", "colour": "red", "radius_mm": 15.},
                       {"kind": "disc", "colour": "white", "radius_mm": 10.}]}
    result.update(changes)
    return result


def make_map(plan=None):
    return ObservedObstacleMap(obstacle_plan() if plan is None else plan,
        field_size_mm=(1143., 1181.), session_id=SESSION)


class ObjectObstacleMapTests(unittest.TestCase):
    def setUp(self):
        self.adapter, self.frames, self.obstacles = make_adapter(), TrackedFrames(), make_map()
        self.seq = 0

    def feed(self, *, objects=None, change=None):
        self.seq += 1
        now = 10.+self.seq*.02
        row = self.frames.frame(self.seq, now, objects=objects)
        world = self.adapter.update(row, now, source_session_id=SESSION)
        if change:
            world = change(world)
        return self.obstacles.update(world, row["objects"], now)

    def warm(self, **kwargs):
        for _ in range(3):
            result = self.feed(**kwargs)
        return result

    def test_requires_world_readiness_and_explicit_footprint_sizes(self):
        for seq in range(1, 4):
            value = self.feed()
            self.assertEqual(seq == 3, value["ready"])
        self.assertEqual(15., value["objects"][0]["radius_mm"])
        self.assertEqual({"x_mm": 383., "y_mm": 283., "width_mm": 34., "height_mm": 34.},
                         self.obstacles.rectangles[0])
        for key in ("unobserved_is_free", "motion_permitted", "physical_clearance_verified", "field_coverage_verified"):
            self.assertFalse(value[key])

    def test_new_tentative_candidate_is_an_immediate_hazard_not_a_pickup_identity(self):
        self.warm()
        value = self.feed(objects=[object_candidate(), object_candidate(700, 900, colour="white", kind="disc")])
        self.assertTrue(value["ready"])
        self.assertEqual(2, len(value["objects"]))
        self.assertEqual("tentative", value["objects"][1]["confirmation_state"])
        world = self.adapter.poll(10.+self.seq*.02)
        self.assertFalse(world.objects[1].valid_for_pick)

    def test_moving_hazard_updates_geometry_without_rewriting_history_clock(self):
        first = self.warm()
        value = self.feed(objects=[object_candidate(410, 305)])
        self.assertEqual(first["revision"]+1, value["revision"])
        self.assertEqual((410., 305.), value["objects"][0]["position_mm"])
        self.assertEqual(4, value["objects"][0]["source_sequence"])
        stable = self.feed(objects=[object_candidate(410, 305)])
        self.assertEqual(value["revision"], stable["revision"])

    def test_lost_object_latches_and_keeps_last_footprint_not_empty_space(self):
        before = self.warm()
        value = self.feed(objects=[])
        self.assertEqual("object_obstacle_lost", value["fault"])
        self.assertEqual(before["objects"], value["objects"])
        for _ in range(3):
            value = self.feed()
        self.assertFalse(value["ready"])
        self.assertEqual("object_obstacle_lost", value["fault"])

    def test_ambiguous_raw_candidates_without_stable_ids_latch(self):
        self.warm()
        value = self.feed(objects=[object_candidate(399, 300), object_candidate(401, 300)])
        self.assertFalse(value["ready"])
        self.assertEqual("object_obstacle_unconfirmed_or_ambiguous", value["fault"])
        self.assertEqual(1, len(value["objects"]))

    def test_unknown_class_is_not_assigned_an_invented_size(self):
        value = self.warm(objects=[object_candidate(colour="green")])
        self.assertEqual("object_obstacle_unknown_footprint", value["fault"])
        self.assertEqual([], value["objects"])

    def test_expiry_retains_geometry_without_claiming_freshness(self):
        value = self.warm()
        before = copy.deepcopy(value["objects"])
        value = self.obstacles.poll(10.26)
        self.assertFalse(value["ready"])
        self.assertEqual("object_obstacle_observation_expired", value["reason"])
        self.assertEqual(before, value["objects"])
        self.assertEqual(3, value["source_sequence"])

    def test_world_rejection_does_not_renew_or_clear_geometry(self):
        before = self.warm()
        value = self.obstacles.poll(10.07, world_ready=False)
        self.assertFalse(value["ready"])
        self.assertEqual(before["objects"], value["objects"])
        self.assertEqual("object_obstacle_world_not_ready", value["reason"])

    def test_source_change_or_backwards_clock_latches(self):
        self.warm()
        value = self.feed(change=lambda w: replace(w, session_id="other"))
        self.assertEqual("object_obstacle_source_changed", value["fault"])
        self.setUp()
        self.warm()
        with self.assertRaises(ValueError):
            self.obstacles.poll(1.)
        self.assertEqual("object_obstacle_clock_invalid", self.obstacles.fault)

    def test_geometry_and_snapshot_mutation_cannot_change_map(self):
        value = self.warm()
        value["objects"][0]["radius_mm"] = 1000
        with self.assertRaises(TypeError):
            self.obstacles.rectangles[0]["x_mm"] = 0
        self.assertEqual(15., self.obstacles.snapshot()["objects"][0]["radius_mm"])

    def test_replans_are_bounded_and_fault_stays_latched(self):
        self.warm()
        for expected in (True, True, True, False, False):
            self.assertEqual(expected, self.obstacles.request_replan())
        self.assertEqual(3, self.obstacles.route_replans)
        self.assertEqual("object_obstacle_replan_budget_exceeded", self.obstacles.fault)

    def test_review_schema_and_missing_sizes_rejected_before_runtime(self):
        for patch in ({"reviewed": False}, {"reviewed": 1}, {"reviewed_by": " "}, {"evidence": ""},
                      {"coordinate_system": "m"}, {"schema_version": True}, {"position_margin_mm": -1},
                      {"max_route_replans": True}, {"max_route_replans": 101}, {"footprints": []}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                make_map(obstacle_plan(**patch))
        for patch in ({"radius_mm": None}, {"radius_mm": True}, {"radius_mm": 0},
                      {"radius_mm": float("inf")}, {"kind": []}, {"colour": ""}, {"radius_mm": 1e30}):
            plan = obstacle_plan()
            plan["footprints"][0].update(patch)
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                make_map(plan)

    def test_empty_initial_frame_is_not_a_claim_of_verified_whole_field_coverage(self):
        value = self.warm(objects=[])
        self.assertTrue(value["ready"])
        self.assertEqual([], value["objects"])
        self.assertFalse(value["field_coverage_verified"])

    def test_64_object_boundary_is_supported_but_65_latches_without_partial_map(self):
        objects = [object_candidate(80.+140*x, 80.+140*y) for x in range(8) for y in range(8)]
        value = self.warm(objects=objects)
        self.assertTrue(value["ready"])
        self.assertEqual(64, len(value["objects"]))
        before = copy.deepcopy(value["objects"])
        value = self.feed(objects=[*objects, object_candidate(1100., 1100.)])
        self.assertEqual("object_obstacle_capacity_or_payload", value["fault"])
        self.assertEqual(before, value["objects"])
        self.assertFalse(value["ready"])

    def test_oversized_first_ready_batch_does_not_partially_activate(self):
        objects = [object_candidate(80.+140*x, 80.+140*y) for x in range(8) for y in range(8)]
        value = self.warm(objects=[*objects, object_candidate(1100., 1100.)])
        self.assertEqual("object_obstacle_capacity_or_payload", value["fault"])
        self.assertFalse(value["activated"])
        self.assertEqual([], value["objects"])

    def test_profile_input_mutation_after_construction_does_not_change_sizes_or_limits(self):
        plan = obstacle_plan()
        self.obstacles = make_map(plan)
        plan["footprints"][0]["radius_mm"] = 900
        plan["position_margin_mm"] = 900
        plan["max_route_replans"] = 0
        value = self.warm()
        self.assertEqual(15., value["objects"][0]["radius_mm"])
        self.assertEqual(2., value["position_margin_mm"])
        self.assertTrue(self.obstacles.request_replan())

    def test_stale_recovery_keeps_track_identity_and_requires_new_ready_frames(self):
        before = self.warm()
        self.assertFalse(self.obstacles.poll(10.26)["ready"])
        # Advance the host clock rather than pretending old frames are new.
        self.seq = 13
        for index in range(3):
            value = self.feed()
            self.assertEqual(index == 2, value["ready"])
        self.assertIsNone(value["fault"])
        self.assertEqual(before["objects"][0]["track_id"], value["objects"][0]["track_id"])
        self.assertGreater(value["objects"][0]["observed_at_s"], before["objects"][0]["observed_at_s"])

    def test_source_configuration_and_replay_changes_latch_without_geometry_replacement(self):
        for patch in ({"source_name": "foreign"}, {"configuration_id": "foreign"}, {"is_replay": True}):
            self.setUp()
            before = self.warm()
            value = self.feed(change=lambda w, patch=patch: replace(w, **patch))
            with self.subTest(patch=patch):
                self.assertEqual("object_obstacle_source_changed", value["fault"])
                self.assertEqual(before["objects"], value["objects"])

    def test_duplicate_fresh_world_sequence_is_not_a_new_geometry_lease(self):
        before = self.warm()
        world = self.adapter.poll(10.07)
        value = self.obstacles.update(world, [object_candidate()], 10.07)
        self.assertEqual("object_obstacle_frame_not_fresh", value["fault"])
        self.assertEqual(before["captured_at_s"], value["captured_at_s"])
        self.assertEqual(before["objects"], value["objects"])


class ObstacleMissionHarness(MissionHarness):
    def __init__(self, *, wire_fake=False, plan=None):
        super().__init__()
        fleet, roles, mission = fixture()
        mission["tasks"][0]["pickup"].update(x_mm=600., y_mm=500.)
        self.session = LiveControlSession(roles=roles, field_size_mm=(1143., 1181.), source_name=SOURCE,
            session_id="mission-test", mission_plan=mission, fleet=fleet, track_objects=True,
            obstacle_plan=obstacle_plan() if plan is None else plan, wire_fake=wire_fake)

    def step(self, *, objects=None, **kwargs):
        other = kwargs.pop("change", None)

        def change(row):
            row["objects"] = [object_candidate(450., 700.)] if objects is None else objects
            if other:
                other(row)
        return super().step(change=change, **kwargs)


class ObjectObstacleRuntimeTests(unittest.TestCase):
    def test_visible_hazard_participates_in_a_star_and_command_guards(self):
        h = ObstacleMissionHarness()
        for _ in range(6):
            event = h.step(objects=[object_candidate(450., 500.)])
        self.assertIsNone(event["closed_reason"])
        route = event["mission"]["route"]
        self.assertTrue(any(abs(point["y_mm"]-500) > 40 for point in route))
        self.assertTrue(event["object_obstacles"]["ready"])
        self.assertFalse(event["motion_permitted"])

    def test_new_obstacle_on_remaining_route_causes_bounded_replan(self):
        h = ObstacleMissionHarness()
        for _ in range(6):
            h.step()
        event = h.step(objects=[object_candidate(450., 700.), object_candidate(450., 500., kind="disc", colour="white")])
        self.assertIsNone(event["closed_reason"])
        self.assertEqual(1, event["object_obstacles"]["route_replans"])
        self.assertTrue(any(abs(point["y_mm"]-500) > 30 for point in event["mission"]["route"]))

    def test_new_hazard_in_braking_envelope_cancels_due_wire_motion_first(self):
        h = ObstacleMissionHarness(wire_fake=True)
        for _ in range(6):
            h.step()
        h.session.wire.configure_link("H1", h.now, request_delay_s=.04)
        queued = h.step()
        before = next(r["receiver"]["accepted_drive_count"] for r in queued["wire"]["robots"] if r["robot_id"] == "H1")
        h.step()
        event = h.step(objects=[object_candidate(450., 700.), object_candidate(330., 500., kind="disc", colour="white")])
        self.assertEqual("mission_obstacle_envelope", event["closed_reason"])
        after = next(r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
        self.assertEqual(before, after)
        self.assertTrue(all(r["stop_requested"] for r in event["wire"]["robots"]))

    def test_ack_wait_zero_bank_does_not_hide_a_delayed_drive_hazard(self):
        h = ObstacleMissionHarness(wire_fake=True)
        for _ in range(10):
            h.step()
        h.session.wire.configure_link("H1", h.now, request_delay_s=.05)
        queued = h.step()
        before = next(r["receiver"]["accepted_drive_count"] for r in queued["wire"]["robots"] if r["robot_id"] == "H1")
        for _ in range(2):
            waiting = h.step()
        self.assertTrue(all(c["forward_velocity_mm_s"] == 0 for c in waiting["actuator"]["robots"]))
        # Outside the zero-speed body envelope, inside the queued 64 mm/s
        # stopping envelope. The measured synthetic tags remain stationary.
        event = h.step(objects=[object_candidate(450., 700.),
                               object_candidate(360., 500., kind="disc", colour="white")])
        self.assertEqual("mission_obstacle_envelope", event["closed_reason"])
        after = next(r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
        self.assertEqual(before, after)
        self.assertTrue(all(r["stop_requested"] for r in event["wire"]["robots"]))

    def test_first_drive_is_cancelled_while_receiver_and_bank_are_both_zero(self):
        for delay, polls in ((.025, 1), (.045, 2), (.065, 3)):
            with self.subTest(delay=delay):
                h = ObstacleMissionHarness(wire_fake=True)
                for _ in range(3):
                    h.step()
                h.session.wire.configure_link("H1", h.now, request_delay_s=delay)
                queued = h.step()
                candidate = next(c for c in queued["actuator"]["robots"] if c["robot_id"] == "H1")
                self.assertGreater(candidate["forward_velocity_mm_s"], 0)
                for _ in range(polls):
                    waiting = h.step()
                receiver = next(r["receiver"] for r in waiting["wire"]["robots"] if r["robot_id"] == "H1")
                self.assertEqual(0, receiver["accepted_drive_count"])
                self.assertEqual(0, receiver["v_mm_s"])
                self.assertTrue(all(c["forward_velocity_mm_s"] == 0 for c in waiting["actuator"]["robots"]))
                event = h.step(objects=[object_candidate(450., 700.),
                    object_candidate(350., 500., kind="disc", colour="white")])
                self.assertEqual("mission_obstacle_envelope", event["closed_reason"])
                receiver = next(r["receiver"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
                self.assertEqual(0, receiver["accepted_drive_count"])
                self.assertEqual(0, receiver["v_mm_s"])

    def test_delayed_ack_existing_receiver_motion_is_guarded_before_new_authorization(self):
        h = ObstacleMissionHarness(wire_fake=True)
        for _ in range(10):
            h.step()
        h.session.wire.configure_link("H1", h.now, response_delay_s=.05)
        queued = h.step()
        before = next(r["receiver"]["accepted_drive_count"] for r in queued["wire"]["robots"] if r["robot_id"] == "H1")
        for _ in range(2):
            h.step()
        event = h.step(objects=[object_candidate(450., 700.),
            object_candidate(360., 500., kind="disc", colour="white")])
        self.assertEqual("mission_obstacle_envelope", event["closed_reason"])
        receiver = next(r["receiver"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
        self.assertEqual(before, receiver["accepted_drive_count"])
        self.assertEqual(0, receiver["v_mm_s"])

    def test_lost_hazard_closes_without_clearing_obstacle_records(self):
        h = ObstacleMissionHarness(wire_fake=True)
        for _ in range(6):
            before = h.step()
        event = h.step(objects=[])
        self.assertEqual("object_obstacle_lost", event["closed_reason"])
        self.assertEqual(before["object_obstacles"]["objects"], event["object_obstacles"]["objects"])
        self.assertFalse(event["object_obstacles"]["ready"])

    def test_replan_budget_does_not_enable_old_route_fallback(self):
        h = ObstacleMissionHarness(plan=obstacle_plan(max_route_replans=0))
        for _ in range(6):
            h.step()
        event = h.step(objects=[object_candidate(450., 700.), object_candidate(450., 500., kind="disc", colour="white")])
        self.assertEqual("object_obstacle_replan_budget_exceeded", event["closed_reason"])
        self.assertTrue(all(r["forward_velocity_mm_s"] == 0 for r in event["actuator"]["robots"]))

    def test_output_intent_cannot_remove_or_ignore_pickup_target_obstacle(self):
        h = ObstacleMissionHarness(wire_fake=True)
        for _ in range(4):
            event = h.step(objects=[object_candidate(600., 500., kind="disc", colour="white")])
        self.assertEqual("route_unavailable", event["closed_reason"])
        self.assertEqual([], event["object_obstacles"]["contact_exemptions"])
        self.assertEqual(1, len(event["object_obstacles"]["objects"]))


if __name__ == "__main__":
    unittest.main()
