from __future__ import annotations

import copy
import math
import unittest
from dataclasses import replace

from test_mission_bindings import PROFILE, BoundMissionHarness, binding_plan
from test_mission_runtime import fixture
from test_world_state import (
    SESSION,
    SOURCE,
    TrackedFrames,
    make_adapter,
    object_candidate,
)

from robo_control.mission_runtime import MissionExecutor
from robo_control.observed_pickup import ObservedPickupTarget, PickupPolicy
from robo_control.runtime_session import LiveControlSession


def policy_data(**changes):
    value = {"mode": "observed_piece", "heading_rad": 0., "tool_forward_mm": 80., "tool_left_mm": 0.,
             "max_anchor_drift_mm": 30., "replan_distance_mm": 5., "max_replans": 3}
    value.update(changes)
    return value


def make_policy(**changes):
    return PickupPolicy.parse(policy_data(**changes), field_size_mm=(1143., 1181.))


class ObservedPickupTargetTests(unittest.TestCase):
    def test_next_task_boundary_does_not_mark_previous_target_as_in_use(self):
        fleet, roles, plan = fixture(task_ids=("move-R1", "move-R2"))
        plan["schema_version"] = 2
        for entry in plan["tasks"]:
            entry["pickup"] = policy_data()
        mission = MissionExecutor(plan, fleet, roles=roles, field_size_mm=(1143., 1181.))
        world = self.warm()
        self.target.observe(world)
        mission.world = world.as_dict()
        mission.observed_pickup = self.target
        self.assertTrue(mission.snapshot()["observed_pickup_in_use"])
        # Completion advances index before a new camera frame constructs the
        # next task's target. Preserve the old target as HISTORY only.
        mission.index = 1
        self.assertFalse(mission.snapshot()["observed_pickup_in_use"])
        self.assertEqual("move-R1", mission.snapshot()["observed_pickup"]["task_id"])

    def setUp(self):
        self.adapter, self.frames = make_adapter(), TrackedFrames()
        self.target = ObservedPickupTarget(make_policy(), task_id="move-R1", piece_id="R1")
        self.sequence = 0

    def feed(self, point=(400, 300)):
        self.sequence += 1
        now = 10.+self.sequence*.02
        return self.adapter.update(self.frames.frame(self.sequence, now, objects=[object_candidate(*point)]),
                                   now, source_session_id=SESSION)

    def warm(self):
        for _ in range(3):
            world = self.feed()
        return self.adapter.bind_piece("R1", world.objects[0].object_id, world.generated_at_s, evidence="synthetic review")

    def test_rotates_body_tool_offset_and_never_uses_piece_yaw(self):
        world = self.warm()
        for heading, expected in ((0, (320, 280)), (math.pi/2, (420, 220)),
                                  (math.pi, (480, 320)), (-math.pi/2, (380, 380))):
            target = ObservedPickupTarget(make_policy(heading_rad=heading, tool_left_mm=20), task_id="t", piece_id="R1")
            goal = target.observe(world)
            self.assertAlmostEqual(expected[0], goal["x_mm"])
            self.assertAlmostEqual(expected[1], goal["y_mm"])
            self.assertIsNone(world.piece("R1").yaw_rad)
            self.assertFalse(target.snapshot()["physical_pickup_verified"])

    def test_fresh_positions_update_goal_without_resetting_anchor(self):
        self.target.observe(self.warm())
        goal = self.target.observe(self.feed((410, 310)))
        self.assertEqual({"x_mm": 330., "y_mm": 310., "heading_rad": 0.}, goal)
        self.assertEqual((400., 300.), self.target.snapshot()["anchor_position_mm"])
        self.assertAlmostEqual(math.sqrt(200), self.target.snapshot()["anchor_drift_mm"])

    def test_exact_drift_limit_allowed_but_more_latches(self):
        self.target.observe(self.warm())
        self.assertIsNotNone(self.target.observe(self.feed((430, 300))))
        self.assertIsNone(self.target.observe(self.feed((430.1, 300))))
        self.assertEqual("pickup_anchor_drift_exceeded", self.target.fault)
        self.assertIsNone(self.target.goal)
        self.assertIsNone(self.target.observe(self.feed((400, 300))))

    def test_unbound_piece_cannot_use_catalog_position_as_fallback(self):
        for _ in range(3):
            world = self.feed()
        self.assertIsNone(self.target.observe(world))
        self.assertEqual("pickup_observation_unavailable", self.target.fault)
        self.assertIsNone(self.target.goal)

    def test_changed_track_session_or_reordered_observation_latches(self):
        for change, expected in (("track", "pickup_track_changed"), ("session", "pickup_source_session_changed"),
                                 ("sequence", "pickup_observation_reordered")):
            self.setUp()
            world = self.warm()
            self.target.observe(world)
            changed = self.feed()
            if change == "track":
                changed = replace(changed, pieces=tuple(replace(p, track_id="other") if p.piece_id == "R1" else p
                                                      for p in changed.pieces))
            elif change == "session":
                changed = replace(changed, session_id="other")
            else:
                changed = replace(changed, source_sequence=2)
            with self.subTest(change=change):
                self.assertIsNone(self.target.observe(changed))
                self.assertEqual(expected, self.target.fault)

    def test_repeated_snapshot_is_not_new_observation_or_replan(self):
        world = self.warm()
        goal = self.target.observe(world)
        self.target.planned()
        for _ in range(10):
            self.assertEqual(goal, self.target.observe(world))
        self.assertFalse(self.target.replan_required())
        self.assertEqual(0, self.target.replans)

    def test_replan_threshold_uses_last_planned_goal_and_budget_is_bounded(self):
        self.target.observe(self.warm())
        self.target.planned()
        self.target.observe(self.feed((405, 300)))
        self.assertFalse(self.target.replan_required())
        for i, x in enumerate((406, 400, 406, 400)):
            self.target.observe(self.feed((x, 300)))
            self.assertTrue(self.target.replan_required())
            self.assertEqual(i < 3, self.target.request_replan())
            if i < 3:
                self.assertFalse(self.target.replan_required())
                self.target.planned()
        self.assertEqual("pickup_replan_budget_exceeded", self.target.fault)

    def test_observation_recovery_does_not_reset_drift_anchor(self):
        self.target.observe(self.warm())
        world = self.adapter.invalidate(10.065, reason="test-camera-gap")
        self.assertIsNone(self.target.observe(world))
        self.assertIsNone(self.target.fault)
        for _ in range(3):
            world = self.feed((425, 300))
        self.assertIsNotNone(self.target.observe(world))
        self.assertEqual((400., 300.), self.target.snapshot()["anchor_position_mm"])

    def test_policy_requires_every_explicit_finite_parameter(self):
        for patch in ({"heading_rad": None}, {"tool_forward_mm": None}, {"tool_left_mm": True},
                      {"max_anchor_drift_mm": 0}, {"replan_distance_mm": 31}, {"max_replans": 1.5},
                      {"max_replans": True}, {"max_replans": -1}, {"max_replans": 101},
                      {"tool_forward_mm": float("inf")}, {"tool_left_mm": 1e20},
                      {"replan_distance_mm": float("nan")}, {"mode": "scenario"}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                make_policy(**patch)
        data = policy_data()
        data.pop("tool_left_mm")
        with self.assertRaises(ValueError):
            PickupPolicy.parse(data, field_size_mm=(1143., 1181.))

    def test_output_mutation_does_not_change_goal_or_anchor(self):
        self.target.observe(self.warm())
        result = self.target.snapshot()
        result["robot_goal"]["x_mm"] = 1000
        self.assertEqual(320, self.target.goal["x_mm"])


class ObservedMissionHarness(BoundMissionHarness):
    def __init__(self, *, policy=None, wire_fake=False, obstacle=None, robot_x=300.):
        super().__init__()
        fleet, roles, plan = fixture()
        plan["schema_version"] = 2
        plan["tasks"][0]["pickup"] = policy_data() if policy is None else policy
        if obstacle:
            plan["obstacles_mm"] = [obstacle]
        self.positions["H1"][0] = robot_x
        self.session = LiveControlSession(roles=roles, field_size_mm=(1143., 1181.), source_name=SOURCE,
            session_id="mission-test", track_objects=True, mission_plan=plan, fleet=fleet,
            binding_plan=binding_plan("D1", "disc", "white"), observation_profile_id=PROFILE, wire_fake=wire_fake)


class ObservedPickupRuntimeTests(unittest.TestCase):
    def test_schema_one_stays_static_and_schema_two_cannot_silently_fallback(self):
        fleet, roles, plan = fixture()
        plan["tasks"][0]["pickup"] = policy_data()
        with self.assertRaises(ValueError):
            MissionExecutor(plan, fleet, roles=roles, field_size_mm=(1143., 1181.))
        plan["schema_version"] = 2
        with self.assertRaisesRegex(ValueError, "bindings"):
            LiveControlSession(roles=roles, source_name=SOURCE, field_size_mm=(1143., 1181.),
                               mission_plan=plan, fleet=fleet)
        observed = LiveControlSession(roles=roles, source_name=SOURCE, field_size_mm=(1143., 1181.),
                                     mission_plan=plan, fleet=fleet, mission_observe_only=True)
        self.assertTrue(observed.mission_observe_only)

    def test_measured_object_drives_goal_not_scenario_or_old_static_pickup(self):
        h = ObservedMissionHarness()
        for _ in range(6):
            event = h.step()
        self.assertIsNone(event["closed_reason"])
        observed = event["mission"]["observed_pickup"]
        self.assertEqual({"x_mm": 320., "y_mm": 300., "heading_rad": 0.}, observed["robot_goal"])
        self.assertEqual(observed["robot_goal"], event["mission"]["route"][-1])
        self.assertEqual([], event["mission"]["completed_tasks"])

    def test_small_target_change_updates_endpoint_even_without_a_star_replan(self):
        h = ObservedMissionHarness()
        for _ in range(6):
            h.step()
        event = h.step(objects=[object_candidate(403, 300, kind="disc", colour="white")])
        self.assertEqual(323., event["mission"]["route"][-1]["x_mm"])
        self.assertEqual(0, event["mission"]["observed_pickup"]["replans"])

    def test_large_target_change_replans_from_current_measured_robot_position(self):
        h = ObservedMissionHarness()
        for _ in range(6):
            h.step()
        event = h.step(objects=[object_candidate(410, 300, kind="disc", colour="white")])
        self.assertIsNone(event["closed_reason"])
        self.assertEqual(330., event["mission"]["route"][-1]["x_mm"])
        self.assertEqual(1, event["mission"]["observed_pickup"]["replans"])
        h.step(no_record=True)
        self.assertEqual(1, h.session.mission.observed_pickup.replans)

    def test_target_drift_closes_before_due_wire_drive_is_delivered(self):
        h = ObservedMissionHarness(wire_fake=True, policy=policy_data(max_anchor_drift_mm=10., replan_distance_mm=5.))
        for _ in range(6):
            h.step()
        h.session.wire.configure_link("H1", h.now, request_delay_s=.04)
        queued = h.step()
        before = next(r["receiver"]["accepted_drive_count"] for r in queued["wire"]["robots"] if r["robot_id"] == "H1")
        h.step()
        event = h.step(objects=[object_candidate(420, 300, kind="disc", colour="white")])
        self.assertEqual("pickup_anchor_drift_exceeded", event["closed_reason"])
        after = next(r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
        self.assertEqual(before, after)

    def test_robot_goal_outside_field_is_rejected_before_start(self):
        h = ObservedMissionHarness(policy=policy_data(tool_forward_mm=390), wire_fake=True)
        for _ in range(3):
            event = h.step()
        self.assertEqual("observed_pickup_outside_free_space", event["closed_reason"])
        self.assertFalse(event["wire"]["started"])

    def test_robot_goal_inside_static_obstacle_is_rejected(self):
        h = ObservedMissionHarness(obstacle={"x_mm": 310, "y_mm": 290, "width_mm": 20, "height_mm": 20})
        for _ in range(3):
            event = h.step()
        self.assertEqual("observed_pickup_outside_free_space", event["closed_reason"])

    def test_robot_goal_occupied_by_other_robot_is_rejected_before_wire_start(self):
        h = ObservedMissionHarness(wire_fake=True)
        h.positions["B1"] = [320., 300., 0.]
        for _ in range(3):
            event = h.step()
        self.assertEqual("observed_pickup_outside_free_space", event["closed_reason"])
        self.assertFalse(event["wire"]["started"])

    def test_replan_budget_error_is_a_stop_not_old_goal_reuse(self):
        h = ObservedMissionHarness(policy=policy_data(max_replans=0))
        for _ in range(6):
            h.step()
        event = h.step(objects=[object_candidate(410, 300, kind="disc", colour="white")])
        self.assertEqual("pickup_replan_budget_exceeded", event["closed_reason"])
        self.assertTrue(all(r["forward_velocity_mm_s"] == 0 and r["angular_velocity_rad_s"] == 0
                            for r in event["actuator"]["robots"]))

    def test_cube_never_gets_observed_pickup_without_orientation_or_load_evidence(self):
        fleet, roles, plan = fixture()
        cube_task = next(t["id"] for t in fleet["task_plan"] if t["piece_id"].startswith("C"))
        plan["schema_version"] = 2
        plan["tasks"][0].update(task_id=cube_task, pickup=policy_data())
        with self.assertRaises(ValueError):
            MissionExecutor(plan, fleet, roles=roles, field_size_mm=(1143., 1181.))

    def test_schema_two_without_observed_target_and_wrong_shapes_are_rejected(self):
        fleet, roles, plan = fixture()
        plan["schema_version"] = 2
        with self.assertRaises(ValueError):
            MissionExecutor(plan, fleet, roles=roles, field_size_mm=(1143., 1181.))
        plan["tasks"][0]["pickup"] = policy_data()
        for key in tuple(policy_data()):
            changed = copy.deepcopy(plan)
            changed["tasks"][0]["pickup"].pop(key)
            with self.subTest(key=key), self.assertRaises(ValueError):
                MissionExecutor(changed, fleet, roles=roles, field_size_mm=(1143., 1181.))

    def test_small_target_drift_cannot_confirm_arrival_at_previous_endpoint(self):
        h = ObservedMissionHarness()
        h.positions["H1"] = [320., 300., 0.]
        for _ in range(3):
            event = h.step()
        self.assertEqual("approach", event["mission"]["phase"])
        self.assertEqual(1, h.session.mission.arrival_streak)
        for _ in range(4):
            event = h.step(objects=[object_candidate(404, 300, kind="disc", colour="white")])
        self.assertEqual(324., event["mission"]["route"][-1]["x_mm"])
        self.assertEqual("approach", event["mission"]["phase"])
        self.assertEqual(0, h.session.mission.arrival_streak)
        self.assertEqual(0, event["mission"]["observed_pickup"]["replans"])

    def test_sensor_stage_does_not_treat_occlusion_as_pickup_or_repeat_action(self):
        h = ObservedMissionHarness()
        h.positions["H1"] = [320., 300., 0.]
        for _ in range(8):
            event = h.step()
        self.assertEqual("close_servo", event["mission"]["phase"])
        command_id = event["mission"]["manipulator_intent"]["command_id"]
        for _ in range(5):
            event = h.step(objects=[])
        self.assertIsNone(event["closed_reason"])
        self.assertEqual("close_servo", event["mission"]["phase"])
        self.assertEqual(command_id, event["mission"]["manipulator_intent"]["command_id"])
        self.assertFalse(event["mission"]["observed_pickup_in_use"])
        self.assertEqual([], event["mission"]["completed_tasks"])

    def test_foreign_world_cannot_initialize_pickup_anchor(self):
        h = ObservedMissionHarness()
        for _ in range(2):
            h.step()
        world = h.session.world_adapter.poll(h.now)
        foreign = replace(world, ready=True, session_id="other")
        h.session.mission.observe_pickup(foreign)
        self.assertEqual("pickup_wrong_world_session", h.session.mission.fault)
        self.assertIsNone(h.session.mission.observed_pickup)


if __name__ == "__main__":
    unittest.main()
