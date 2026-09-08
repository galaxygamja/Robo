from __future__ import annotations

import copy
import unittest

from test_mission_runtime import MissionHarness, fixture
from test_world_state import (
    SESSION,
    SOURCE,
    TrackedFrames,
    make_adapter,
    object_candidate,
)

from robo_control.mission_bindings import MissionBindings, profile_identity
from robo_control.runtime_session import LiveControlSession
from robo_control.world_state import PieceSpec

PROFILE = "a" * 64


def binding_plan(piece_id="R1", kind="cylinder", colour="red", *, source=SOURCE):
    return {"schema_version": 1, "coordinate_system": "bottom_left_x_right_y_up_mm",
            "field_size_mm": [1143., 1181.], "source_name": source, "is_replay": False,
            "observation_profile_id": PROFILE, "reviewed": True,
            "reviewed_by": "unit-test-only", "evidence": "synthetic fixture, not a real review",
            "bindings": [{"piece_id": piece_id, "kind": kind, "colour": colour,
                          "region_mm": {"x_mm": 350., "y_mm": 250., "width_mm": 100., "height_mm": 100.}}]}


def manager(plan=None, pieces=None, required=("R1",)):
    return MissionBindings(plan or binding_plan(),
        pieces=pieces or [PieceSpec("R1", "cylinder", "red"), PieceSpec("D1", "disc")],
        field_size_mm=(1143., 1181.), source_name=SOURCE, is_replay=False,
        observation_profile_id=PROFILE, required_piece_ids=required)


class BindingPlanValidationTests(unittest.TestCase):
    def test_profile_canonicalization_and_each_input_affects_identity(self):
        result = profile_identity({"a": 1, "b": 2}, {"tags": [1]}, {"colors": [2]})
        self.assertEqual(result, profile_identity({"b": 2, "a": 1}, {"tags": [1]}, {"colors": [2]}))
        for args in (({"a": 2, "b": 2}, {"tags": [1]}, {"colors": [2]}),
                     ({"a": 1, "b": 2}, {"tags": [3]}, {"colors": [2]}),
                     ({"a": 1, "b": 2}, {"tags": [1]}, None)):
            self.assertNotEqual(result, profile_identity(*args))

    def test_draft_wrong_schema_and_unknown_fields_are_rejected(self):
        for patch in ({"schema_version": True}, {"schema_version": 2}, {"reviewed": False},
                      {"reviewed": 1}, {"reviewed_by": " "}, {"evidence": ""},
                      {"unknown": True}, {"coordinate_system": "metres"}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                manager({**binding_plan(), **patch})

    def test_source_replay_geometry_and_perception_are_pinned(self):
        for patch in ({"source_name": "webcam:another"}, {"is_replay": True}, {"is_replay": 0},
                      {"field_size_mm": [1, 2]}, {"field_size_mm": [True, 1181]},
                      {"observation_profile_id": "b"*64}, {"observation_profile_id": None}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                manager({**binding_plan(), **patch})

    def test_each_region_must_be_finite_positive_and_within_field(self):
        for patch in ({"x_mm": -1}, {"y_mm": True}, {"width_mm": 0}, {"height_mm": -1},
                      {"x_mm": float("nan")}, {"width_mm": float("inf")},
                      {"x_mm": 1100}, {"y_mm": 1180}, {"height_mm": 10**1000}, {"other": 5}):
            plan = binding_plan()
            plan["bindings"][0]["region_mm"].update(patch)
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                manager(plan)

    def test_catalog_coverage_and_duplicate_identity_are_rejected(self):
        for patch in ({"piece_id": "unknown"}, {"kind": "cube"}, {"colour": "green"},
                      {"colour": []}, {"piece_id": []}):
            plan = binding_plan()
            plan["bindings"][0].update(patch)
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                manager(plan)
        with self.assertRaises(ValueError):
            manager(required=("R1", "D1"))
        plan = binding_plan()
        plan["bindings"] *= 2
        with self.assertRaises(ValueError):
            manager(plan)

    def test_regions_cannot_touch_even_for_different_colours(self):
        for x in (400., 450.):
            plan = binding_plan()
            second = copy.deepcopy(plan["bindings"][0])
            second.update(piece_id="R2", colour="green")
            second["region_mm"]["x_mm"] = x
            plan["bindings"].append(second)
            with self.subTest(x=x), self.assertRaises(ValueError):
                manager(plan, [PieceSpec("R1", "cylinder", "red"), PieceSpec("R2", "cylinder", "green")])

    def test_plan_is_copied(self):
        plan = binding_plan()
        value = manager(plan)
        plan["bindings"][0]["piece_id"] = "unknown"
        self.assertEqual("R1", value.plan["bindings"][0]["piece_id"])


class BindingBatchTests(unittest.TestCase):
    def setUp(self):
        self.frames, self.adapter = TrackedFrames(), make_adapter()
        self.bindings = manager()

    def feed(self, seq, *, objects=None):
        now = 10. + seq * .02
        self.adapter.update(self.frames.frame(seq, now, objects=objects), now, source_session_id=SESSION)
        return self.bindings.update(self.adapter, now)

    def warm(self, **kwargs):
        for seq in range(1, 4):
            world = self.feed(seq, **kwargs)
        return world

    def test_only_three_confirmed_frames_allow_one_shot_binding(self):
        for seq in (1, 2, 3):
            world = self.feed(seq)
            self.assertEqual(seq == 3, self.bindings.activated)
        self.assertTrue(world.piece("R1").valid_for_pick)
        self.assertEqual((400., 300.), world.piece("R1").position_mm)
        self.assertEqual(3, self.bindings.snapshot()["applied_sequence"])
        self.assertFalse(self.bindings.snapshot()["physical_identity_verified"])
        world = self.feed(4)
        self.assertIsNone(self.bindings.require_pickup(world, "R1"))
        self.assertEqual(3, self.bindings.snapshot()["applied_sequence"])

    def test_empty_wrong_kind_or_multiple_occupants_do_not_fallback(self):
        variants = [([], "empty_region"), ([object_candidate(colour="green")], "kind_colour_mismatch"),
                    ([object_candidate(kind="disc")], "kind_colour_mismatch"),
                    ([object_candidate(380, 280), object_candidate(430, 320, colour="green")],
                     "multiple_region_occupants")]
        for objects, reason in variants:
            with self.subTest(reason=reason):
                self.setUp()
                world = self.warm(objects=objects)
                self.assertFalse(self.bindings.activated)
                self.assertIsNone(world.piece("R1").track_id)
                self.assertEqual(reason, self.bindings.snapshot()["candidates"][0]["reason"])

    def test_tentative_competitor_prevents_binding(self):
        self.feed(1)
        self.feed(2)
        world = self.feed(3, objects=[object_candidate(), object_candidate(430, 340, colour="green")])
        self.assertTrue(world.ready)
        self.assertFalse(self.bindings.activated)
        self.assertEqual("multiple_region_occupants", self.bindings.snapshot()["candidates"][0]["reason"])

    def test_region_does_not_seed_measurement(self):
        world = self.warm(objects=[])
        self.assertIsNone(world.piece("R1").position_mm)
        self.assertIsNone(world.piece("R1").track_id)

    def test_missing_bound_target_latches_and_reappearance_does_not_rebind(self):
        original = self.warm().piece("R1").track_id
        world = self.feed(4, objects=[])
        self.assertEqual("pickup_binding_lost:R1", self.bindings.require_pickup(world, "R1"))
        for seq in (5, 6, 7):
            world = self.feed(seq)
        self.assertEqual(original, world.piece("R1").track_id)
        self.assertEqual("pickup_binding_lost:R1", self.bindings.require_pickup(world, "R1"))

    def test_finished_pickup_is_not_used_as_grip_or_delivery_evidence(self):
        self.warm()
        world = self.feed(4, objects=[])
        self.assertIsNone(self.bindings.require_pickup(world, None))
        self.assertFalse(world.piece("R1").valid_for_pick)
        self.assertNotIn("score", self.bindings.snapshot())

    def test_manual_unbind_during_pickup_is_not_silently_repaired(self):
        self.warm()
        world = self.adapter.unbind_piece("R1", 10.07)
        self.assertEqual("pickup_binding_lost:R1", self.bindings.require_pickup(world, "R1"))

    def test_preexisting_external_binding_is_not_taken_over(self):
        for seq in (1, 2, 3):
            now = 10.+seq*.02
            world = self.adapter.update(self.frames.frame(seq, now), now, source_session_id=SESSION)
        self.adapter.bind_piece("R1", world.objects[0].object_id, now, evidence="other operator")
        self.bindings.update(self.adapter, now)
        self.assertEqual("binding_batch_rejected", self.bindings.fault)

    def test_batch_failure_changes_no_bindings_even_with_valid_first_row(self):
        for seq in (1, 2, 3):
            now = 10.+seq*.02
            world = self.adapter.update(self.frames.frame(seq, now), now, source_session_id=SESSION)
        oid = world.objects[0].object_id
        for batch in ([("R1", oid, "review"), ("unknown", oid, "review")],
                      [("R1", oid, "review"), ("R1", oid, "duplicate")],
                      [("R1", oid, "review"), ["broken"]], [], None):
            with self.subTest(batch=batch), self.assertRaises(ValueError):
                self.adapter.bind_pieces(batch, now)
            self.assertIsNone(self.adapter.poll(now).piece("R1").track_id)

    def test_whole_batch_waits_for_all_regions(self):
        plan = binding_plan()
        plan["bindings"].append({"piece_id": "D1", "kind": "disc", "colour": "white",
                                "region_mm": {"x_mm": 550, "y_mm": 250, "width_mm": 100, "height_mm": 100}})
        self.bindings = manager(plan, required=("R1", "D1"))
        world = self.warm()
        self.assertIsNone(world.piece("R1").track_id)
        for seq in (4, 5, 6):
            world = self.feed(seq, objects=[object_candidate(), object_candidate(600, 300, kind="disc", colour="white")])
        self.assertTrue(self.bindings.activated)
        self.assertTrue(world.piece("R1").valid_for_pick)
        self.assertTrue(world.piece("D1").valid_for_pick)

    def test_snapshot_mutation_has_no_effect(self):
        self.warm()
        payload = self.bindings.snapshot()
        payload["mappings"].clear()
        payload["candidates"].clear()
        self.assertIn("R1", self.bindings.snapshot()["mappings"])
        self.assertTrue(self.bindings.snapshot()["candidates"])


class BoundMissionHarness(MissionHarness):
    def __init__(self, *, wire_fake=False, wire_options=None):
        super().__init__()
        fleet, roles, plan = fixture()
        plan["tasks"][0]["pickup"]["x_mm"] = 450.
        self.session = LiveControlSession(roles=roles, field_size_mm=(1143., 1181.),
            source_name=SOURCE, session_id="mission-test", mission_plan=plan, fleet=fleet,
            track_objects=True, binding_plan=binding_plan("D1", "disc", "white"),
            observation_profile_id=PROFILE, wire_fake=wire_fake, wire_options=wire_options)

    def step(self, *, objects=None, **kwargs):
        extra = kwargs.pop("change", None)

        def change(row):
            row["objects"] = ([object_candidate(kind="disc", colour="white")] if objects is None else objects)
            if extra:
                extra(row)
        return super().step(change=change, **kwargs)


class BindingRuntimeTests(unittest.TestCase):
    def test_observation_only_keeps_tasks_unstarted_and_outputs_zero(self):
        h = MissionHarness()
        fleet, roles, plan = fixture()
        h.session = LiveControlSession(roles=roles, source_name=SOURCE, field_size_mm=(1143., 1181.),
            mission_plan=plan, fleet=fleet, track_objects=True, mission_observe_only=True)
        for _ in range(20):
            event = h.step(change=lambda r: r.update(objects=[object_candidate()]))
        self.assertTrue(event["mission_observe_only"])
        self.assertTrue(event["mission"]["world"]["ready"])
        self.assertTrue(event["mission"]["world"]["objects"][0]["valid_for_pick"])
        self.assertIsNone(event["mission"]["active_task_id"])
        self.assertIsNone(event["mission"]["manipulator_intent"])
        self.assertEqual(0, event["mission"]["elapsed_s"])
        self.assertIsNone(h.session.mission.started_at)
        self.assertTrue(all(r["forward_velocity_mm_s"] == 0 and r["angular_velocity_rad_s"] == 0
                            for r in event["actuator"]["robots"]))

    def test_observation_only_cannot_also_bind_or_arm(self):
        fleet, roles, plan = fixture()
        for patch in ({"wire_fake": True}, {"binding_plan": binding_plan()}, {"mission_observe_only": 1}):
            args = {"roles": roles, "source_name": SOURCE, "field_size_mm": (1143., 1181.),
                    "mission_plan": plan, "fleet": fleet, "track_objects": True, "mission_observe_only": True}
            args.update(patch)
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                LiveControlSession(**args)

    def test_options_require_mission_and_tracking(self):
        for args in ({}, {"track_objects": True}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                LiveControlSession(roles={"H1": "hamster"}, source_name=SOURCE,
                    field_size_mm=(1143., 1181.), binding_plan=binding_plan(), **args)

    def test_missing_mapping_never_starts_mission_or_fake_wire(self):
        h = BoundMissionHarness(wire_fake=True)
        for _ in range(30):
            event = h.step(objects=[])
        self.assertEqual("awaiting_piece_bindings", event["status"])
        self.assertIsNone(event["mission"]["active_task_id"])
        self.assertFalse(event["wire"]["started"])
        self.assertTrue(all(r["forward_velocity_mm_s"] == 0 for r in event["actuator"]["robots"]))

    def test_bound_objects_enable_existing_motion_without_hardware_claim(self):
        h = BoundMissionHarness(wire_fake=True)
        for _ in range(6):
            event = h.step()
        self.assertEqual("bound", event["bindings"]["status"])
        self.assertEqual("approach", event["mission"]["phase"])
        self.assertTrue(any(r["forward_velocity_mm_s"] > 0 for r in event["actuator"]["robots"]))
        self.assertFalse(event["motion_permitted"])

    def test_target_loss_closes_and_cancels_queued_drive_before_delivery(self):
        h = BoundMissionHarness(wire_fake=True)
        for _ in range(6):
            h.step()
        h.session.wire.configure_link("H1", h.now, request_delay_s=.04)
        queued = h.step()
        count = next(r["receiver"]["accepted_drive_count"] for r in queued["wire"]["robots"] if r["robot_id"] == "H1")
        h.step()
        event = h.step(objects=[])
        self.assertEqual("pickup_binding_lost:D1", event["closed_reason"])
        actual = next(r["receiver"]["accepted_drive_count"] for r in event["wire"]["robots"] if r["robot_id"] == "H1")
        self.assertEqual(count, actual)
        self.assertTrue(all(r["stop_requested"] for r in event["wire"]["robots"]))

    def test_polling_does_not_bind_or_extend_object_evidence(self):
        h = BoundMissionHarness()
        for _ in range(6):
            event = h.step()
        sequence = event["bindings"]["applied_sequence"]
        for _ in range(11):
            event = h.step(no_record=True)
        self.assertEqual(sequence, event["bindings"]["applied_sequence"])
        self.assertFalse(event["mission"]["world"]["ready"])
        self.assertTrue(all(r["forward_velocity_mm_s"] == 0 for r in event["actuator"]["robots"]))

    def test_explicit_wire_reconnect_allows_three_new_frames_without_rebinding(self):
        h = BoundMissionHarness(wire_fake=True)
        for _ in range(6):
            event = h.step()
        original = event["bindings"]["mappings"]
        h.session.wire.stop(h.now, "test_transport_fault")
        h.step()
        h.session.reconnect_wire(h.now, operator_confirmed=True)
        for _ in range(3):
            event = h.step()
            self.assertIsNone(event["closed_reason"])
        self.assertIsNone(h.session.wire.fault)
        self.assertEqual(original, event["bindings"]["mappings"])

    def test_missing_target_during_reconnect_is_rejected_once_world_is_ready(self):
        h = BoundMissionHarness(wire_fake=True)
        for _ in range(6):
            h.step()
        h.session.wire.stop(h.now, "test_transport_fault")
        h.step()
        h.session.reconnect_wire(h.now, operator_confirmed=True)
        for _ in range(3):
            event = h.step(objects=[])
        self.assertEqual("pickup_binding_lost:D1", event["closed_reason"])


if __name__ == "__main__":
    unittest.main()
