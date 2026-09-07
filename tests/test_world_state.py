from __future__ import annotations

import copy
import json
import math
import unittest
from dataclasses import FrozenInstanceError

from robo_control.fleet import DEFAULT_ROLES
from robo_control.qualifier import Piece
from robo_control.vision.object_tracking import ObjectTracker
from robo_control.vision.tracking import PoseTracker
from robo_control.world_state import ObservationWorldAdapter, PieceSpec

ROLES = dict(DEFAULT_ROLES)
SOURCE, SESSION = "webcam:test", "world-test-session"


def object_candidate(x=400., y=300., *, colour="red", kind="cylinder"):
    return {"kind": kind, "color": colour, "center_mm": [x, y]}


class TrackedFrames:
    def __init__(self):
        self.poses = PoseTracker(ROLES)
        self.objects = ObjectTracker(ROLES, max_frame_age_s=.2)

    def frame(self, sequence, stamp, *, objects=None, missing=(), changes=None):
        raw = {"status": "detected", "sequence": sequence, "captured_at_s": stamp,
            "received_at_s": stamp, "source_name": SOURCE, "is_replay": False,
            "configuration_id": "config-A", "coordinate_system": "bottom_left_x_right_y_up_mm",
            "field_size_mm": [1143., 1181.], "registered_robot_ids": list(ROLES), "device_io": False,
            "observation_complete": not missing, "unknown_tag_ids": [], "duplicate_tag_ids": [],
            "robots": [{"robot_id": rid, "robot_center_mm": [150. + 200*i, 500.], "heading_rad": .25}
                       for i, rid in enumerate(ROLES) if rid not in missing],
            "objects": [object_candidate()] if objects is None else objects}
        raw.update(changes or {})
        raw.update(self.poses.update(raw, stamp))
        raw.update(self.objects.update(raw, stamp))
        return raw


def make_adapter(**changes):
    args = {"roles": ROLES, "pieces": [PieceSpec("R1", "cylinder", "red"), PieceSpec("D1", "disc")],
        "source_name": SOURCE, "session_id": SESSION, "field_size_mm": (1143., 1181.),
        "configuration_id": "config-A"}
    args.update(changes)
    return ObservationWorldAdapter(**args)


class WorldStateTests(unittest.TestCase):
    def setUp(self):
        self.frames, self.adapter = TrackedFrames(), make_adapter()

    def feed(self, seq, stamp, **kwargs):
        return self.adapter.update(self.frames.frame(seq, stamp, **kwargs), stamp, source_session_id=SESSION)

    def warm(self, **kwargs):
        for seq, stamp in ((1, 10.), (2, 10.05), (3, 10.1)):
            world = self.feed(seq, stamp, **kwargs)
        return world

    def bind(self):
        world = self.warm()
        oid = world.objects[0].object_id
        return self.adapter.bind_piece("R1", oid, 10.1, evidence="operator selected this observed object"), oid

    def test_empty_state_has_no_scenario_pose_battery_or_success(self):
        self.adapter = make_adapter(pieces=[PieceSpec.from_piece(Piece("R1", "cylinder", 999, 888, "red"))])
        world = self.adapter.poll(10.)
        self.assertFalse(world.ready)
        self.assertTrue(all(r.position_mm is None and r.heading_rad is None for r in world.robots))
        self.assertIsNone(world.piece("R1").position_mm)
        self.assertIsNone(world.piece("R1").yaw_rad)
        payload = world.as_dict()
        self.assertNotIn("battery_percent", payload["robots"][0])
        self.assertNotIn("released", payload["pieces"][0])
        self.assertNotIn("score", payload)
        self.assertFalse(payload["motion_permitted"])

    def test_requires_three_measured_frames_and_preserves_roles_units(self):
        for seq in (1, 2, 3):
            world = self.feed(seq, 10. + seq * .02)
            self.assertEqual(seq == 3, world.ready)
            self.assertEqual(seq == 3, world.robot("H1").valid_for_control)
        self.assertEqual("beaver", world.robot("H2").role)
        self.assertEqual((150., 500.), world.robot("H1").position_mm)
        self.assertEqual((.15, .5), world.robot("H1").position_m)
        self.assertAlmostEqual(.25, world.robot("H1").heading_rad)
        self.assertIsNone(world.robot("unknown"))
        self.assertIsNone(world.piece("unknown"))

    def test_poll_does_not_count_or_renew_frames_and_exact_200ms_expires(self):
        self.feed(1, 10.)
        for stamp in (10., 10.01, 10.02):
            world = self.adapter.poll(stamp)
            self.assertEqual(1, world.fresh_streak)
        self.feed(2, 10.05)
        self.feed(3, 10.1)
        self.assertTrue(self.adapter.poll(10.299).ready)
        stale = self.adapter.poll(10.3)
        self.assertFalse(stale.ready)
        self.assertIn("observation_expired", stale.reasons)
        self.assertFalse(stale.objects[0].position_valid)
        self.assertFalse(stale.objects[0].valid_for_pick)
        self.assertEqual(3, stale.source_sequence)
        self.assertEqual(10.1, stale.captured_at_s)
        self.assertFalse(self.feed(4, 10.31).ready)
        self.assertFalse(self.feed(5, 10.35).ready)
        self.assertTrue(self.feed(6, 10.4).ready)

    def test_input_and_output_mutation_cannot_change_saved_observations(self):
        self.warm()
        raw = self.frames.frame(4, 10.15)
        original = copy.deepcopy(raw)
        world = self.adapter.update(raw, 10.15, source_session_id=SESSION)
        self.assertEqual(original, raw)
        raw["robots"][0]["robot_center_mm"][0] = 999.
        raw["tracks"][0]["robot_center_mm"][0] = 888.
        payload = world.as_dict()
        payload["robots"][0]["position_mm"] = [777., 888.]
        self.assertEqual((150., 500.), self.adapter.poll(10.16).robot("H1").position_mm)
        with self.assertRaises(FrozenInstanceError):
            world.robots[0].role = "beaver"
        json.dumps(payload, allow_nan=False)

    def test_known_missing_robot_immediately_blocks_but_old_position_is_diagnostic(self):
        self.warm()
        world = self.feed(4, 10.15, missing=("H1",))
        self.assertFalse(world.ready)
        self.assertEqual("missing", world.robot("H1").state)
        self.assertEqual((150., 500.), world.robot("H1").position_mm)
        self.assertFalse(world.robot("H1").valid_for_control)
        self.assertFalse(world.objects[0].valid_for_pick)
        for seq in (5, 6, 7):
            self.assertEqual(seq == 7, self.feed(seq, 10.15 + (seq - 4)*.02).ready)

    def test_duplicate_sequence_and_rejected_frame_block_without_reviving_old_data(self):
        self.warm()
        row = self.frames.frame(4, 10.15)
        row["status"] = "rejected_frame"
        self.assertFalse(self.adapter.update(row, 10.15, source_session_id=SESSION).ready)
        row["status"] = "detected"
        rejected = self.adapter.update(row, 10.16, source_session_id=SESSION)
        self.assertIn("out_of_order_sequence", rejected.reasons)
        self.assertFalse(rejected.ready)

    def test_provenance_change_closes_the_session_permanently(self):
        for patch in ({"source_name": "another"}, {"is_replay": True}, {"is_replay": 0},
                      {"configuration_id": "config-B"}, {"coordinate_system": "metres"},
                      {"field_size_mm": [1.143, 1.181]}, {"device_io": True},
                      {"registered_robot_ids": ["H1"]}, {"tracking_session_closed": True}):
            with self.subTest(patch=patch):
                adapter, frames = make_adapter(), TrackedFrames()
                row = frames.frame(1, 10.)
                row.update(patch)
                self.assertEqual("closed", adapter.update(row, 10., source_session_id=SESSION).status)
                for seq in (2, 3, 4):
                    self.assertFalse(adapter.update(frames.frame(seq, 10.+seq*.02),
                        10.+seq*.02, source_session_id=SESSION).ready)

    def test_foreign_outer_session_and_source_close_latch(self):
        self.warm()
        closed = self.adapter.update(None, 10.11, source_session_id="restarted-source")
        self.assertEqual("closed", closed.status)
        self.assertFalse(self.adapter.poll(10.12).ready)
        self.adapter = make_adapter()
        closed = self.adapter.update({"status": "source_closed"}, 10., source_session_id=SESSION)
        self.assertEqual("closed", closed.status)
        self.assertEqual("closed", self.adapter.close(10.1).status)

    def test_invalid_clock_raises_and_latches_no_recovery(self):
        for stamp in (True, -1, float("nan"), float("inf"), 9.):
            with self.subTest(stamp=stamp):
                adapter = make_adapter()
                adapter.poll(10.)
                with self.assertRaises(ValueError):
                    adapter.poll(stamp)
                self.assertEqual("closed", adapter.poll(10.1).status)

    def test_bad_times_and_payloads_are_atomic_and_never_usable(self):
        changes = [{"received_at_s": 100.}, {"captured_at_s": True}, {"captured_at_s": 9.},
            {"received_at_s": None}, {"tracks": []}, {"tracks": [None]}, {"robots": None},
            {"objects": [None]}, {"object_tracks": None}, {"object_tracking_device_io": True}]
        for patch in changes:
            with self.subTest(patch=patch):
                adapter, frames = make_adapter(), TrackedFrames()
                row = frames.frame(1, 10.)
                row.update(patch)
                world = adapter.update(row, 10., source_session_id=SESSION)
                self.assertFalse(world.ready)
                self.assertTrue(all(r.position_mm is None for r in world.robots))

    def test_mutated_raw_robot_or_object_measurement_is_rejected(self):
        mutations = [lambda r: r["robots"][0]["robot_center_mm"].__setitem__(0, 999.),
            lambda r: r["tracks"][0].__setitem__("heading_rad", math.nan),
            lambda r: r["tracks"][0].__setitem__("observed_at_s", 8.),
            lambda r: r["objects"][0]["center_mm"].__setitem__(0, 401.),
            lambda r: r["object_tracks"].append(copy.deepcopy(r["object_tracks"][0])),
            lambda r: r["objects"].append(copy.deepcopy(r["objects"][0])),
            lambda r: r["object_tracks"][0].__setitem__("center_mm", [-1., 2.])]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                self.frames, self.adapter = TrackedFrames(), make_adapter()
                self.warm()
                row = self.frames.frame(4, 10.15)
                mutate(row)
                world = self.adapter.update(row, 10.15, source_session_id=SESSION)
                self.assertFalse(world.ready)
                self.assertEqual((150., 500.), world.robot("H1").position_mm)
                self.assertFalse(world.objects[0].valid_for_pick)

    def test_malformed_last_object_cannot_partially_commit_new_robot_pose(self):
        self.warm()
        row = self.frames.frame(4, 10.15)
        row["robots"][0]["robot_center_mm"][0] = 160.
        row["tracks"][0]["robot_center_mm"][0] = 160.
        row["object_tracks"].append({"object_id": "broken"})
        world = self.adapter.update(row, 10.15, source_session_id=SESSION)
        self.assertFalse(world.ready)
        self.assertEqual((150., 500.), world.robot("H1").position_mm)

    def test_no_automatic_colour_or_scenario_position_binding(self):
        world = self.warm()
        self.assertTrue(world.objects[0].valid_for_pick)
        self.assertIsNone(world.piece("R1").track_id)
        self.assertIsNone(world.piece("R1").position_mm)
        self.assertFalse(world.piece("R1").valid_for_pick)

    def test_explicit_binding_is_one_to_one_and_preserves_measured_values(self):
        world, oid = self.bind()
        piece = world.piece("R1")
        self.assertEqual(oid, piece.track_id)
        self.assertEqual((400., 300.), piece.position_mm)
        self.assertIsNone(piece.yaw_rad)
        self.assertTrue(piece.valid_for_pick)
        for args in (("R1", oid, "again"), ("D1", oid, "wrong-kind"), ("unknown", oid, "unknown"),
                     ("D1", "unknown", "unknown"), ("D1", oid, "")):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.adapter.bind_piece(args[0], args[1], 10.1, evidence=args[2])
        self.assertIsNone(self.adapter.unbind_piece("R1", 10.11).piece("R1").position_mm)

    def test_two_same_colour_mission_ids_cannot_share_one_track(self):
        self.adapter = make_adapter(pieces=[PieceSpec("R1", "cylinder", "red"), PieceSpec("R2", "cylinder", "red")])
        _, oid = self.bind()
        with self.assertRaises(ValueError):
            self.adapter.bind_piece("R2", oid, 10.1, evidence="duplicate")

    def test_missing_is_not_pickable_and_reappearance_needs_three_object_frames(self):
        self.bind()
        self.assertFalse(self.feed(4, 10.15, objects=[]).piece("R1").valid_for_pick)
        for seq in (5, 6, 7):
            piece = self.feed(seq, 10.15 + (seq-4)*.02).piece("R1")
            self.assertEqual(seq == 7, piece.valid_for_pick)

    def test_lost_track_is_not_silently_rebound_to_new_same_colour_id(self):
        _, old = self.bind()
        self.feed(4, 10.7, objects=[])
        for seq in (5, 6, 7):
            world = self.feed(seq, 10.7 + (seq-4)*.05)
        self.assertEqual(old, world.piece("R1").track_id)
        self.assertFalse(world.piece("R1").valid_for_pick)
        self.assertEqual("bound_track_identity_lost", world.piece("R1").reason)
        new = next(o.object_id for o in world.objects if o.valid_for_pick)
        self.assertNotEqual(old, new)
        self.adapter.unbind_piece("R1", 10.85)
        self.assertTrue(self.adapter.bind_piece("R1", new, 10.85, evidence="explicit remapping").piece("R1").valid_for_pick)

    def test_ambiguous_binding_stays_broken_even_if_upstream_claims_recovery(self):
        _, oid = self.bind()
        row = self.frames.frame(4, 10.15)
        row["object_tracks"][0]["identity_uncertain"] = True
        self.adapter.update(row, 10.15, source_session_id=SESSION)
        for seq in (5, 6, 7):
            world = self.feed(seq, 10.15 + (seq-4)*.02)
        self.assertEqual(oid, world.piece("R1").track_id)
        self.assertFalse(world.piece("R1").valid_for_pick)
        self.assertEqual("bound_track_identity_lost", world.piece("R1").reason)

    def test_bound_track_colour_cannot_change_and_then_repair_itself(self):
        self.bind()
        row = self.frames.frame(4, 10.15)
        row["objects"][0]["color"] = "green"
        row["object_tracks"][0]["color"] = "green"
        self.adapter.update(row, 10.15, source_session_id=SESSION)
        self.assertFalse(self.feed(5, 10.2).piece("R1").valid_for_pick)

    def test_release_hint_is_never_a_measured_position_or_success(self):
        _, oid = self.bind()
        self.frames.objects.mark_gripped(oid, "B1", 10.11, evidence="fake sensor assertion for unit test")
        owned = self.feed(4, 10.15).piece("R1")
        self.assertEqual("B1", owned.owner_robot_id)
        self.assertFalse(owned.valid_for_pick)
        self.frames.objects.mark_released(oid, "B1", 10.16, center_mm=(800, 900), evidence="unit-test hint")
        row = self.frames.frame(5, 10.17, objects=[])
        pending = self.adapter.update(row, 10.17, source_session_id=SESSION).piece("R1")
        self.assertIsNone(pending.position_mm)
        self.assertFalse(pending.position_valid)
        self.assertFalse(pending.valid_for_pick)
        self.assertEqual("released", pending.lifecycle)
        for seq in (6, 7, 8):
            world = self.feed(seq, 10.17 + (seq-5)*.02, objects=[object_candidate(800, 900)])
        self.assertEqual((800., 900.), world.piece("R1").position_mm)
        self.assertTrue(world.piece("R1").position_valid)
        self.assertNotIn("released", world.as_dict()["pieces"][0])

    def test_cube_yaw_is_unknown_even_with_scenario_angle(self):
        self.adapter = make_adapter(pieces=[PieceSpec.from_piece(Piece("C1", "cube", 99, 88, yaw_rad=1.2))])
        world = self.warm(objects=[object_candidate(colour="white", kind="cube")])
        world = self.adapter.bind_piece("C1", world.objects[0].object_id, 10.1, evidence="manual cube mapping")
        self.assertIsNone(world.piece("C1").yaw_rad)
        self.assertIsNone(world.piece("C1").owner_robot_id)

    def test_replay_host_age_and_media_order_are_separate(self):
        self.adapter = make_adapter(is_replay=True)
        for seq in (1, 2, 3):
            stamp = 10. + seq * .001
            world = self.feed(seq, stamp, changes={"is_replay": True, "media_time_s": seq*.05})
        self.assertTrue(world.ready)
        self.assertTrue(world.is_replay)
        bad = self.feed(4, 10.004, changes={"is_replay": True, "media_time_s": .1})
        self.assertFalse(bad.ready)
        self.assertIn("invalid_media_time", bad.reasons)

    def test_constructor_rejects_invalid_registry_catalog_and_deadlines(self):
        for patch in ({"roles": {}}, {"session_id": ""}, {"source_name": ""}, {"is_replay": 1},
                      {"field_size_mm": [0, 1]}, {"max_age_s": .5}, {"max_age_s": True},
                      {"recovery_frames": 2}, {"recovery_frames": True}, {"pieces": [None]},
                      {"pieces": [PieceSpec("R1", "cylinder", "red")]*2}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                make_adapter(**patch)
        for values in (("", "disc", None), ("R1", "cylinder", None), ("R1", "other", None)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                PieceSpec(*values)

    def test_snapshot_cannot_be_used_as_a_motion_permission(self):
        world, _ = self.bind()
        self.assertTrue(world.ready)
        payload = world.as_dict()
        for field in ("device_io", "hardware_ready", "motion_permitted", "physical_identity_verified"):
            self.assertIs(False, payload[field])
        self.assertFalse(self.adapter.close(10.2).ready)
        # Historical immutable snapshots remain historical; consumers must poll.
        self.assertTrue(world.ready)
        self.assertFalse(self.adapter.poll(10.3).ready)

    def test_explicit_upstream_rejection_invalidates_poll_and_needs_recovery(self):
        self.bind()
        rejected = self.adapter.invalidate(10.11, reason="upstream_rejected_source")
        self.assertFalse(rejected.ready)
        self.assertFalse(rejected.piece("R1").position_valid)
        self.assertFalse(self.adapter.poll(10.12).ready)
        for seq in (4, 5, 6):
            self.assertEqual(seq == 6, self.feed(seq, 10.12+(seq-3)*.02).ready)
        self.adapter.close(10.2)
        self.assertEqual("closed", self.adapter.invalidate(10.21, reason="cannot_reopen").status)

    def test_expired_or_tentative_object_cannot_be_bound(self):
        first = self.feed(1, 10.)
        with self.assertRaises(ValueError):
            self.adapter.bind_piece("R1", first.objects[0].object_id, 10., evidence="too soon")
        self.feed(2, 10.05)
        world = self.feed(3, 10.1)
        with self.assertRaises(ValueError):
            self.adapter.bind_piece("R1", world.objects[0].object_id, 10.3, evidence="expired")

    def test_individual_duplicate_robot_and_incomplete_flags_block(self):
        for change in ("duplicate", "flag", "unknown_tag", "wrong_stamp", "false_boolean"):
            with self.subTest(change=change):
                adapter, frames = make_adapter(), TrackedFrames()
                row = frames.frame(1, 10.)
                if change == "duplicate":
                    row["tracks"][-1] = copy.deepcopy(row["tracks"][0])
                elif change == "flag":
                    row["observation_complete"] = False
                elif change == "unknown_tag":
                    row["unknown_tag_ids"] = [999]
                elif change == "wrong_stamp":
                    row["tracks"][0]["observed_at_s"] = 100.
                else:
                    row["tracks"][0]["valid_for_control"] = 1
                self.assertFalse(adapter.update(row, 10., source_session_id=SESSION).ready)

    def test_blank_or_malformed_input_keeps_unknowns_and_returns_serializable_stop(self):
        for row in (None, [], 1, {}, {"status": "detected"}):
            with self.subTest(row=row):
                world = make_adapter().update(row, 10., source_session_id=SESSION)
                self.assertFalse(world.ready)
                self.assertTrue(all(r.position_mm is None for r in world.robots))
                json.dumps(world.as_dict(), allow_nan=False)

    def test_lost_object_removed_from_upstream_is_not_repaired_by_reusing_its_id(self):
        self.bind()
        row = self.frames.frame(4, 10.15)
        row["objects"], row["object_tracks"] = [], []
        self.adapter.update(row, 10.15, source_session_id=SESSION)
        self.assertFalse(self.feed(5, 10.2).piece("R1").valid_for_pick)
        self.assertEqual("bound_track_identity_lost", self.adapter.poll(10.21).piece("R1").reason)


if __name__ == "__main__":
    unittest.main()
