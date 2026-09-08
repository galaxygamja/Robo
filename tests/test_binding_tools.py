from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from test_mission_bindings import PROFILE, BoundMissionHarness, binding_plan
from test_mission_runtime import MissionHarness, fixture
from test_world_state import SOURCE, object_candidate

from robo_control.binding_tools import create_draft, main, review_snapshot
from robo_control.mission_bindings import load_binding_plan
from robo_control.runtime_report import inspect_report
from robo_control.runtime_session import LiveControlSession


def recorded_rows(*, bound=False, missing=False):
    h = BoundMissionHarness() if bound else MissionHarness()
    fleet, roles, plan = fixture()
    if not bound:
        h.session = LiveControlSession(roles=roles, source_name=SOURCE, field_size_mm=(1143., 1181.),
            mission_plan=plan, fleet=fleet, track_objects=True, mission_observe_only=True,
            session_id="mission-test", configuration_id="recording-profile")
    rows = [{"schema_version": 1, "event": "session_started", "session_id": "mission-test",
             "configuration_id": h.session.configuration_id, "roles": roles,
             "source_name": SOURCE, "is_replay": False, "input_mode": "live_camera",
             "output_mode": "dry_run_commands", "transport_mode": "mock", "drive_model": "differential_body",
             "mission_observe_only": not bound, "mission_plan": plan,
             "binding_plan": binding_plan("D1", "disc", "white") if bound else None,
             "observation_profile_id": PROFILE, "device_io": False, "motion_permitted": False}]
    for _ in range(5):
        if bound:
            event = h.step(objects=[] if missing else None)
        else:
            event = h.step(change=lambda r: r.update(configuration_id="recording-profile",
                           objects=[] if missing else [object_candidate(kind="disc", colour="white")]))
        rows.append(event)
    rows.append(h.session.close(h.now+.01, "operator_stop"))
    return rows


class BindingToolsTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.path = self.root / "session.jsonl"
        self.rows = recorded_rows()
        self.save()

    def save(self, rows=None):
        self.path.write_text("".join(json.dumps(row)+"\n" for row in (self.rows if rows is None else rows)), encoding="utf-8")

    def selection(self):
        return "D1="+review_snapshot(self.path)["objects"][0]["object_id"]

    def test_inspect_uses_latest_ready_not_closed_world_and_is_historical(self):
        result = review_snapshot(self.path)
        self.assertEqual(5, result["source_sequence"])
        self.assertTrue(result["historical_only"])
        self.assertFalse(result["physical_identity_verified"])
        self.assertFalse(result["motion_permitted"])
        self.assertEqual([400., 300.], result["objects"][0]["position_mm"])

    def test_draft_requires_explicit_selection_and_never_copies_track_ids(self):
        result = create_draft(self.path, [self.selection()], half_size_mm=15)
        self.assertFalse(result["reviewed"])
        self.assertEqual("", result["reviewed_by"])
        self.assertEqual("", result["evidence"])
        self.assertEqual(PROFILE, result["observation_profile_id"])
        self.assertEqual({"x_mm": 385., "y_mm": 285., "width_mm": 30, "height_mm": 30},
                         result["bindings"][0]["region_mm"])
        self.assertNotIn("track_id", json.dumps(result))

    def test_draft_is_rejected_as_runtime_review(self):
        result = create_draft(self.path, [self.selection()], half_size_mm=15)
        fleet, roles, plan = fixture()
        with self.assertRaisesRegex(ValueError, "reviewed"):
            LiveControlSession(roles=roles, source_name=SOURCE, field_size_mm=(1143., 1181.),
                mission_plan=plan, fleet=fleet, track_objects=True, binding_plan=result,
                observation_profile_id=PROFILE)

    def test_missing_unknown_duplicate_and_wrong_class_selection_rejected(self):
        valid = self.selection()
        oid = valid.split("=")[1]
        for selection in ([], ["D1"], ["D1=x=y"], ["unknown="+oid], ["D1=missing"],
                          ["R1="+oid], [valid, valid], [valid, "D2="+oid]):
            with self.subTest(selection=selection), self.assertRaises(ValueError):
                create_draft(self.path, selection, half_size_mm=15)

    def test_nonfinite_or_out_of_field_region_is_rejected_not_clipped(self):
        for r in (True, 0, -1, float("nan"), float("inf"), 500., 10**1000):
            with self.subTest(r=r), self.assertRaises(ValueError):
                create_draft(self.path, [self.selection()], half_size_mm=r)

    def test_open_report_and_no_ready_world_are_rejected(self):
        self.save(self.rows[:-1])
        with self.assertRaises(ValueError):
            review_snapshot(self.path)
        self.save([self.rows[0], self.rows[1], self.rows[-1]])
        with self.assertRaisesRegex(ValueError, "No ready"):
            review_snapshot(self.path)

    def test_missing_profile_cannot_make_new_runtime_plan(self):
        self.rows[0].pop("observation_profile_id")
        self.save()
        with self.assertRaises(ValueError):
            create_draft(self.path, [self.selection()], half_size_mm=15)

    def test_wrong_world_provenance_is_rejected(self):
        for change in ({"session_id": "other"}, {"generated_at_s": 0}, {"source_name": "other"},
                       {"configuration_id": "other"}, {"is_replay": True}, {"physical_identity_verified": True}):
            rows = copy.deepcopy(self.rows)
            rows[-2]["mission"]["world"].update(change)
            self.save(rows)
            with self.subTest(change=change), self.assertRaises(ValueError):
                review_snapshot(self.path)

    def test_duplicate_json_keys_are_rejected(self):
        text = self.path.read_text(encoding="utf-8")
        self.path.write_text(text.replace('"schema_version": 1', '"schema_version": 1, "schema_version": 1', 1), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            review_snapshot(self.path)

    def test_cli_never_overwrites_existing_file_or_source(self):
        output = self.root / "draft.json"
        selection = self.selection()
        args = ["draft", str(self.path), "--select", selection, "--half-size-mm", "15", "--output", str(output)]
        with patch("sys.stdout"), patch("sys.stderr"):
            self.assertEqual(0, main(args))
            original = output.read_bytes()
            self.assertEqual(2, main(args))
            self.assertEqual(original, output.read_bytes())
            original_report = self.path.read_bytes()
            self.assertEqual(2, main([*args[:-1], str(self.path)]))
            self.assertEqual(original_report, self.path.read_bytes())

    def test_inspection_is_read_only(self):
        original = self.path.read_bytes()
        with patch("sys.stdout"):
            self.assertEqual(0, main(["inspect", str(self.path)]))
        self.assertEqual(original, self.path.read_bytes())

    def test_binding_loader_rejects_duplicate_nonfinite_oversized_and_invalid_utf8(self):
        path = self.root/"plan.json"
        for text in (b'{"a": 1, "a": 2}', b'{"a": NaN}', b'{"a": 1e9999}',
                     b" "*65537, b'{"a": "\xff"}', b"["*2000+b"]"*2000):
            path.write_bytes(text)
            with self.subTest(text=text[:50]), self.assertRaises(ValueError):
                load_binding_plan(path)
        path.write_text(json.dumps(binding_plan()), encoding="utf-8-sig")
        self.assertEqual(binding_plan(), load_binding_plan(path))


class BindingReportTests(unittest.TestCase):
    def inspect(self, rows):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"runtime.jsonl"
            path.write_text("".join(json.dumps(row)+"\n" for row in rows), encoding="utf-8")
            return inspect_report(path)

    def test_mapping_and_observation_mode_roundtrip(self):
        result = self.inspect(recorded_rows(bound=True))
        self.assertEqual("bound", result["bindings"]["status"])
        self.assertGreater(result["binding_wait_ticks"], 0)
        self.assertFalse(result["mission_observe_only"])
        result = self.inspect(recorded_rows())
        self.assertTrue(result["mission_observe_only"])
        self.assertIsNone(result["bindings"])

    def test_waiting_report_does_not_claim_mapping_completion(self):
        result = self.inspect(recorded_rows(bound=True, missing=True))
        self.assertEqual("waiting", result["bindings"]["status"])
        self.assertEqual({}, result["bindings"]["mappings"])
        self.assertEqual(5, result["binding_wait_ticks"])

    def test_mixed_modes_or_unconfigured_bindings_are_rejected(self):
        for change in ({"bindings": {}}, {"mission_observe_only": False}):
            rows = recorded_rows()
            rows[-2].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_observation_only_and_waiting_modes_reject_nonzero_motion(self):
        for bound in (False, True):
            rows = recorded_rows(bound=bound, missing=True)
            for field, value in (("forward_velocity_mm_s", 1), ("velocity_world_mm_s", [1, 0]),
                                 ("angular_velocity_rad_s", 1)):
                changed = copy.deepcopy(rows)
                changed[-2]["actuator"]["robots"][0][field] = value
                with self.subTest(bound=bound, field=field), self.assertRaises(ValueError):
                    self.inspect(changed)

    def test_binding_false_claims_and_future_sequence_are_rejected(self):
        for change in ({"physical_identity_verified": True}, {"device_io": 0}, {"motion_permitted": True},
                       {"applied_sequence": 1000}, {"applied_sequence": True}, {"session_id": "other"},
                       {"mappings": {}}, {"status": "waiting"}, {"reason": "mismatch"},
                       {"reviewed_by": "other"}, {"candidates": [None]}):
            rows = recorded_rows(bound=True)
            rows[-2]["bindings"].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_mapping_cannot_change_after_application(self):
        for change in ({"mappings": {"D1": "different-track"}}, {"applied_sequence": 4}):
            rows = recorded_rows(bound=True)
            rows[-2]["bindings"].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_internally_consistent_mapping_cannot_contradict_world_identities(self):
        rows = recorded_rows(bound=True)
        for row in rows[1:]:
            value = row["bindings"]
            if value["applied_sequence"] is not None:
                value["mappings"] = {"D1": "invented-consistent-track"}
                for candidate in value["candidates"]:
                    candidate["candidate_track_ids"] = ["invented-consistent-track"]
        with self.assertRaisesRegex(ValueError, "world"):
            self.inspect(rows)

    def test_bound_world_provenance_and_current_catalog_are_checked_on_every_tick(self):
        for change in ({"source_name": "foreign"}, {"configuration_id": "foreign"},
                       {"is_replay": True}, {"generated_at_s": 0.}):
            rows = recorded_rows(bound=True)
            rows[-2]["mission"]["world"].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "world"):
                self.inspect(rows)
        rows = recorded_rows(bound=True)
        pieces = rows[-2]["mission"]["world"]["pieces"]
        rows[-2]["mission"]["world"]["pieces"] = [*pieces, copy.deepcopy(pieces[0])]
        with self.assertRaisesRegex(ValueError, "world"):
            self.inspect(rows)


if __name__ == "__main__":
    unittest.main()
