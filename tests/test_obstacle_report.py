from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

from test_observed_obstacles import ObstacleMissionHarness
from test_world_state import SOURCE

from robo_control.runtime_report import inspect_report


def report_rows(*, lost=False):
    h = ObstacleMissionHarness()
    rows = [{"schema_version": 1, "event": "session_started", "session_id": "mission-test",
        "configuration_id": None, "roles": h.session.controller.roles, "source_name": SOURCE,
        "is_replay": False, "input_mode": "live_camera", "output_mode": "dry_run_commands",
        "transport_mode": "mock", "drive_model": "differential_body", "mission_observe_only": False,
        "mission_plan": h.session.mission.plan, "obstacle_plan": h.session.object_obstacles.plan,
        "device_io": False, "motion_permitted": False}]
    for _ in range(5):
        rows.append(h.step())
    if lost:
        rows.append(h.step(objects=[]))
    else:
        rows.append(h.session.close(h.now+.01, "operator_stop"))
    return rows


class ObstacleReportTests(unittest.TestCase):
    def inspect(self, rows):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"runtime.jsonl"
            path.write_text("".join(json.dumps(row)+"\n" for row in rows), encoding="utf-8")
            return inspect_report(path)

    def test_normal_and_lost_geometry_logs_do_not_claim_current_clear_space(self):
        for lost in (False, True):
            result = self.inspect(report_rows(lost=lost))
            self.assertFalse(result["object_obstacles"]["ready"])
            self.assertEqual(1, len(result["object_obstacles"]["objects"]))
            self.assertFalse(result["object_obstacles"]["unobserved_is_free"])
            self.assertFalse(result["object_obstacles"]["field_coverage_verified"])
            if lost:
                self.assertEqual("object_obstacle_lost", result["closed_reason"])
                self.assertEqual("fault", result["object_obstacles"]["status"])

    def test_wrong_mode_physical_claims_or_contact_exemptions_rejected(self):
        for change in ({"mode": "simulated_obstacles"}, {"session_id": "other"},
                       {"physical_clearance_verified": True}, {"field_coverage_verified": True},
                       {"device_io": 0}, {"unobserved_is_free": True}, {"contact_exemptions": ["D1"]}):
            rows = report_rows()
            rows[-2]["object_obstacles"].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_wrong_clock_geometry_or_policy_rejected(self):
        for change in ({"captured_at_s": 1000.}, {"captured_at_s": 1.}, {"source_sequence": 1000},
                       {"source_sequence": True}, {"max_age_s": 1.}, {"route_replans": 4},
                       {"position_margin_mm": 3.}, {"field_size_mm": [1, 2]}, {"at_s": 0.},
                       {"source_name": "other"}, {"configuration_id": "other"}, {"is_replay": True}):
            rows = report_rows()
            rows[-2]["object_obstacles"].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_unconfigured_evidence_and_removed_map_rejected(self):
        rows = report_rows()
        rows[0].pop("obstacle_plan")
        with self.assertRaises(ValueError):
            self.inspect(rows)
        rows = report_rows()
        rows[-2]["object_obstacles"] = None
        with self.assertRaises(ValueError):
            self.inspect(rows)

    def test_objects_cannot_disappear_be_resized_or_change_identity(self):
        rows = report_rows()
        rows[-1]["object_obstacles"]["objects"] = []
        with self.assertRaises(ValueError):
            self.inspect(rows)
        for change in ({"track_id": "other"}, {"kind": "cube"}, {"colour": "green"},
                       {"radius_mm": 100}, {"observed_at_s": 0}, {"source_sequence": 0},
                       {"position_mm": [-1, 20]}, {"confirmation_state": "lost"}):
            rows = report_rows()
            rows[-1]["object_obstacles"]["objects"][0].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_changed_geometry_without_revision_and_counter_rewind_rejected(self):
        rows = report_rows()
        rows[-1]["object_obstacles"]["objects"][0]["position_mm"] = [451., 700.]
        with self.assertRaises(ValueError):
            self.inspect(rows)
        for change in ({"revision": 0}, {"route_replans": -1}, {"activated": False}):
            rows = report_rows()
            rows[-1]["object_obstacles"].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_waiting_map_cannot_claim_movement(self):
        rows = report_rows()
        rows[1]["actuator"]["robots"][0].update(forward_velocity_mm_s=1, velocity_world_mm_s=[1, 0])
        with self.assertRaises(ValueError):
            self.inspect(rows)

    def test_duplicate_objects_and_unknown_fields_rejected(self):
        rows = report_rows()
        rows[-1]["object_obstacles"]["objects"].append(copy.deepcopy(rows[-1]["object_obstacles"]["objects"][0]))
        with self.assertRaises(ValueError):
            self.inspect(rows)

    def test_ready_geometry_must_match_current_world_even_with_a_new_revision(self):
        for change in ({"position_mm": [700., 700.]}, {"confirmation_state": "tentative"}):
            rows = report_rows()
            rows[-2]["object_obstacles"]["revision"] += 1
            rows[-2]["object_obstacles"]["objects"][0].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "world|evidence"):
                self.inspect(rows)

    def test_ready_map_cannot_omit_all_current_world_objects(self):
        rows = report_rows()
        for row in rows[1:]:
            row["object_obstacles"]["objects"] = []
        with self.assertRaisesRegex(ValueError, "world|evidence"):
            self.inspect(rows)

    def test_ready_map_cannot_use_an_older_but_unexpired_world_sequence(self):
        rows = report_rows()
        old = rows[-3]["object_obstacles"]
        current = rows[-2]["object_obstacles"]
        for key in ("source_sequence", "captured_at_s", "objects"):
            current[key] = copy.deepcopy(old[key])
        with self.assertRaisesRegex(ValueError, "world|evidence"):
            self.inspect(rows)

    def test_ready_world_cannot_change_provenance_or_identity_evidence(self):
        for change in ({"source_name": "foreign"}, {"configuration_id": "foreign"},
                       {"is_replay": True}, {"generated_at_s": 0.}):
            rows = report_rows()
            rows[-2]["mission"]["world"].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "world|evidence"):
                self.inspect(rows)
        for change in ({"identity_uncertain": True}, {"position_evidence": "assumed"}):
            rows = report_rows()
            rows[-2]["mission"]["world"]["objects"][0].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "world|evidence"):
                self.inspect(rows)
        rows = report_rows()
        rows[-1]["object_obstacles"]["ignore_lost_objects"] = True
        with self.assertRaises(ValueError):
            self.inspect(rows)


if __name__ == "__main__":
    unittest.main()
