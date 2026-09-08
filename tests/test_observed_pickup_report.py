from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

from test_mission_bindings import PROFILE
from test_observed_pickup import ObservedMissionHarness, policy_data
from test_world_state import SOURCE, object_candidate

from robo_control.runtime_report import inspect_report


def report_rows(*, fault=False):
    h = ObservedMissionHarness(policy=policy_data(max_anchor_drift_mm=10.))
    rows = [{"schema_version": 1, "event": "session_started", "session_id": "mission-test",
        "configuration_id": None, "roles": h.session.controller.roles, "source_name": SOURCE,
        "is_replay": False, "input_mode": "live_camera", "output_mode": "dry_run_commands",
        "transport_mode": "mock", "drive_model": "differential_body", "mission_observe_only": False,
        "mission_plan": h.session.mission.plan, "binding_plan": h.session.bindings.plan,
        "observation_profile_id": PROFILE, "device_io": False, "motion_permitted": False}]
    for _ in range(5):
        rows.append(h.step())
    if fault:
        rows.append(h.step(objects=[object_candidate(420, 300, kind="disc", colour="white")]))
    else:
        rows.append(h.session.close(h.now+.01, "operator_stop"))
    return rows


class ObservedPickupReportTests(unittest.TestCase):
    def inspect(self, rows):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"runtime.jsonl"
            path.write_text("".join(json.dumps(row)+"\n" for row in rows), encoding="utf-8")
            return inspect_report(path)

    def test_normal_and_fault_reports_remain_honest_and_readable(self):
        for fault in (False, True):
            result = self.inspect(report_rows(fault=fault))
            self.assertFalse(result["physical_stop_verified"])
            self.assertFalse(result["mission"]["observed_pickup"]["physical_pickup_verified"])
            self.assertFalse(result["mission"]["observed_pickup_in_use"])
            if fault:
                self.assertEqual("pickup_anchor_drift_exceeded", result["closed_reason"])
                self.assertIsNone(result["mission"]["observed_pickup"]["robot_goal"])

    def test_future_frame_time_sequence_or_physical_claim_is_rejected(self):
        for change in ({"source_sequence": 1000}, {"source_sequence": True}, {"observed_at_s": 999.},
                       {"session_id": "other"}, {"physical_pickup_verified": True}, {"device_io": 0},
                       {"position_evidence": "scenario"}, {"replans": 10}, {"anchor_drift_mm": 12.},
                       {"mode": "scenario"}, {"task_id": "other"}, {"robot_goal": {}}, {"fault": []}):
            rows = report_rows()
            rows[-2]["mission"]["observed_pickup"].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_wrong_offset_derivation_and_unknown_orientation_are_rejected(self):
        for change in ({"x_mm": 1000}, {"y_mm": 301}, {"heading_rad": .5}, {"heading_rad": None}):
            rows = report_rows()
            rows[-2]["mission"]["observed_pickup"]["robot_goal"].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)

    def test_unconfigured_mode_and_current_use_after_close_are_rejected(self):
        rows = report_rows()
        rows[0]["mission_plan"]["schema_version"] = 1
        with self.assertRaises(ValueError):
            self.inspect(rows)
        rows = report_rows()
        rows[-1]["mission"]["observed_pickup_in_use"] = True
        with self.assertRaises(ValueError):
            self.inspect(rows)

    def test_anchor_identity_and_budget_cannot_reset(self):
        for change in ({"track_id": "new-track"}, {"piece_id": "D2"}, {"source_sequence": 2},
                       {"anchor_position_mm": [400., 310.], "anchor_drift_mm": 10.}):
            rows = report_rows()
            rows[-1]["mission"]["observed_pickup"].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.inspect(rows)
        rows = report_rows()
        rows[4]["mission"]["observed_pickup"]["replans"] = 1
        with self.assertRaises(ValueError):
            self.inspect(rows)

    def test_faulted_target_cannot_reuse_last_valid_goal(self):
        rows = report_rows(fault=True)
        goal = copy.deepcopy(rows[-2]["mission"]["observed_pickup"]["robot_goal"])
        rows[-1]["mission"]["observed_pickup"]["robot_goal"] = goal
        with self.assertRaises(ValueError):
            self.inspect(rows)

    def test_consistent_but_invented_target_position_is_not_current_world_evidence(self):
        rows = report_rows()
        for row in rows[3:]:
            target = row["mission"]["observed_pickup"]
            target["object_position_mm"] = [450., 300.]
            target["anchor_position_mm"] = [450., 300.]
            target["robot_goal"]["x_mm"] = 370.
        with self.assertRaisesRegex(ValueError, "world|evidence"):
            self.inspect(rows)

    def test_in_use_target_requires_exact_current_frame_and_pickable_identity(self):
        for change in ({"source_sequence": 4}, {"observed_at_s": 10.08}):
            rows = report_rows()
            rows[-2]["mission"]["observed_pickup"].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "world|evidence"):
                self.inspect(rows)
        for collection, change in (("pieces", {"track_id": "not-the-target"}),
                                    ("pieces", {"valid_for_pick": False}),
                                    ("objects", {"position_evidence": "scenario"}),
                                    ("objects", {"identity_uncertain": True})):
            rows = report_rows()
            rows[-2]["mission"]["world"][collection][0].update(change)
            with self.subTest(collection=collection, change=change), self.assertRaisesRegex(ValueError, "world|evidence"):
                self.inspect(rows)

    def test_current_use_cannot_survive_unready_world_or_sensor_phase(self):
        for change in ({"ready": False}, {"source_name": "foreign"}, {"configuration_id": "foreign"},
                       {"generated_at_s": 0.}, {"physical_identity_verified": True}):
            rows = report_rows()
            rows[-2]["mission"]["world"].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "world|evidence"):
                self.inspect(rows)
        rows = report_rows()
        rows[-2]["mission"]["phase"] = "pickup_close"
        with self.assertRaises(ValueError):
            self.inspect(rows)

    def test_previous_tasks_target_cannot_be_current_during_next_task_startup(self):
        rows = report_rows()
        plan = rows[0]["mission_plan"]
        entry = copy.deepcopy(plan["tasks"][0])
        entry["task_id"] = "move-D2"
        plan["tasks"].append(entry)
        rows[-2]["mission"].update(task_index=1, active_task_id=None, phase=None)
        with self.assertRaisesRegex(ValueError, "current|task"):
            self.inspect(rows)


if __name__ == "__main__":
    unittest.main()
