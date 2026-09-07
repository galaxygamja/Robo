from __future__ import annotations

import copy
import math
import unittest
from dataclasses import replace

from robo_control.control_loop import ClosedLoopController, ControlLimits
from robo_control.runtime_session import LiveControlSession


def detection(seq=1, stamp=10.0, ids=("H1",), **changes):
    result = {
        "status": "detected", "sequence": seq, "captured_at_s": stamp,
        "received_at_s": stamp, "source_name": "webcam:test", "is_replay": False,
        "coordinate_system": "bottom_left_x_right_y_up_mm", "field_size_mm": [1143.0, 1181.0],
        "registered_robot_ids": list(ids), "device_io": False,
        "observation_complete": True, "unknown_tag_ids": [], "duplicate_tag_ids": [],
        "hardware_verified": False, "tag_size_mm": None, "objects": [],
        "robots": [{"robot_id": rid, "robot_center_mm": [300.0 + i * 400, 500.0],
                    "heading_rad": 0.0} for i, rid in enumerate(ids)],
    }
    result.update(changes)
    return result


def make_session(ids=("H1",), **changes):
    args = {"roles": {rid: "beaver" for rid in ids}, "field_size_mm": (1143.0, 1181.0),
            "source_name": "webcam:test", "session_id": "runtime-test",
            "goals": {rid: {"x_mm": 400.0 + i * 400, "y_mm": 500.0} for i, rid in enumerate(ids)},
            "radii_mm": {rid: 40.0 for rid in ids}}
    args.update(changes)
    return LiveControlSession(**args)


def zero_output(event):
    return all(r["velocity_world_mm_s"] == [0.0, 0.0]
               and r["angular_velocity_rad_s"] == 0.0
               and r["wheel_velocity_rad_s"] == [0.0] * 4 for r in event["actuator"]["robots"])


class RuntimeSessionTests(unittest.TestCase):
    def warmed(self, **kwargs):
        session = make_session(**kwargs)
        for seq in range(1, 4):
            now = 10.0 + (seq - 1) * 0.05
            event = session.advance(detection(seq, now), now)
        self.assertFalse(zero_output(event))
        return session

    def test_requires_three_fresh_measurements_and_has_no_simulated_pose_integrator(self):
        session = make_session()
        for seq in range(1, 5):
            now = 10.0 + (seq - 1) * .05
            event = session.advance(detection(seq, now), now)
            self.assertEqual(zero_output(event), seq < 3)
            self.assertEqual([300., 500.], event["observation"]["tracks"][0]["robot_center_mm"])
            self.assertFalse(event["device_io"])
            self.assertFalse(event["motion_permitted"])
            self.assertFalse(event["actuator"]["motion_permitted"])

    def test_empty_polls_never_reissue_command_and_capture_deadline_stops_at_200_ms(self):
        session = self.warmed()
        issued_sequence = session.bank.sequence
        deadline = session.last_capture_s + .2
        for now in (10.15, 10.20, 10.25, deadline - 1e-6):
            event = session.advance(None, now)
            self.assertIsNone(event["command"])
            self.assertEqual(issued_sequence, session.bank.sequence)
            self.assertFalse(zero_output(event))
        event = session.advance(None, deadline)
        self.assertTrue(zero_output(event))
        self.assertEqual("observation_watchdog", event["status"])
        self.assertEqual(0, event["fresh_streak"])

    def test_invalid_or_missing_robot_immediately_stops_every_registered_robot(self):
        mutations = [lambda r: r.update(robots=[]),
                     lambda r: r["robots"].append(copy.deepcopy(r["robots"][0])),
                     lambda r: r.update(observation_complete=False),
                     lambda r: r.update(unknown_tag_ids=[99]),
                     lambda r: r["robots"][0].update(heading_rad=float("nan")),
                     lambda r: r["robots"][0].update(robot_center_mm=[-1, 500]),
                     lambda r: r["robots"][0].update(robot_center_mm=[300, float("inf")])]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                session = self.warmed()
                row = detection(4, 10.15)
                mutate(row)
                event = session.advance(row, 10.15)
                self.assertTrue(zero_output(event))
                self.assertEqual(0, event["fresh_streak"])

    def test_reacquisition_after_a_gap_requires_three_new_frames(self):
        session = self.warmed()
        session.advance(detection(4, 10.15, robots=[]), 10.15)
        for seq in (5, 6, 7):
            stamp = 10.2 + (seq - 5) * .05
            event = session.advance(detection(seq, stamp), stamp)
            self.assertEqual(seq < 7, zero_output(event))

    def test_expired_gap_cannot_be_hidden_by_fresh_frame_at_same_tick(self):
        session = self.warmed()
        session.advance(None, 10.19)
        session.advance(None, 10.28)
        event = session.advance(detection(4, 10.31), 10.31)
        self.assertEqual(1, event["fresh_streak"])
        self.assertTrue(zero_output(event))

    def test_supervisor_overrun_latches_even_if_new_frames_arrive(self):
        session = self.warmed()
        stopped = session.advance(detection(4, 10.21), 10.21)
        self.assertEqual("supervisor_deadline_missed", stopped["closed_reason"])
        self.assertTrue(zero_output(stopped))
        resumed = session.advance(detection(5, 10.22), 10.22)
        self.assertTrue(zero_output(resumed))
        self.assertEqual("closed", resumed["status"])

    def test_late_delivery_future_time_and_replayed_sequence_stop(self):
        rows = [detection(4, 9.9), detection(4, 10.3), detection(3, 10.1)]
        for row in rows:
            with self.subTest(row=row):
                event = self.warmed().advance(row, 10.15)
                self.assertTrue(zero_output(event))
                self.assertEqual(0, event["fresh_streak"])

    def test_provenance_and_coordinate_units_are_checked_before_tracking(self):
        changes = [{"source_name": "webcam:other"}, {"is_replay": True},
                   {"is_replay": 0}, {"device_io": True}, {"device_io": 0},
                   {"coordinate_system": "meters"}, {"field_size_mm": [1.143, 1.181]},
                   {"registered_robot_ids": ["H1", "H1"]}, {"registered_robot_ids": [["H1"]]},
                   {"robots": None}, {"robots": [None]}]
        for change in changes:
            with self.subTest(change=change):
                event = self.warmed().advance(detection(4, 10.15, **change), 10.15)
                self.assertTrue(zero_output(event))

    def test_source_close_and_clock_fault_cannot_reopen(self):
        session = self.warmed()
        closed = session.advance({"status": "source_closed"}, 10.15)
        self.assertTrue(zero_output(closed))
        self.assertEqual("source_closed", closed["closed_reason"])
        self.assertTrue(zero_output(session.advance(detection(4, 10.2), 10.2)))
        for now in (float("nan"), float("inf"), True, 9.):
            s = self.warmed()
            with self.subTest(now=now), self.assertRaises(ValueError):
                s.advance(None, now)
            self.assertEqual("invalid_runtime_clock", s.closed_reason)
            self.assertTrue(all(r["velocity_world_mm_s"] == [0., 0.] for r in s.bank.snapshot()["robots"]))

    def test_diagnostic_mutation_cannot_change_private_pose_or_command_history(self):
        session = make_session()
        raw = detection()
        event = session.advance(raw, 10.)
        raw["robots"][0]["robot_center_mm"][0] = 999.
        event["observation"]["tracks"][0]["robot_center_mm"][0] = 888.
        event["command"]["robots"][0]["velocity_world_mm_s"][0] = 777.
        self.assertEqual([300., 500.], session.tracker.last["H1"]["robot_center_mm"])
        self.assertEqual([0., 0.], session.command["robots"][0]["velocity_world_mm_s"])

    def test_absent_goals_use_observed_positions_without_requesting_motion(self):
        session = make_session(goals={})
        for seq in range(1, 4):
            event = session.advance(detection(seq, 10. + seq * .02), 10. + seq * .02)
        self.assertTrue(zero_output(event))
        self.assertEqual("at_goal", event["status"])

    def test_configuration_does_not_relax_live_deadlines_or_skip_confirmation(self):
        for change in ({"recovery_frames": 1}, {"recovery_frames": True},
                       {"max_tick_gap_s": .2}, {"max_tick_gap_s": float("nan")},
                       {"is_replay": 1}, {"limits": replace(ControlLimits(), max_pose_age_s=.5)}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                make_session(**change)

    def test_replay_media_clock_preserved_but_output_is_only_dry_run(self):
        session = make_session(is_replay=True)
        for seq in (1, 2, 3):
            stamp = 10. + seq * .05
            row = detection(seq, stamp, is_replay=True, media_time_s=seq * .1)
            row["robots"][0]["robot_center_mm"][0] += seq * 2
            event = session.advance(row, stamp)
        self.assertAlmostEqual(20., event["observation"]["tracks"][0]["velocity_mm_s"][0])
        self.assertFalse(event["hardware_ready"])

    def test_camera_configuration_identity_is_pinned_for_whole_session(self):
        session = make_session(configuration_id="calibration-A")
        for seq in (1, 2, 3):
            now = 10. + seq * .02
            event = session.advance(detection(seq, now, configuration_id="calibration-A"), now)
        self.assertFalse(zero_output(event))
        changed = session.advance(detection(4, 10.08, configuration_id="calibration-B"), 10.08)
        self.assertEqual("configuration_changed", changed["status"])
        self.assertTrue(zero_output(changed))

    def test_object_identity_expires_even_when_camera_sends_no_more_records(self):
        session = make_session(track_objects=True)
        objects = [{"color": "red", "kind": "cylinder", "center_mm": [650., 750.],
                    "area_mm2": 300., "circularity": .95}]
        for seq in (1, 2, 3):
            now = 10. + seq * .02
            event = session.advance(detection(seq, now, objects=objects), now)
            if seq < 3:
                session.advance(None, now + .01)
        self.assertTrue(event["object_tracking"]["object_tracks"][0]["confirmed"])
        for now in (10.15, 10.24, 10.33, 10.42, 10.51, 10.60):
            event = session.advance(None, now)
        self.assertEqual("lost", event["object_tracking"]["object_tracks"][0]["state"])
        self.assertTrue(zero_output(event))

    def test_rejected_camera_source_immediately_invalidates_object_pick_evidence(self):
        session = make_session(track_objects=True)
        objects = [{"color": "red", "kind": "cylinder", "center_mm": [650., 750.],
                    "area_mm2": 300., "circularity": .95}]
        for seq in (1, 2, 3):
            now = 10. + seq * .02
            event = session.advance(detection(seq, now, objects=objects), now)
        self.assertTrue(event["object_tracking"]["object_tracks"][0]["valid_for_pick"])
        objects[0]["center_mm"] = [100., 100.]
        event = session.advance(detection(4, 10.08, objects=objects, source_name="foreign-camera"), 10.08)
        track = event["object_tracking"]["object_tracks"][0]
        self.assertFalse(track["valid_for_pick"])
        self.assertEqual(0, track["confirmation_streak"])
        self.assertEqual([650., 750.], track["center_mm"])
        self.assertTrue(zero_output(event))


class FieldBoundaryTests(unittest.TestCase):
    def measured(self, x=300., y=500., velocity=(0., 0.)):
        return {"source_name": "camera", "sequence": 1, "captured_at_s": 1.,
                "observation_usable": True, "stop_required": False,
                "tracks": [{"robot_id": "H1", "robot_center_mm": [x, y], "heading_rad": 0.,
                    "observed_at_s": 1., "velocity_mm_s": list(velocity), "angular_velocity_rad_s": 0.,
                    "state": "observed", "valid_for_control": True}]}

    def test_all_four_walls_check_complete_body_and_braking_envelope(self):
        for x, y, velocity in ((60, 500, (-180, 0)), (1083, 500, (180, 0)),
                               (500, 60, (0, -180)), (500, 1121, (0, 180))):
            with self.subTest(x=x, y=y):
                controller = ClosedLoopController(roles={"H1": "hamster"}, radii_mm={"H1": 40},
                    field_size_mm=(1143, 1181), goals={})
                event = controller.tick(self.measured(x, y, velocity), 1.)
                self.assertEqual("predicted_collision", event["stop_reason"])
                self.assertEqual("field_boundary_envelope", event["conflicts"][0]["reason"])
                self.assertEqual([0., 0.], event["robots"][0]["velocity_world_mm_s"])

    def test_valid_stationary_pose_inside_field_remains_usable(self):
        controller = ClosedLoopController(roles={"H1": "hamster"},
            field_size_mm=(1143, 1181), goals={"H1": {"x_mm": 500, "y_mm": 500}})
        event = controller.tick(self.measured(), 1.)
        self.assertIsNone(event["stop_reason"])

    def test_invalid_field_and_outside_goal_rejected(self):
        for size in ((0, 1181), (1143, math.inf), (True, 1181), [1], "field"):
            with self.subTest(size=size), self.assertRaises(ValueError):
                ClosedLoopController(field_size_mm=size)
        for x, y in ((10, 500), (1130, 500), (500, 10), (500, 1170)):
            with self.subTest(x=x, y=y), self.assertRaises(ValueError):
                ClosedLoopController(roles={"H1": "hamster"}, field_size_mm=(1143, 1181),
                                     goals={"H1": {"x_mm": x, "y_mm": y}})


if __name__ == "__main__":
    unittest.main()
