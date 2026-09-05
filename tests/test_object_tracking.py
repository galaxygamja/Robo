from __future__ import annotations

import copy
import io
import json
import math
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from robo_control.vision.object_tracking import ObjectTracker


def candidate(x=100.0, y=100.0, *, color="red", kind="cylinder"):
    return {"center_mm": [x, y], "color": color, "kind": kind,
            "identity": None, "classification": "color_geometry_candidate"}


def record(sequence=1, stamp=10.0, objects=None, **updates):
    data = {"status": "detected", "sequence": sequence, "captured_at_s": stamp,
            "received_at_s": stamp, "source_name": "camera-session-1", "is_replay": False,
            "objects": [candidate()] if objects is None else objects}
    data.update(updates)
    return data


class ObjectTrackerTests(unittest.TestCase):
    def tracker(self, **kwargs):
        return ObjectTracker(["H1", "H2", "B1", "B2"], **kwargs)

    def confirmed(self, tracker=None):
        tracker = tracker or self.tracker()
        for index in range(3):
            result = tracker.update(record(index + 1, 10 + index * 0.1), 10 + index * 0.1)
        return tracker, result["object_tracks"][0]["object_id"]

    def test_three_consecutive_detections_confirm_one_stable_id_without_mutation(self):
        tracker = self.tracker()
        raw = record()
        original = copy.deepcopy(raw)
        first = tracker.update(raw, 10)["object_tracks"][0]
        self.assertEqual(original, raw)
        self.assertFalse(first["confirmed"])
        self.assertEqual("tentative", first["state"])
        second = tracker.update(record(2, 10.1, [candidate(102)]), 10.1)["object_tracks"][0]
        self.assertEqual(first["object_id"], second["object_id"])
        self.assertFalse(second["valid_for_pick"])
        third = tracker.update(record(3, 10.2, [candidate(104)]), 10.2)["object_tracks"][0]
        self.assertEqual(first["object_id"], third["object_id"])
        self.assertTrue(third["confirmed"])
        self.assertTrue(third["valid_for_pick"])
        self.assertFalse(third["physical_identity_verified"])

    def test_confirmation_threshold_is_configurable(self):
        tracker = self.tracker(confirm_frames=1)
        track = tracker.update(record(), 10)["object_tracks"][0]
        self.assertTrue(track["confirmed"])

    def test_reordered_candidates_do_not_exchange_ids(self):
        tracker = self.tracker(confirm_frames=1)
        first = tracker.update(record(objects=[candidate(250), candidate(100)]), 10)["object_tracks"]
        ids = {t["center_mm"][0]: t["object_id"] for t in first}
        second = tracker.update(record(2, 10.1, [candidate(101), candidate(249)]), 10.1)["object_tracks"]
        self.assertEqual(ids[100], second[0]["object_id"])
        self.assertEqual([101, 100], second[0]["center_mm"])
        self.assertEqual(ids[250], second[1]["object_id"])
        self.assertEqual([249, 100], second[1]["center_mm"])

    def test_kind_and_color_must_both_match(self):
        for different in (candidate(color="green"), candidate(kind="cube")):
            with self.subTest(different=different):
                tracker = self.tracker(confirm_frames=1)
                tracker.update(record(), 10)
                result = tracker.update(record(2, 10.1, [different]), 10.1)
                self.assertEqual(2, len(result["object_tracks"]))
                self.assertEqual("missing", result["object_tracks"][0]["state"])

    def test_transient_blob_and_interrupted_streak_never_confirm_early(self):
        tracker = self.tracker()
        tracker.update(record(), 10)
        tracker.update(record(2, 10.1, []), 10.1)
        for seq, stamp in ((3, 10.2), (4, 10.3)):
            track = tracker.update(record(seq, stamp), stamp)["object_tracks"][0]
            self.assertFalse(track["confirmed"])
        self.assertTrue(tracker.update(record(5, 10.4), 10.4)["object_tracks"][0]["confirmed"])

    def test_missing_before_timeout_reacquires_same_id_but_loss_expires_identity(self):
        tracker, rid = self.confirmed()
        missing = tracker.update(record(4, 10.3, []), 10.3)["object_tracks"][0]
        self.assertEqual("missing", missing["state"])
        self.assertFalse(missing["valid_for_pick"])
        again = tracker.update(record(5, 10.4, [candidate(103)]), 10.4)["object_tracks"][0]
        self.assertEqual(rid, again["object_id"])
        self.assertTrue(again["valid_for_pick"])
        expired = tracker.snapshot(11)["object_tracks"][0]
        self.assertEqual("lost", expired["state"])
        reappeared = tracker.update(record(6, 11.1, [candidate(103)]), 11.1)["object_tracks"]
        self.assertEqual("lost", reappeared[0]["state"])
        self.assertNotEqual(rid, reappeared[1]["object_id"])
        self.assertFalse(reappeared[1]["confirmed"])

    def test_equal_same_color_crossing_latches_uncertainty_after_separation(self):
        tracker = self.tracker(confirm_frames=1, max_distance_mm=80)
        initial = tracker.update(record(objects=[candidate(100), candidate(200)]), 10)
        ids = [t["object_id"] for t in initial["object_tracks"]]
        crossing = tracker.update(record(2, 10.1, [candidate(150), candidate(150)]), 10.1)
        self.assertEqual(ids, crossing["ambiguous_object_ids"])
        self.assertEqual(2, len(crossing["object_tracks"]))
        self.assertTrue(all(t["state"] == "ambiguous" and not t["valid_for_pick"]
                            for t in crossing["object_tracks"]))
        separated = tracker.update(record(3, 10.2, [candidate(105), candidate(195)]), 10.2)
        self.assertEqual(ids, separated["ambiguous_object_ids"])
        self.assertTrue(all(t["identity_uncertain"] for t in separated["object_tracks"]))
        fresh_ids = tracker.update(record(4, 10.7, [candidate(105), candidate(195)]), 10.7)["object_tracks"]
        self.assertTrue(all(t["state"] == "lost" for t in fresh_ids[:2]))
        self.assertTrue(all(t["object_id"] not in ids for t in fresh_ids[2:]))

    def test_one_detection_equally_near_two_tracks_does_not_merge_their_ids(self):
        tracker = self.tracker(confirm_frames=1, max_distance_mm=80)
        tracker.update(record(objects=[candidate(100), candidate(200)]), 10)
        result = tracker.update(record(2, 10.1, [candidate(150)]), 10.1)
        self.assertEqual(2, len(result["ambiguous_object_ids"]))
        self.assertTrue(all(t["center_mm"][0] in {100, 200} for t in result["object_tracks"]))

    def test_near_duplicate_new_candidates_do_not_create_two_ids(self):
        result = self.tracker().update(record(objects=[candidate(100), candidate(102)]), 10)
        self.assertEqual([], result["object_tracks"])
        self.assertEqual(2, len(result["object_tracking_rejections"]))

    def test_overlapping_colors_or_kinds_never_confirm_as_two_objects(self):
        for conflict in (candidate(color="green"), candidate(kind="cube")):
            with self.subTest(conflict=conflict):
                tracker = self.tracker()
                for seq in (1, 2, 3):
                    stamp = 10 + seq * .1
                    result = tracker.update(record(seq, stamp, [candidate(), conflict]), stamp)
                    self.assertEqual([], result["object_tracks"])
                    self.assertEqual(2, len(result["object_tracking_rejections"]))
                    self.assertTrue(all(r["reason"] == "ambiguous_association"
                                        for r in result["object_tracking_rejections"]))

    def test_cross_color_overlap_latches_existing_tracks_until_identity_expiry(self):
        tracker = self.tracker(confirm_frames=1, max_distance_mm=80)
        first = tracker.update(record(objects=[candidate(100), candidate(200, color="green")]), 10)
        ids = sorted(t["object_id"] for t in first["object_tracks"])
        overlap = tracker.update(record(2, 10.1, [candidate(150), candidate(150, color="green")]), 10.1)
        self.assertEqual(ids, overlap["ambiguous_object_ids"])
        self.assertTrue(all(t["identity_uncertain"] and not t["valid_for_pick"]
                            for t in overlap["object_tracks"]))
        separated = tracker.update(record(3, 10.2, [candidate(105), candidate(195, color="green")]), 10.2)
        self.assertEqual(ids, separated["ambiguous_object_ids"])
        self.assertTrue(all(not t["valid_for_pick"] for t in separated["object_tracks"]))

    def test_fast_video_decode_uses_media_gap_for_expiry_not_velocity(self):
        tracker = self.tracker()
        for seq, host, media, x in ((1, 10, 0, 100), (2, 10.001, .1, 110), (3, 10.002, .2, 120)):
            result = tracker.update(record(seq, host, [candidate(x)], is_replay=True, media_time_s=media), host)
        self.assertTrue(result["object_tracks"][0]["confirmed"])
        self.assertEqual("host_freshness_and_replay_media_expiry", result["object_tracking_time_basis"])
        result = tracker.update(record(4, 10.003, [candidate(125)], is_replay=True, media_time_s=1.0), 10.003)
        self.assertEqual("lost", result["object_tracks"][0]["state"])
        self.assertFalse(result["object_tracks"][1]["confirmed"])

    def test_equal_coarse_host_ticks_with_increasing_sequence_and_media_are_valid(self):
        tracker = self.tracker()
        for sequence, media in ((1, 0.0), (2, 0.1), (3, 0.2)):
            result = tracker.update(record(sequence, 10.0, is_replay=True, media_time_s=media), 10.0)
        self.assertIsNone(result["object_tracking_frame_reason"])
        self.assertTrue(result["object_tracks"][0]["confirmed"])

    def test_media_clock_cannot_bypass_host_freshness_and_reversed_media_is_rejected(self):
        tracker = self.tracker()
        tracker.update(record(is_replay=True, media_time_s=1.0), 10)
        result = tracker.update(record(2, 10.1, is_replay=True, media_time_s=.9), 10.1)
        self.assertEqual("invalid_or_out_of_order_media_time", result["object_tracking_frame_reason"])
        result = tracker.update(record(3, 10.2, is_replay=True, media_time_s=1.1), 11)
        self.assertEqual("stale_or_invalid_timestamp", result["object_tracking_frame_reason"])
        self.assertFalse(result["object_tracks"][0]["valid_for_pick"])

    def test_still_image_declares_host_only_time_and_never_confirms_single_default_frame(self):
        result = self.tracker().update(record(is_replay=True, media_time_s=None), 10)
        self.assertEqual("host_only_replay_media_unavailable", result["object_tracking_time_basis"])
        self.assertFalse(result["object_tracks"][0]["confirmed"])

    def test_explicit_grip_ownership_is_never_inferred_from_image_or_loss(self):
        tracker, rid = self.confirmed()
        self.assertIsNone(tracker.snapshot(10.2)["object_tracks"][0]["owner_robot_id"])
        tracker.update(record(4, 10.3), 10.3)
        held = tracker.mark_gripped(rid, "H2", 10.3, evidence="test gripper acknowledgement")["object_tracks"][0]
        self.assertEqual("H2", held["owner_robot_id"])
        self.assertEqual("gripped", held["lifecycle"])
        self.assertFalse(held["valid_for_pick"])
        lost = tracker.snapshot(11)["object_tracks"][0]
        self.assertEqual("lost", lost["state"])
        self.assertEqual("H2", lost["owner_robot_id"])
        result = tracker.update(record(5, 11.1), 11.1)
        self.assertEqual(1, len(result["object_tracks"]))
        self.assertEqual("H2", result["object_tracks"][0]["owner_robot_id"])
        with self.assertRaises(ValueError):
            tracker.mark_released(rid, "B1", 11.1, center_mm=[300, 100], evidence="wrong owner")

    def test_release_hint_does_not_count_as_vision_and_requires_three_new_hits(self):
        tracker, rid = self.confirmed()
        tracker.mark_gripped(rid, "B1", 10.2, evidence="simulation-only grip")
        released = tracker.mark_released(rid, "B1", 10.3, center_mm=[300, 100], evidence="simulation-only release")["object_tracks"][0]
        self.assertEqual("released_pending", released["state"])
        self.assertEqual("explicit_release_hint", released["position_evidence"])
        self.assertIsNone(released["observed_at_s"])
        self.assertFalse(released["confirmed"])
        for seq, stamp in ((4, 10.4), (5, 10.5), (6, 10.6)):
            result = tracker.update(record(seq, stamp, [candidate(300)]), stamp)
            current = result["object_tracks"][0]
            self.assertEqual(rid, current["object_id"])
            self.assertEqual(seq == 6, current["confirmed"])
        self.assertTrue(current["valid_for_pick"])
        self.assertEqual("released", current["lifecycle"])
        self.assertEqual("released", current["last_lifecycle_event"]["event"])

    def test_pre_release_frame_cannot_confirm_post_release_position(self):
        tracker, rid = self.confirmed()
        tracker.mark_gripped(rid, "B1", 10.2, evidence="fixture")
        tracker.mark_released(rid, "B1", 10.3, center_mm=[300, 100], evidence="fixture")
        result = tracker.update(record(4, 10.25, [candidate(300)]), 10.3)
        self.assertEqual("frame_precedes_lifecycle_event", result["object_tracking_frame_reason"])
        self.assertFalse(result["object_tracks"][0]["confirmed"])

    def test_replay_release_hint_does_not_reuse_pre_release_media_age(self):
        tracker = self.tracker(confirm_frames=1)
        rid = tracker.update(record(is_replay=True, media_time_s=0.0), 10)["object_tracks"][0]["object_id"]
        tracker.mark_gripped(rid, "H1", 10, evidence="fixture")
        tracker.mark_released(rid, "H1", 10.1, center_mm=[300, 100], evidence="fixture")
        result = tracker.update(record(2, 10.1, [candidate(300)], is_replay=True, media_time_s=1.0), 10.1)
        self.assertEqual(1, len(result["object_tracks"]))
        self.assertEqual(rid, result["object_tracks"][0]["object_id"])

    def test_grip_rejects_tentative_lost_owned_unknown_or_unacknowledged_objects(self):
        tracker = self.tracker()
        rid = tracker.update(record(), 10)["object_tracks"][0]["object_id"]
        with self.assertRaises(ValueError):
            tracker.mark_gripped(rid, "H1", 10, evidence="tentative")
        tracker, rid = self.confirmed()
        for robot, evidence in (("NO_SUCH_ROBOT", "fixture"), ("H1", "")):
            with self.assertRaises(ValueError):
                tracker.mark_gripped(rid, robot, 10.2, evidence=evidence)
        tracker.mark_gripped(rid, "H1", 10.2, evidence="fixture")
        with self.assertRaises(ValueError):
            tracker.mark_gripped(rid, "B1", 10.2, evidence="double claim")
        with self.assertRaises(ValueError):
            tracker.mark_released(rid, "H1", 10.3, center_mm=[math.nan, 0], evidence="fixture")
        self.assertEqual("H1", tracker.snapshot(10.3)["object_tracks"][0]["owner_robot_id"])

    def test_bad_candidate_bodies_fail_closed_without_moving_existing_tracks(self):
        for bad in (None, {}, "bad", [None], [{"center_mm": [1, 2], "color": [], "kind": "cube"}],
                    [candidate(math.nan)], [candidate(kind=[])], [candidate(color="")]):
            with self.subTest(bad=bad):
                tracker, _ = self.confirmed()
                data = record(4, 10.3)
                data["objects"] = bad
                result = tracker.update(data, 10.3)
                self.assertIsNotNone(result["object_tracking_frame_reason"])
                self.assertEqual([100, 100], result["object_tracks"][0]["center_mm"])
                self.assertFalse(result["object_tracks"][0]["valid_for_pick"])

    def test_invalid_header_source_order_time_and_mode_are_rejected(self):
        for changes in ({"source_name": "another"}, {"sequence": 1}, {"captured_at_s": 9.9},
                        {"captured_at_s": math.nan}, {"received_at_s": 10.2},
                        {"is_replay": True}, {"is_replay": "yes"}):
            with self.subTest(changes=changes):
                tracker = self.tracker(confirm_frames=1)
                tracker.update(record(), 10)
                result = tracker.update(record(2, 10.1, **changes), 10.1) if "sequence" not in changes else tracker.update({**record(2, 10.1), **changes}, 10.1)
                self.assertIsNotNone(result["object_tracking_frame_reason"])
                self.assertFalse(result["object_tracks"][0]["valid_for_pick"])

    def test_rejected_frame_consumes_valid_sequence_and_capture_watermarks(self):
        tracker = self.tracker(confirm_frames=1)
        tracker.update(record(), 10)
        rejected = tracker.update(record(3, 10.2, status="rejected_frame", reason="resolution_mismatch"), 10.2)
        self.assertEqual("resolution_mismatch", rejected["object_tracking_frame_reason"])
        late = tracker.update(record(2, 10.1, [candidate(120)]), 10.21)
        self.assertEqual("out_of_order_sequence", late["object_tracking_frame_reason"])
        self.assertFalse(late["object_tracks"][0]["valid_for_pick"])
        self.assertEqual([100, 100], late["object_tracks"][0]["center_mm"])
        backwards = tracker.update(record(4, 10.15, [candidate(120)]), 10.22)
        self.assertEqual("stale_or_invalid_timestamp", backwards["object_tracking_frame_reason"])
        newer = tracker.update(record(5, 10.3, [candidate(110)]), 10.3)
        self.assertTrue(newer["object_tracks"][0]["valid_for_pick"])

    def test_rejected_video_frame_also_consumes_media_order(self):
        tracker = self.tracker(confirm_frames=1)
        tracker.update(record(is_replay=True, media_time_s=0.0), 10)
        tracker.update(record(3, 10.2, is_replay=True, media_time_s=.2,
                              status="rejected_frame", reason="invalid_image"), 10.2)
        result = tracker.update(record(4, 10.3, is_replay=True, media_time_s=.1), 10.3)
        self.assertEqual("invalid_or_out_of_order_media_time", result["object_tracking_frame_reason"])
        self.assertFalse(result["object_tracks"][0]["valid_for_pick"])

    def test_source_closed_is_terminal_even_if_same_source_sends_new_frames(self):
        for via_method in (False, True):
            with self.subTest(via_method=via_method):
                tracker = self.tracker(confirm_frames=1)
                initial = tracker.update(record(), 10)
                rid = initial["object_tracks"][0]["object_id"]
                closed = (tracker.close(10.1) if via_method else tracker.update(
                    record(2, 10.1, status="source_closed"), 10.1))
                self.assertTrue(closed["object_tracking_session_closed"])
                resumed = tracker.update(record(3, 10.2, [candidate(110)]), 10.2)
                self.assertEqual("session_closed", resumed["object_tracking_frame_reason"])
                self.assertEqual(rid, resumed["object_tracks"][0]["object_id"])
                self.assertEqual([100, 100], resumed["object_tracks"][0]["center_mm"])
                self.assertFalse(resumed["object_tracks"][0]["valid_for_pick"])
                with self.assertRaises(ValueError):
                    tracker.mark_gripped(rid, "H1", 10.2, evidence="must not revive a closed session")
                self.assertEqual("session_closed", tracker.snapshot(10.3)["object_tracking_frame_reason"])
                # Reconnection is explicit: another tracker instance/session.
                fresh = self.tracker(confirm_frames=1).update(
                    record(1, 10.4, source_name="camera-session-2"), 10.4)
                self.assertFalse(fresh["object_tracking_session_closed"])
                self.assertTrue(fresh["object_tracks"][0]["valid_for_pick"])

    def test_invalid_clock_raises_without_silently_rewinding(self):
        tracker = self.tracker()
        tracker.update(record(), 10)
        for now in (9.9, True, math.nan, math.inf, "10"):
            with self.subTest(now=now), self.assertRaises(ValueError):
                tracker.snapshot(now)

    def test_retained_identity_memory_is_bounded_and_capacity_loss_reported(self):
        tracker = self.tracker(max_tracks=1)
        tracker.update(record(), 10)
        result = tracker.update(record(2, 11, [candidate(500)]), 11)
        self.assertEqual(1, len(result["object_tracks"]))
        self.assertEqual("track_capacity_reached", result["object_tracking_rejections"][0]["reason"])

    def test_invalid_configuration_is_rejected(self):
        for options in ({"confirm_frames": 0}, {"confirm_frames": True}, {"max_tracks": 0},
                        {"max_distance_mm": math.inf}, {"miss_timeout_s": -1},
                        {"max_frame_age_s": 0}, {"ambiguity_margin_mm": -1},
                        {"ambiguity_margin_mm": 40}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.tracker(**options)
        with self.assertRaises(ValueError):
            ObjectTracker(["H1", "H1"])


try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


@unittest.skipUnless(cv2 is not None and np is not None and hasattr(cv2, "aruco"), "install vision extra")
class ObjectTrackingCliTests(unittest.TestCase):
    def test_unpaced_video_retains_raw_candidates_and_appends_confirmed_tracks_and_terminal_record(self):
        from robo_control.vision.__main__ import main
        from robo_control.vision.calibration import FieldCalibration
        from robo_control.vision.tags import TagDetectorConfig

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration, tags, colors, video, report = (root / name for name in
                ("camera.json", "tags.json", "colors.json", "replay.avi", "objects.jsonl"))
            FieldCalibration((320, 240), ((0, 0), (319, 0), (319, 239), (0, 239)), (319, 239)).save(calibration)
            TagDetectorConfig(tag_to_robot={0: "H1"}).save(tags)
            colors.write_text(json.dumps({"schema_version": 1, "profiles": [{
                "color": "red", "kind": "cylinder", "hsv_ranges": [
                    [[0, 130, 100], [10, 255, 255]], [[170, 130, 100], [179, 255, 255]]],
                "min_area_mm2": 150, "max_area_mm2": 500, "min_circularity": .6}]}), encoding="utf-8")
            writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (320, 240))
            self.assertTrue(writer.isOpened())
            try:
                for x in (100, 110, 120):
                    image = np.full((240, 320, 3), 255, np.uint8)
                    cv2.circle(image, (x, 100), 10, (0, 0, 255), -1)
                    writer.write(image)
            finally:
                writer.release()
            out, err = io.StringIO(), io.StringIO()
            with redirect_stdout(out), redirect_stderr(err):
                status = main(["detect", "--video", str(video), "--calibration", str(calibration),
                    "--tags", str(tags), "--colors", str(colors), "--track", "--frames", "5",
                    "--max-age-ms", "5000", "--report", str(report)])
            self.assertEqual(0, status, err.getvalue())
            records = [json.loads(line) for line in report.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(4, len(records))
            self.assertEqual(1, len(records[0]["objects"]), records)
            self.assertEqual("source_closed", records[-1]["status"])
            self.assertTrue(records[-1]["object_tracking_session_closed"])
            self.assertTrue(records[-1]["tracking_session_closed"])
            self.assertEqual("missing", records[-1]["object_tracks"][0]["state"])
            identities = []
            for index, row in enumerate(records[:3]):
                self.assertEqual(1, len(row["objects"]))
                self.assertIsNone(row["objects"][0]["identity"])
                self.assertEqual(1, len(row["object_tracks"]))
                self.assertEqual(index == 2, row["object_tracks"][0]["confirmed"])
                identities.append(row["object_tracks"][0]["object_id"])
                self.assertFalse(row["device_io"])
            self.assertEqual(1, len(set(identities)))


if __name__ == "__main__":
    unittest.main()
