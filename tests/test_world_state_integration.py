from __future__ import annotations

import json
import unittest

from test_world_state import ROLES, SESSION, SOURCE, TrackedFrames, make_adapter

from robo_control.adapters import CameraFrame
from robo_control.qualifier import default_scenario_path, load_scenario
from robo_control.runtime_session import LiveControlSession
from robo_control.world_state import ObservationWorldAdapter, PieceSpec

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


class RuntimeWorldIntegrationTests(unittest.TestCase):
    def test_runtime_observation_boundary_and_between_frame_poll(self):
        runtime = LiveControlSession(roles=ROLES, source_name=SOURCE,
            field_size_mm=(1143., 1181.), session_id=SESSION, configuration_id="config-A", track_objects=True)
        frames, adapter = TrackedFrames(), make_adapter()
        for seq in (1, 2, 3):
            now = 10. + seq*.05
            row = frames.frame(seq, now)
            # The runtime consumes raw detection, then uses its own trackers.
            for key in ("tracks", "object_tracks"):
                row.pop(key)
            event = runtime.advance(row, now)
            world = adapter.update(event["observation"], now, source_session_id=event["session_id"])
            self.assertEqual(seq == 3, world.ready)
            for robot in world.robots:
                raw = next(r for r in event["observation"]["robots"] if r["robot_id"] == robot.robot_id)
                self.assertEqual(tuple(raw["robot_center_mm"]), robot.position_mm)
        event = runtime.advance(None, 10.18)
        self.assertIsNone(event["observation"])
        self.assertTrue(adapter.update(event["observation"], 10.18, source_session_id=event["session_id"]).ready)
        for now in (10.23, 10.28, 10.33, 10.35):
            event = runtime.advance(None, now)
            world = adapter.poll(now)
        self.assertFalse(world.ready)
        self.assertTrue(all(r["velocity_world_mm_s"] == [0., 0.] for r in event["actuator"]["robots"]))
        self.assertFalse(world.as_dict()["device_io"])
        self.assertEqual("closed", adapter.close(10.36).status)

    def test_scenario_is_catalog_only_not_evidence_of_loaded_cubes_or_locations(self):
        scenario, pieces, _ = load_scenario(default_scenario_path())
        roles = {r["id"]: r["role"] for r in scenario["ground_robots"]}
        adapter = ObservationWorldAdapter(roles=roles, pieces=[PieceSpec.from_piece(p) for p in pieces],
            source_name=SOURCE, session_id=SESSION, field_size_mm=(1143., 1181.))
        world = adapter.poll(10.)
        self.assertEqual(len(pieces), len(world.pieces))
        for piece in world.pieces:
            self.assertIsNone(piece.position_mm)
            self.assertIsNone(piece.owner_robot_id)
            self.assertEqual("unknown", piece.lifecycle)
            self.assertFalse(piece.valid_for_pick)
        original = next(p for p in pieces if p.kind == "cube")
        self.assertIsNotNone(original.held_by)
        self.assertIsNone(world.piece(original.id).owner_robot_id)

    def test_json_roundtrip_inputs_produce_independent_json_snapshots(self):
        adapter, frames = make_adapter(), TrackedFrames()
        for seq in (1, 2, 3):
            now = 10. + seq * .05
            row = json.loads(json.dumps(frames.frame(seq, now), allow_nan=False))
            world = adapter.update(row, now, source_session_id=SESSION)
        serialized = json.loads(json.dumps(world.as_dict(), allow_nan=False))
        self.assertTrue(serialized["ready"])
        self.assertEqual([150., 500.], serialized["robots"][0]["position_mm"])
        self.assertIsNone(serialized["pieces"][0]["position_mm"])
        self.assertFalse(serialized["physical_identity_verified"])

    def test_runtime_known_bad_input_without_observation_is_explicitly_invalidated(self):
        runtime = LiveControlSession(roles=ROLES, source_name=SOURCE,
            field_size_mm=(1143., 1181.), session_id=SESSION, configuration_id="config-A", track_objects=True)
        adapter, frames = make_adapter(), TrackedFrames()
        for seq in (1, 2, 3):
            now = 10. + seq*.05
            event = runtime.advance(frames.frame(seq, now), now)
            world = adapter.update(event["observation"], now, source_session_id=event["session_id"])
        self.assertTrue(world.ready)
        row = frames.frame(4, 10.18)
        row["source_name"] = "other-camera"
        event = runtime.advance(row, 10.18)
        self.assertIsNone(event["observation"])
        self.assertEqual("unexpected_camera_source", event["status"])
        world = adapter.invalidate(10.18, reason=event["status"])
        self.assertFalse(world.ready)
        self.assertFalse(adapter.poll(10.19).ready)


@unittest.skipIf(cv2 is None, "OpenCV vision extra is not installed")
class MeasuredImageWorldIntegrationTests(unittest.TestCase):
    def test_real_image_detection_tracking_and_explicit_mission_binding(self):
        from robo_control.vision.calibration import FieldCalibration
        from robo_control.vision.colors import ColorDetector
        from robo_control.vision.detection import DetectionPipeline
        from robo_control.vision.object_tracking import ObjectTracker
        from robo_control.vision.tags import TagDetectorConfig
        from robo_control.vision.tracking import PoseTracker

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        image = np.full((480, 640, 3), 255, np.uint8)
        for tag, (x, y) in enumerate(((80, 70), (440, 70), (80, 330), (440, 330))):
            marker = cv2.aruco.generateImageMarker(dictionary, tag, 70)
            image[y:y+70, x:x+70] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        cv2.circle(image, (320, 240), 10, (0, 0, 255), -1)
        calibration = FieldCalibration((640, 480), ((0, 0), (639, 0), (639, 479), (0, 479)), (1143, 1181))
        tags = TagDetectorConfig(tag_to_robot={0: "H1", 1: "H2", 2: "B1", 3: "B2"})
        colors = ColorDetector(calibration, {"schema_version": 1, "profiles": [{"color": "red", "kind": "cylinder",
            "hsv_ranges": [[[0, 120, 100], [10, 255, 255]], [[170, 120, 100], [179, 255, 255]]],
            "min_area_mm2": 100., "max_area_mm2": 3000., "min_circularity": .5}]})
        clock = [10.]
        pipeline = DetectionPipeline(calibration, tags, colors=colors, clock=lambda: clock[0])
        poses, objects = PoseTracker(ROLES), ObjectTracker(ROLES)
        adapter = make_adapter(configuration_id=None)
        for seq in (1, 2, 3):
            clock[0] = 10. + seq * .05
            frame = CameraFrame(image, clock[0], seq, SOURCE, received_at_s=clock[0], is_replay=False)
            record = pipeline.process(frame).record
            self.assertEqual("detected", record["status"])
            record.update(poses.update(record, clock[0]))
            record.update(objects.update(record, clock[0]))
            world = adapter.update(record, clock[0], source_session_id=SESSION)
        self.assertTrue(world.ready)
        self.assertEqual(4, len(world.robots))
        self.assertEqual(1, len(world.objects))
        world = adapter.bind_piece("R1", world.objects[0].object_id, clock[0], evidence="known generated image fixture")
        piece = world.piece("R1")
        self.assertTrue(piece.position_valid)
        self.assertTrue(piece.valid_for_pick)
        np.testing.assert_allclose(piece.position_mm, (320*1143/639, (479-240)*1181/479), atol=2.)
        self.assertFalse(world.as_dict()["physical_identity_verified"])
        # Removing the marker creates a missing observation, not a completed task.
        image[220:260, 300:340] = 255
        clock[0] += .05
        record = pipeline.process(CameraFrame(image, clock[0], 4, SOURCE, received_at_s=clock[0])).record
        record.update(poses.update(record, clock[0]))
        record.update(objects.update(record, clock[0]))
        missing = adapter.update(record, clock[0], source_session_id=SESSION).piece("R1")
        self.assertFalse(missing.position_valid)
        self.assertFalse(missing.valid_for_pick)
        self.assertIsNone(missing.owner_robot_id)


if __name__ == "__main__":
    unittest.main()
