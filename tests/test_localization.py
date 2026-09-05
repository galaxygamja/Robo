import copy
import io
import json
import math
import tempfile
import unittest
from contextlib import redirect_stdout, redirect_stderr
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from robo_control.fleet import DEFAULT_ROLES, roles_from_scenario, validate_tag_registry
from robo_control.qualifier import allocate_tasks, configured_tasks, ground_conflicts, load_scenario, run_mock
from robo_control.vision.tracking import PoseTracker

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

ROOT = Path(__file__).resolve().parents[1]


def record(ids, seq=1, stamp=10., *, x=300., replay=False, media=None):
    return {"status":"detected", "sequence":seq, "captured_at_s":stamp,"source_name":"camera:test:1",
            "is_replay":replay,"media_time_s":media,"tag_size_mm":50.,"hardware_verified":False,
            "observation_complete":True,"unknown_tag_ids":[],"duplicate_tag_ids":[],
            "robots":[{"robot_id":rid,"robot_center_mm":[x+i*100,200.],"heading_rad":0.} for i,rid in enumerate(ids)]}


class TrackerTests(unittest.TestCase):
    def test_registry_4_5_6_12_and_watchdog(self):
        for n in (4,5,6,12):
            ids=[f"robot{i}" for i in range(n)]
            tracker=PoseTracker(ids)
            result=tracker.update(record(ids),10.01)
            self.assertEqual(n,len(result["tracks"]))
            self.assertTrue(result["observation_usable"])
            self.assertFalse(result["motion_permitted"])
            lost=tracker.snapshot(10.6)
            self.assertTrue(lost["stop_required"])
            self.assertTrue(all(t["state"]=="stale" and not t["valid_for_control"] for t in lost["tracks"]))

    def test_missing_duplicate_unknown_never_reuse_pose_as_valid(self):
        for mutation in (lambda r:r["robots"].pop(),lambda r:r["robots"].append(r["robots"][0]),
                         lambda r:r["robots"].append({"robot_id":"stranger","robot_center_mm":[0,0],"heading_rad":0})):
            tracker=PoseTracker(["H1","H2"]);tracker.update(record(["H1","H2"]),10.01)
            r=record(["H1","H2"],2,10.1);mutation(r)
            result=tracker.update(r,10.11)
            self.assertTrue(result["stop_required"])
            self.assertFalse(result["observation_usable"])

    def test_order_source_jumps_and_angle_wrap(self):
        tracker=PoseTracker(["H1"])
        first=record(["H1"]);first["robots"][0]["heading_rad"]=math.pi-.01
        tracker.update(first,10.01)
        second=record(["H1"],2,10.1);second["robots"][0]["heading_rad"]=-math.pi+.01
        self.assertTrue(tracker.update(second,10.11)["observation_usable"])
        self.assertTrue(tracker.update(second,10.12)["stop_required"])
        jump=record(["H1"],3,10.2,x=900.)
        self.assertEqual("pose_jump",tracker.update(jump,10.21)["tracking_rejections"]["H1"])
        jump["source_name"]="new-session";jump["sequence"]=4;jump["captured_at_s"]=10.3
        self.assertTrue(tracker.update(jump,10.31)["stop_required"])

    def test_unpaced_replay_uses_media_clock_not_decode_speed(self):
        t=PoseTracker(["H1"])
        t.update(record(["H1"],replay=True,media=0.),10.0001)
        result=t.update(record(["H1"],2,10.001,x=310.,replay=True,media=.1),10.0011)
        self.assertTrue(result["observation_usable"])
        self.assertAlmostEqual(result["tracks"][0]["velocity_mm_s"][0],100.)
        self.assertFalse(result["hardware_ready"])

    def test_malformed_inputs_fail_closed(self):
        for value in (None, [None], [{"robot_id":[]}], ["oops"]):
            r=record(["H1"]);r["robots"]=value
            self.assertTrue(PoseTracker(["H1"]).update(r,10.01)["stop_required"])
        for size in (None,True,float("inf"),float("nan"),-1):
            r=record(["H1"]);r.update(tag_size_mm=size,hardware_verified=True)
            result=PoseTracker(["H1"]).update(r,10.01)
            self.assertFalse(result["measurement_setup_marked_verified"])
            self.assertFalse(result["hardware_ready"])


class FleetExpansionTests(unittest.TestCase):
    def test_1_hamster_3_beavers_keep_original_printed_ids(self):
        data,pieces,zones=load_scenario(ROOT/"config/qualifier_senior.json")
        roles=roles_from_scenario(data)
        self.assertEqual(roles,DEFAULT_ROLES)
        self.assertEqual(list(roles.values()).count("hamster"),1)
        tags=json.loads((ROOT/"config/robot_tags.json").read_text())
        self.assertEqual(tags["tag_to_robot"],{"0":"H1","1":"H2","2":"B1","3":"B2"})
        mapping={int(k):v for k,v in tags["tag_to_robot"].items()}
        self.assertEqual(validate_tag_registry(data,mapping),roles)
        mapping[1]="another_robot"
        with self.assertRaises(ValueError):validate_tag_registry(data,mapping)
        self.assertEqual(configured_tasks(pieces,zones,data["task_plan"])[1].robot_id,"H1")

    def test_5_6_12_allocator_collision_and_stop_commands(self):
        base,original,zones=load_scenario(ROOT/"config/qualifier_senior.json")
        for n in (5,6,12):
            data=copy.deepcopy(base)
            for i in range(4,n):
                data["ground_robots"].append({"id":f"extra{i}","role":"beaver","name":f"Extra {i}"})
            roles=roles_from_scenario(data)
            pieces=[replace(p,held_by="extra4") if p.id=="C1" else p for p in original]
            tasks=allocate_tasks(pieces,zones,roles=roles)
            self.assertEqual(16,len(tasks))
            self.assertEqual("extra4",next(t.robot_id for t in tasks if t.piece_id=="C1"))
            poses={rid:(100.,100.,50.) for rid in roles}
            self.assertEqual(n*(n-1)//2,len(ground_conflicts(poses,roles=roles)))
            data.pop("task_plan");data["pieces"]=[{**p,"held_by":"extra4"} if p["id"]=="C1" else p for p in data["pieces"]]
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/"fleet.json";path.write_text(json.dumps(data),encoding="utf-8")
                result=run_mock(path)
                self.assertEqual(n,len(result["stop_commands"]))
                self.assertEqual(160,result["score"]["total"])


@unittest.skipUnless(cv2 is not None and np is not None,"install vision extra")
class RealImageExpansionTests(unittest.TestCase):
    def setUp(self):
        from robo_control.vision.calibration import FieldCalibration
        self.calibration=FieldCalibration((960,720),((0,0),(959,0),(959,719),(0,719)),(959.,719.))

    def image(self,n):
        image=np.full((720,960,3),255,np.uint8)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        for i in range(n):
            x,y=50+(i%4)*220,50+(i//4)*200
            marker=cv2.aruco.generateImageMarker(dictionary,i,60)
            image[y:y+60,x:x+60]=cv2.cvtColor(np.rot90(marker,i%4).copy(),cv2.COLOR_GRAY2BGR)
        return image

    def frame(self,image):
        from robo_control.adapters import CameraFrame
        return CameraFrame(image,10.,1,"camera:test")

    def test_generated_rotated_tags_5_6_12(self):
        from robo_control.vision.tags import AprilTagDetector,TagDetectorConfig
        for n in (5,6,12):
            detector=AprilTagDetector(TagDetectorConfig(tag_to_robot={i:f"robot{i}" for i in range(n)},tag_size_mm=60.),self.calibration)
            batch=detector.detect(self.frame(self.image(n)))
            self.assertTrue(batch.observation_complete)
            self.assertEqual(n,len(batch.observations))
            for obs in batch.observations:
                i=obs.tag_id
                self.assertLess(math.dist(obs.robot_center_mm,(79.5+(i%4)*220,719-79.5-(i//4)*200)),1.)
                error=(obs.heading_rad-(i%4)*math.pi/2+math.pi)%(2*math.pi)-math.pi
                self.assertAlmostEqual(error,0.,places=2)

    def test_mean_side_length_cannot_hide_distorted_tag(self):
        from robo_control.vision.tags import AprilTagDetector,TagDetectorConfig
        detector=AprilTagDetector(TagDetectorConfig(tag_to_robot={0:"H1"},tag_size_mm=50.,tag_size_tolerance_fraction=.1),self.calibration)
        detector._detector=SimpleNamespace(detectMarkers=lambda image:([np.array([[[400,400],[420,400],[420,480],[400,480]]],dtype=np.float32)],np.array([[0]]),[]))
        batch=detector.detect(self.frame(self.image(0)))
        self.assertFalse(batch.observation_complete)
        self.assertEqual("tag_size_mismatch",batch.rejected[0].reason)

    def test_hsv_colors_metric_sizes_and_merged_blob_rejection(self):
        from robo_control.vision.colors import ColorDetector
        detector=ColorDetector.load(self.calibration,ROOT/"config/object_colors.json")
        image=self.image(0)
        for x,color in ((200,(0,0,255)),(300,(0,255,255)),(400,(0,255,0))):
            cv2.circle(image,(x,300),10,color,-1)
        # Oversized red patch and noise must not become cylinders.
        cv2.rectangle(image,(500,300),(600,400),(0,0,255),-1)
        cv2.circle(image,(100,100),2,(0,255,0),-1)
        objects=detector.detect(self.frame(image))
        self.assertEqual({"red","green","yellow"},{o["color"] for o in objects})
        self.assertEqual(3,len(objects))
        self.assertTrue(all(o["identity"] is None and abs(o["center_mm"][1]-419)<.1 for o in objects))

    def test_cli_tracks_colors_and_terminal_stop_record(self):
        from robo_control.vision.__main__ import main
        from robo_control.vision.tags import TagDetectorConfig
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);cal=root/"cal.json";tags=root/"tags.json";photo=root/"input.png";log=root/"detect.jsonl"
            self.calibration.save(cal)
            TagDetectorConfig(tag_to_robot={i:f"robot{i}" for i in range(6)},tag_size_mm=60.).save(tags)
            image=self.image(6);cv2.circle(image,(500,600),10,(0,0,255),-1)
            photo.write_bytes(cv2.imencode(".png",image)[1].tobytes())
            with redirect_stdout(io.StringIO()),redirect_stderr(io.StringIO()):
                code=main(["detect","--image",str(photo),"--calibration",str(cal),"--tags",str(tags),"--track","--colors",str(ROOT/"config/object_colors.json"),"--report",str(log)])
            self.assertEqual(code,0)
            rows=[json.loads(s) for s in log.read_text().splitlines()]
            self.assertEqual(len(rows),2)
            self.assertTrue(rows[0]["observation_usable"])
            self.assertEqual(len(rows[0]["objects"]),1)
            self.assertEqual(rows[1]["status"],"source_closed")
            self.assertTrue(rows[1]["stop_required"])


if __name__=="__main__":unittest.main()
