"""Measured waypoint execution of reviewed qualifier tasks; no device transport.

The global route lease belongs to one robot until its task is confirmed done.
Other robots are observed obstacles, never assumed to vacate at a scheduled tick.
Object coordinates in the scene describe intentions, not measured world state.
"""
from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import asdict
from itertools import pairwise

from .control_loop import (
    ClosedLoopController,
    ControlLimits,
    _number,
    _polygon_distance,
)
from .models import Point, Rectangle
from .planner import GridSpec, ReservationTable, SpaceTimePlanner
from .qualifier import Feedback, Manipulator, Piece, Zone, configured_tasks
from .vision.calibration import COORDINATE_SYSTEM

MOTION_PHASES = {"approach": "pickup", "align_pickup": "pickup",
                 "carry": "drop", "align_drop": "drop", "retreat": "retreat"}


class MissionExecutor:
    """Serial task/route lease with measured arrival and fresh sensor gates."""

    def __init__(self, plan, fleet, *, roles, field_size_mm, limits=None):
        self.plan = deepcopy(plan)
        required = {"schema_version", "coordinate_system", "radii_mm", "cell_mm",
                    "obstacles_mm", "tasks"}
        if (not isinstance(plan, dict) or set(plan) != required
                or type(plan["schema_version"]) is not int or plan["schema_version"] != 1
                or plan["coordinate_system"] != COORDINATE_SYSTEM):
            raise ValueError("Mission needs schema 1, field-mm coordinates, radii, cell, obstacles and tasks")
        self.limits = limits or ControlLimits()
        self.roles = dict(roles)
        self.field = tuple(field_size_mm)
        checked = ClosedLoopController(roles=roles, radii_mm=plan["radii_mm"],
                                       field_size_mm=self.field, limits=self.limits)
        self.radii = checked.radii_mm
        self.cell_mm = plan["cell_mm"]
        if (not _number(self.cell_mm) or not 10 <= self.cell_mm <= 100
                or math.prod(math.ceil(v / self.cell_mm) for v in self.field) > 4096):
            raise ValueError("Mission grid needs 10..100 mm cells and at most 4096 cells")
        self.obstacles = plan["obstacles_mm"]
        if not isinstance(self.obstacles, list) or len(self.obstacles) > 64:
            raise ValueError("At most 64 reviewed static obstacles")
        for box in self.obstacles:
            if (not isinstance(box, dict) or set(box) != {"x_mm", "y_mm", "width_mm", "height_mm"}
                    or any(not _number(v) for v in box.values())
                    or box["width_mm"] <= 0 or box["height_mm"] <= 0
                    or box["x_mm"] < 0 or box["y_mm"] < 0
                    or box["x_mm"] + box["width_mm"] > self.field[0]
                    or box["y_mm"] + box["height_mm"] > self.field[1]):
                raise ValueError("Obstacle must be a positive rectangle within the field")
        pieces = [Piece(**p) for p in fleet["pieces"]]
        zones = [Zone(**z) for z in fleet["destinations"]]
        tasks = configured_tasks(pieces, zones, fleet.get("task_plan"), roles=roles)
        self.inventory = {p.id: p for p in pieces}
        self.zones = {z.id: z for z in zones}
        self.tasks = {t.id: t for t in tasks}
        self.entries = deepcopy(plan["tasks"])
        if not isinstance(self.entries, list) or not 1 <= len(self.entries) <= len(tasks):
            raise ValueError("Select at least one known qualifier task")
        seen = set()
        for entry in self.entries:
            if (not isinstance(entry, dict) or set(entry) != {"task_id", "pickup", "drop", "retreat"}
                    or not isinstance(entry["task_id"], str) or entry["task_id"] not in self.tasks
                    or entry["task_id"] in seen):
                raise ValueError("Unique task IDs and explicit pickup/drop/retreat robot poses required")
            seen.add(entry["task_id"])
            rid = self.tasks[entry["task_id"]].robot_id
            for name in ("pickup", "drop", "retreat"):
                goal = entry[name]
                if not isinstance(goal, dict) or set(goal) != {"x_mm", "y_mm", "heading_rad"}:
                    raise ValueError("Robot approach poses need x/y mm and heading rad")
                checked.set_goals({rid: goal})
                if not self._segment_clear((goal["x_mm"], goal["y_mm"]),
                                           (goal["x_mm"], goal["y_mm"]), rid, {}):
                    raise ValueError("Robot approach pose intersects a static obstacle")
        self.started_at = None
        self.elapsed_s = 0.0
        self.index = 0
        self.active = None
        self.route = None
        self.route_index = 0
        self.route_phase = None
        self.arrival_streak = 0
        self.completed = []
        self.fault = None
        self.closed = False
        self.world = None
        self.session_id = None
        self.phase_serial = 0
        self.last_phase = None
        self.feedback_reason = None

    @property
    def done(self):
        return self.index == len(self.entries) and not self.fault

    @property
    def command_id(self):
        return f"{self.session_id}:{self.index}:{self.phase_serial}"

    def poll(self, now_s):
        if self.started_at is None:
            self.started_at = now_s
        self.elapsed_s = now_s - self.started_at
        if self.closed or self.fault or self.done:
            return
        if self.elapsed_s >= 120:
            self.fault = "match_timeout"
        elif self.active:
            self.active.tick(self.elapsed_s)
            self.fault = self.active.fault

    def interrupt(self, reason):
        self.route = None
        self.arrival_streak = 0
        # A manipulation may have occurred before visibility was lost. Require
        # an operator's new session instead of repeating the same servo action.
        if self.active and self.active.phase not in MOTION_PHASES and self.active.phase != "done":
            self.fault = f"manipulation_interrupted:{reason}"

    def close(self):
        self.closed = True
        self.route = None

    def _phase_changed(self):
        phase = self.active.phase
        if phase != self.last_phase:
            self.phase_serial += 1
            self.last_phase = phase
            self.route = None
            self.arrival_streak = 0
            self.active.phase_timeout_s = 30.0 if phase in MOTION_PHASES else 4.0

    def _segment_clear(self, start, end, rid, poses, extra_mm=0.0):
        inset = self.radii[rid] + self.limits.clearance_mm + extra_mm
        if any(not inset <= p[a] <= size - inset for p in (start, end)
               for a, size in enumerate(self.field)):
            return False
        segment = [tuple(start), tuple(end)]
        for box in self.obstacles:
            x, y, w, h = (box[k] for k in ("x_mm", "y_mm", "width_mm", "height_mm"))
            polygon = [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]
            if _polygon_distance(segment, polygon) <= inset:
                return False
        for other, pose in poses.items():
            if other != rid and _polygon_distance(segment, [tuple(pose["robot_center_mm"])]) <= inset + self.radii[other]:
                return False
        return True

    def _plan(self, rid, pose, goal, poses):
        # Reuse the existing A* in metres. Grid ticks are spatial search depth,
        # NEVER elapsed runtime or an assumption that another robot has moved.
        pad = self.radii[rid] + self.limits.clearance_mm + self.cell_mm / math.sqrt(2)
        grid = GridSpec(self.field[0]/1000, self.field[1]/1000, self.cell_mm/1000, pad/1000)
        obstacles = [Rectangle(*(box[k]/1000 for k in ("x_mm", "y_mm", "width_mm", "height_mm")))
                     for box in self.obstacles]
        for other, p in poses.items():
            if other != rid:
                x, y = p["robot_center_mm"]
                r = self.radii[other]
                obstacles.append(Rectangle((x-r)/1000, (y-r)/1000, 2*r/1000, 2*r/1000))
        planner = SpaceTimePlanner(grid, obstacles)
        start = tuple(pose["robot_center_mm"])
        end = (goal["x_mm"], goal["y_mm"])
        cells = planner.plan(Point(start[0]/1000, start[1]/1000), Point(end[0]/1000, end[1]/1000),
                             ReservationTable(), grid.columns * grid.rows)
        if cells is None:
            return None
        points = [start] + [(grid.cell_to_world(c).x*1000, grid.cell_to_world(c).y*1000) for c in cells] + [end]
        # Keep turns, discard only collinear interior nodes. Check the real
        # start/end connectors too, because they need not lie at cell centres.
        reduced = [points[0]]
        for i in range(1, len(points)-1):
            a, b, c = reduced[-1], points[i], points[i+1]
            if abs((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0])) > 1e-6:
                reduced.append(b)
        reduced.append(end)
        if any(not self._segment_clear(a, b, rid, poses) for a, b in pairwise(reduced)):
            return None
        result = [{"x_mm": x, "y_mm": y} for x, y in reduced[1:]]
        result[-1]["heading_rad"] = goal["heading_rad"]
        return result

    def _feedback(self, value, now_s):
        self.feedback_reason = None
        if value is None:
            return None
        rid = self.active.task.robot_id
        if (not isinstance(value, dict) or value.get("session_id") != self.session_id
                or value.get("command_id") != self.command_id or value.get("robot_id") != rid
                or value.get("synthetic") is not False):
            self.feedback_reason = "wrong_feedback_identity_or_synthetic"
            return None
        stamp, signals = value.get("observed_at_s"), value.get("signals")
        if (not _number(stamp) or not 0 <= now_s - stamp < self.limits.max_pose_age_s
                or stamp - self.started_at <= self.active.entered_at_s
                or not isinstance(signals, dict) or any(type(v) is not bool for v in signals.values())):
            self.feedback_reason = "invalid_or_stale_feedback"
            return None
        if self.active.phase == "confirm_clear" and (value.get("piece_id") != self.active.piece.id
                or any(not _number(value.get(k)) for k in ("piece_x_mm", "piece_y_mm", "piece_yaw_rad"))):
            self.feedback_reason = "release_needs_measured_object"
            return None
        return Feedback(stamp-self.started_at, self.active.phase, dict(signals),
                        piece_id=value.get("piece_id"), piece_x_mm=value.get("piece_x_mm"),
                        piece_y_mm=value.get("piece_y_mm"), piece_yaw_rad=value.get("piece_yaw_rad", 0.0))

    def prepare(self, record, now_s, feedback=None, *, world):
        if self.fault or self.closed or self.done:
            return {}
        self.world = world.as_dict()
        if (not world.ready or world.session_id != self.session_id or world.generated_at_s != now_s
                or world.source_sequence != record["sequence"]
                or any(not robot.valid_for_control for robot in world.robots)):
            self.interrupt("world_not_ready")
            return {}
        if self.active is None:
            task = self.tasks[self.entries[self.index]["task_id"]]
            self.active = Manipulator(task, self.inventory[task.piece_id], self.zones[task.destination_id],
                                      start_s=self.elapsed_s, roles=self.roles)
            self.last_phase = None
            self._phase_changed()
        if self.active.phase not in MOTION_PHASES:
            accepted = self._feedback(feedback, now_s)
            self.active.tick(self.elapsed_s, accepted)
            self._phase_changed()
            if self.active.phase == "done":
                self.completed.append({"task_id": self.active.task.id,
                    "released_object": asdict(self.active.released_object), "at_s": now_s})
                self.index += 1
                self.active = None
                return {}
        self.fault = self.active.fault
        if self.fault or self.active.phase not in MOTION_PHASES:
            return {}
        rid = self.active.task.robot_id
        poses = {p.robot_id: {"robot_center_mm": p.position_mm, "heading_rad": p.heading_rad,
                             "velocity_mm_s": p.velocity_mm_s} for p in world.robots}
        goal = self.entries[self.index][MOTION_PHASES[self.active.phase]]
        if self.route is None:
            self.route = self._plan(rid, poses[rid], goal, poses)
            self.route_index = 0
            self.route_phase = self.active.phase
            if self.route is None:
                self.fault = "route_unavailable"
                return {}
        target = self.route[self.route_index]
        # Freshly check the connector on EVERY observation, including after an
        # unexpected externally moved robot or a route-tracking deviation.
        if not self._segment_clear(poses[rid]["robot_center_mm"],
                                   (target["x_mm"], target["y_mm"]), rid, poses):
            self.fault = "route_obstructed"
            return {}
        return {rid: dict(target)}

    def guard_command(self, packet, record):
        if self.active is None or self.fault:
            return
        poses = {p["robot_id"]: p for p in record["tracks"]}
        for command in packet["robots"]:
            rid = command["robot_id"]
            pose = poses[rid]
            measured = pose["velocity_mm_s"]
            proposed = command["velocity_world_mm_s"]
            speed = max(math.hypot(*measured), math.hypot(*proposed))
            extra = speed*self.limits.command_ttl_s + speed*speed/(2*self.limits.braking_mm_s2)
            start = pose["robot_center_mm"]
            for velocity in (measured, proposed):
                end = [start[a]+velocity[a]*self.limits.prediction_horizon_s for a in (0, 1)]
                if not self._segment_clear(start, end, rid, {}, extra):
                    self.fault = "mission_obstacle_envelope"
                    return

    def accept_arrival(self, packet, record, now_s):
        if self.fault or self.active is None or self.active.phase not in MOTION_PHASES:
            return
        rid = self.active.task.robot_id
        pose = next(p for p in record["tracks"] if p["robot_id"] == rid)
        command = next(c for c in packet["robots"] if c["robot_id"] == rid)
        settled = math.hypot(*pose["velocity_mm_s"]) <= 10 and abs(pose["angular_velocity_rad_s"]) <= .1
        self.arrival_streak = self.arrival_streak + 1 if (packet["stop_reason"] is None
            and command["at_goal"] and settled) else 0
        if self.arrival_streak < 2:
            return
        self.arrival_streak = 0
        if self.route_index < len(self.route)-1:
            self.route_index += 1
            return
        # Derive ONLY navigation signals from camera poses. Servo/presence/
        # release/settled-piece signals must come through the sensor boundary.
        stamp = record["captured_at_s"]-self.started_at
        self.active.tick(self.elapsed_s, Feedback(stamp, self.active.phase,
            {signal: True for signal in self.active.required_signals}))
        self._phase_changed()
        if self.active.phase == "done":
            self.completed.append({"task_id": self.active.task.id,
                "released_object": asdict(self.active.released_object), "at_s": now_s})
            self.index += 1
            self.active = None

    def snapshot(self):
        intent = None
        if self.active and not self.closed and not self.fault:
            intent = self.active.command()
            # Qualifier's historical four-wheel placeholder is not an output
            # contract for this differential body-command runtime.
            intent.pop("wheel_velocity_rad_s", None)
            intent.pop("motion_goal", None)
            intent.update(command_id=self.command_id, session_id=self.session_id,
                          device_io=False, dispatch_enabled=False)
        return {"status": "closed" if self.closed else "fault" if self.fault else "completed" if self.done
                else "running" if self.active else "awaiting_observation",
                "fault": self.fault, "elapsed_s": self.elapsed_s,
                "task_index": self.index, "completed_tasks": deepcopy(self.completed),
                "active_task_id": self.active.task.id if self.active else None,
                "phase": self.active.phase if self.active else None,
                "route": deepcopy(self.route), "waypoint_index": self.route_index,
                "route_lease_robot_id": self.active.task.robot_id if self.active and not self.closed else None,
                "manipulator_intent": intent, "feedback_rejection": self.feedback_reason,
                "world": deepcopy(self.world), "device_io": False,
                "score": None, "physical_mission_verified": False}
