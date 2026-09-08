"""Measured object position + explicitly reviewed tool offset -> robot goal.

No wheel geometry, tool reach, cube yaw, ownership or sensor success is inferred.
The offset is ROBOT CENTRE TO PICKUP CONTACT in the body frame (+forward,+left).
The desired body heading is reviewed; it is not the object's unmeasured yaw.
"""
from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import dataclass

from .mission_bindings import number


@dataclass(frozen=True, slots=True)
class PickupPolicy:
    heading_rad: float
    tool_forward_mm: float
    tool_left_mm: float
    max_anchor_drift_mm: float
    replan_distance_mm: float
    max_replans: int

    @classmethod
    def parse(cls, value, *, field_size_mm):
        fields = {"mode", "heading_rad", "tool_forward_mm", "tool_left_mm",
                  "max_anchor_drift_mm", "replan_distance_mm", "max_replans"}
        if (not isinstance(value, dict) or set(value) != fields or value["mode"] != "observed_piece"
                or any(not number(value[key]) for key in fields - {"mode", "max_replans"})
                or type(value["max_replans"]) is not int or not 0 <= value["max_replans"] <= 100):
            raise ValueError("Observed pickup needs explicit finite heading/tool offsets/drift/replan limits")
        diagonal = math.hypot(*field_size_mm)
        if (math.hypot(value["tool_forward_mm"], value["tool_left_mm"]) > diagonal
                or not 0 < value["replan_distance_mm"] <= value["max_anchor_drift_mm"] <= diagonal):
            raise ValueError("Observed pickup offset/drift/replan distance exceeds field bounds")
        return cls((value["heading_rad"]+math.pi) % (2*math.pi)-math.pi,
                   float(value["tool_forward_mm"]), float(value["tool_left_mm"]),
                   float(value["max_anchor_drift_mm"]), float(value["replan_distance_mm"]), value["max_replans"])


class ObservedPickupTarget:
    """A task-local anchor/drift/replan budget that survives camera interruptions."""

    def __init__(self, policy, *, task_id, piece_id):
        self.policy = policy
        self.task_id, self.piece_id = task_id, piece_id
        self._session = self._track = self._anchor = None
        self._sequence = 0
        self._goal = self._planned_goal = None
        self._point = self._capture = self._shift = None
        self.replans = 0
        self.fault = None

    @property
    def goal(self):
        return deepcopy(self._goal)

    def _reject(self, reason):
        self.fault = reason
        self._goal = None  # Old geometry is not a fallback for invalid evidence.

    def observe(self, world):
        if self.fault:
            return None
        if not world.ready:
            return None
        if self._session is not None and self._session != world.session_id:
            return self._reject("pickup_source_session_changed")
        piece = world.piece(self.piece_id)
        if (piece is None or piece.kind == "cube" or not piece.valid_for_pick or not piece.position_valid
                or piece.track_id is None or piece.position_mm is None
                or piece.observed_at_s != world.captured_at_s):
            return self._reject("pickup_observation_unavailable")
        if self._track is not None and self._track != piece.track_id:
            return self._reject("pickup_track_changed")
        if world.source_sequence < self._sequence:
            return self._reject("pickup_observation_reordered")
        if world.source_sequence == self._sequence:
            # A repeated snapshot does not reset the anchor or consume budget.
            return self.goal
        point = piece.position_mm
        self._session, self._track = world.session_id, piece.track_id
        self._anchor = self._anchor or point
        self._point, self._capture, self._sequence = point, piece.observed_at_s, world.source_sequence
        self._shift = math.dist(point, self._anchor)
        if self._shift > self.policy.max_anchor_drift_mm:
            return self._reject("pickup_anchor_drift_exceeded")
        heading = self.policy.heading_rad
        forward, left = self.policy.tool_forward_mm, self.policy.tool_left_mm
        self._goal = {"x_mm": point[0] - forward*math.cos(heading) + left*math.sin(heading),
                      "y_mm": point[1] - forward*math.sin(heading) - left*math.cos(heading),
                      "heading_rad": heading}
        return self.goal

    def replan_required(self):
        if self._planned_goal is None or self._goal is None:
            return False
        return math.hypot(self._goal["x_mm"] - self._planned_goal["x_mm"],
                          self._goal["y_mm"] - self._planned_goal["y_mm"]) > self.policy.replan_distance_mm

    def request_replan(self):
        if self.fault:
            return False
        if self.replans >= self.policy.max_replans:
            self._reject("pickup_replan_budget_exceeded")
            return False
        self.replans += 1
        # The pending new plan must not consume another budget unit on polls
        # or frames arriving while transport is applying backpressure.
        self._planned_goal = None
        return True

    def planned(self):
        self._planned_goal = self.goal

    def snapshot(self):
        return {"mode": "observed_piece", "task_id": self.task_id, "piece_id": self.piece_id,
                "session_id": self._session, "track_id": self._track,
                "source_sequence": self._sequence, "observed_at_s": self._capture,
                "object_position_mm": self._point, "anchor_position_mm": self._anchor,
                "anchor_drift_mm": self._shift, "robot_goal": self.goal,
                "replans": self.replans, "fault": self.fault,
                "position_evidence": "vision" if self._point is not None else None,
                "physical_pickup_verified": False, "device_io": False, "motion_permitted": False}
