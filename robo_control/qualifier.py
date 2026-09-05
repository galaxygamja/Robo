"""Hardware-independent Korean Senior qualifier mission model.

This module sends no radio, serial, GPIO, or drone commands. The CLI explicitly
uses synthetic feedback; the command records are intentions for a future adapter.
All coordinates are millimetres and destination rectangles describe INNER edges.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass, replace
from math import cos, hypot, isfinite, sin
from pathlib import Path
from typing import Any, Iterable

from .config import default_data_path

RULE_VERSION = "KR qualifier v2 draft r5 (2026-08-12), Senior"
MATCH_SECONDS = 120.0
EPSILON_MM = 1e-6
EXPECTED_COLOUR = {"H": "red", "PCC-L": "yellow", "PCC-R": "yellow", "RZ": "green"}
ROBOT_ROLES = {"H1": "hamster", "H2": "hamster", "B1": "beaver", "B2": "beaver"}
ROBOT_NAMES = {"H1": "햄스터 1", "H2": "햄스터 2", "B1": "한가한 비버", "B2": "바쁜 비버"}


def finite_values(*values: float) -> None:
    if not all(isfinite(value) for value in values):
        raise ValueError("Coordinates and times must be finite")


@dataclass(frozen=True)
class Piece:
    id: str
    kind: str
    x_mm: float
    y_mm: float
    colour: str | None = None
    released: bool = True
    held_by: str | None = None
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        finite_values(self.x_mm, self.y_mm, self.yaw_rad)
        if self.kind not in {"disc", "cube", "cylinder"}:
            raise ValueError(f"Unknown piece kind: {self.kind}")
        if self.kind == "cylinder" and self.colour not in {"red", "yellow", "green"}:
            raise ValueError("Cylinder colour must be red, yellow, or green")
        if type(self.released) is not bool:
            raise ValueError("released must be a JSON Boolean, not a string or number")
        if self.held_by is not None and self.held_by not in ROBOT_ROLES:
            raise ValueError("held_by must name a ground robot")
        if not self.id:
            raise ValueError("A physical piece needs a stable, non-empty ID")

    @property
    def half_extent_mm(self) -> float:
        if self.kind == "cube":
            return 12.5 * (abs(cos(self.yaw_rad)) + abs(sin(self.yaw_rad)))
        return 28.0 if self.kind == "disc" else 10.0

    @property
    def free(self) -> bool:
        return self.released and self.held_by is None


@dataclass(frozen=True)
class Zone:
    id: str
    shape: str
    x_mm: float
    y_mm: float
    width_mm: float = 0.0
    height_mm: float = 0.0
    radius_mm: float = 0.0

    def __post_init__(self) -> None:
        finite_values(self.x_mm, self.y_mm, self.width_mm, self.height_mm, self.radius_mm)
        if self.shape == "rect" and self.width_mm > 0 and self.height_mm > 0:
            return
        if self.shape == "circle" and self.radius_mm > 0:
            return
        raise ValueError("A zone needs positive dimensions and shape rect or circle")

    def contains(self, piece: Piece) -> bool:
        """Strict full footprint containment: touching a boundary never counts."""
        if self.shape == "circle":
            if piece.kind != "disc":
                return False
            return hypot(piece.x_mm - self.x_mm, piece.y_mm - self.y_mm) + 28.0 < self.radius_mm - EPSILON_MM
        half = piece.half_extent_mm
        return (
            piece.x_mm - half > self.x_mm + EPSILON_MM
            and piece.x_mm + half < self.x_mm + self.width_mm - EPSILON_MM
            and piece.y_mm - half > self.y_mm + EPSILON_MM
            and piece.y_mm + half < self.y_mm + self.height_mm - EPSILON_MM
        )

    @property
    def center(self) -> tuple[float, float]:
        if self.shape == "circle":
            return self.x_mm, self.y_mm
        return self.x_mm + self.width_mm / 2, self.y_mm + self.height_mm / 2


def unique_pieces(pieces: Iterable[Piece]) -> dict[str, Piece]:
    result: dict[str, Piece] = {}
    for piece in pieces:
        if piece.id in result:
            raise ValueError(f"Duplicate physical piece ID: {piece.id}")
        result[piece.id] = piece
    return result


def score_senior(pieces: Iterable[Piece], zones: Iterable[Zone], *, elapsed_s: float) -> dict[str, Any]:
    """Score ONE final snapshot taken at the end, never a task completion log.

    A caller must capture the final snapshot at/before 120 s. Snapshots dated
    later are rejected because they cannot reconstruct the true final state.
    """
    finite_values(elapsed_s)
    if not 0 <= elapsed_s <= MATCH_SECONDS:
        raise ValueError("Score the frozen end snapshot at 0..120 seconds")
    inventory = unique_pieces(pieces)
    all_zones = list(zones)
    zone_map = {zone.id: zone for zone in all_zones}
    if len(zone_map) != len(all_zones):
        raise ValueError("Duplicate destination ID")
    required = {*EXPECTED_COLOUR, "LAB-1", "LAB-2", "LAB-3"}
    if not required <= zone_map.keys():
        raise ValueError(f"Missing destinations: {sorted(required - zone_map.keys())}")
    if any(zone_map[key].shape != "rect" for key in EXPECTED_COLOUR):
        raise ValueError("Healthcare zones must describe rectangular inner edges")
    if any(zone_map[f"LAB-{i}"].shape != "circle" for i in range(1, 4)):
        raise ValueError("Each LAB slot must describe its own circle")

    contents: dict[str, list[Piece]] = {key: [] for key in required}
    ignored: dict[str, str] = {}
    for piece in inventory.values():
        if not piece.free:
            ignored[piece.id] = "not_released"
            continue
        candidates = [key for key in required if zone_map[key].contains(piece)]
        if len(candidates) != 1:
            ignored[piece.id] = "outside_or_boundary" if not candidates else "ambiguous_overlapping_zones"
            continue
        contents[candidates[0]].append(piece)

    contaminated = sorted(
        key for key, colour in EXPECTED_COLOUR.items()
        if any(p.kind == "cylinder" and p.colour != colour for p in contents[key])
    )
    valid_cylinders = {
        key: sorted(p.id for p in contents[key] if p.kind == "cylinder" and p.colour == colour)
        if key not in contaminated else []
        for key, colour in EXPECTED_COLOUR.items()
    }
    # The split is a physical condition. A contaminated PCC loses its own points;
    # a correct yellow in that PCC still satisfies the separate distribution rule.
    yellow_split = all(any(p.kind == "cylinder" and p.colour == "yellow" for p in contents[key]) for key in ("PCC-L", "PCC-R"))
    discs = []
    for i in range(1, 4):
        discs.extend(sorted(p.id for p in contents[f"LAB-{i}"] if p.kind == "disc")[:1])
    cubes = []
    for key, limit in (("H", 2), ("PCC-L", 1), ("PCC-R", 1)):
        cubes.extend(sorted(p.id for p in contents[key] if p.kind == "cube")[:limit])
    scored = {
        "discs": discs[:3],
        "cubes": cubes[:4],
        "red": valid_cylinders["H"][:3],
        "yellow": (valid_cylinders["PCC-L"] + valid_cylinders["PCC-R"])[:3] if yellow_split else [],
        "green": valid_cylinders["RZ"][:3],
    }
    points = {category: 10 * len(ids) for category, ids in scored.items()}
    return {
        "rule_version": RULE_VERSION,
        "draft_scoring": True,
        "elapsed_s": elapsed_s,
        "points": points,
        "total": sum(points.values()),
        "maximum": 160,
        "scored_ids": scored,
        "contaminated_destinations": contaminated,
        "yellow_split_satisfied": yellow_split,
        "ignored_objects": ignored,
    }


@dataclass(frozen=True)
class Task:
    id: str
    piece_id: str
    robot_id: str
    destination_id: str
    target_x_mm: float
    target_y_mm: float


def allocate_tasks(pieces: Iterable[Piece], zones: Iterable[Zone]) -> list[Task]:
    """Deterministic role allocator, not a shortest-path or optimal-time solver.

    Two hamster queues share three discs. Two beavers share nine cylinders and
    four cubes. Preloaded cubes remain assigned to their actual hopper owner.
    """
    inventory = unique_pieces(pieces)
    zone_map = {zone.id: zone for zone in zones}
    tasks: list[Task] = []
    loads = {robot: 0 for robot in ROBOT_ROLES}
    slot_use: dict[str, int] = {}

    def add(piece: Piece, destination: str) -> None:
        compatible = ("H1", "H2") if piece.kind == "disc" else ("B1", "B2")
        if piece.kind == "cube" and piece.held_by not in compatible:
            raise ValueError("Cube tasks require a known preloaded beaver hopper owner")
        if piece.held_by:
            if piece.held_by not in compatible:
                raise ValueError(f"{piece.id} is preloaded on an incompatible robot")
            robot_id = piece.held_by
        else:
            robot_id = min(compatible, key=lambda robot: (loads[robot], robot))
        zone = zone_map[destination]
        x, y = zone.center
        index = slot_use.get(destination, 0)
        slot_use[destination] = index + 1
        if zone.shape == "rect":
            # A simple staging grid, explicitly provisional. Cubes are assigned
            # first so later cylinders do not use the same destination point.
            x = zone.x_mm + 25 + (index % 3) * 35
            y = zone.y_mm + 25 + (index // 3) * 40
        placed = replace(piece, x_mm=x, y_mm=y, released=True, held_by=None, yaw_rad=0)
        if not zone.contains(placed):
            raise ValueError(f"Provisional placement grid does not fit {piece.id} in {destination}")
        tasks.append(Task(f"move-{piece.id}", piece.id, robot_id, destination, x, y))
        loads[robot_id] += 1

    ordered = sorted(inventory.values(), key=lambda piece: piece.id)
    discs = [piece for piece in ordered if piece.kind == "disc"]
    cubes = [piece for piece in ordered if piece.kind == "cube"]
    if len(discs) != 3 or len(cubes) != 4:
        raise ValueError("Senior setup requires three discs and four cubes")
    for piece, destination in zip(discs, ("LAB-1", "LAB-2", "LAB-3"), strict=True):
        add(piece, destination)
    for piece, destination in zip(cubes, ("H", "PCC-L", "H", "PCC-R"), strict=True):
        add(piece, destination)
    for colour, destinations in (("red", ("H",) * 3), ("yellow", ("PCC-L", "PCC-R", "PCC-L")), ("green", ("RZ",) * 3)):
        available = [piece for piece in ordered if piece.kind == "cylinder" and piece.colour == colour]
        if len(available) != 4:
            raise ValueError(f"Senior setup requires four {colour} cylinders")
        for piece, destination in zip(available[:3], destinations, strict=True):
            add(piece, destination)
    return tasks


def configured_tasks(pieces: Iterable[Piece], zones: Iterable[Zone], plan: list[dict[str, Any]] | None) -> list[Task]:
    """Validate an explicit scene plan, or use the generic role allocator.

    The shipped plan shares object IDs, assignments and drop positions with the
    web scene. It is a scenario schedule, not a learned or globally optimal plan.
    """
    objects, destinations = list(pieces), list(zones)
    fallback = allocate_tasks(objects, destinations)  # Validate the kit inventory.
    if plan is None:
        return fallback
    inventory = unique_pieces(objects)
    zone_map = {zone.id: zone for zone in destinations}
    tasks = [Task(**entry) for entry in plan]
    if len(tasks) != 16 or len({task.id for task in tasks}) != 16 or len({task.piece_id for task in tasks}) != 16:
        raise ValueError("The Senior scene plan needs 16 unique tasks and physical objects")
    projected = dict(inventory)
    for task in tasks:
        if task.piece_id not in inventory or task.destination_id not in zone_map or task.robot_id not in ROBOT_ROLES:
            raise ValueError("Scene plan references an unknown object, zone, or robot")
        finite_values(task.target_x_mm, task.target_y_mm)
        piece = inventory[task.piece_id]
        if (piece.kind == "disc") != (ROBOT_ROLES[task.robot_id] == "hamster"):
            raise ValueError("Scene plan violates robot manipulation roles")
        if piece.held_by not in {None, task.robot_id}:
            raise ValueError("Scene plan reassigns another robot's preloaded object")
        projected[piece.id] = replace(piece, x_mm=task.target_x_mm, y_mm=task.target_y_mm,
                                      released=True, held_by=None, yaw_rad=0)
        if not zone_map[task.destination_id].contains(projected[piece.id]):
            raise ValueError(f"Scene drop target is outside {task.destination_id}")
    if score_senior(projected.values(), destinations, elapsed_s=0)["total"] != 160:
        raise ValueError("The complete scene plan must satisfy all Senior placement rules")
    return tasks


@dataclass(frozen=True)
class Feedback:
    observed_at_s: float
    phase: str
    signals: dict[str, bool]
    piece_id: str | None = None
    piece_x_mm: float | None = None
    piece_y_mm: float | None = None
    piece_yaw_rad: float = 0.0
    synthetic: bool = False


class Manipulator:
    """One carried item at a time, with fresh sensor gates at every transition."""

    PICK_PHASES = ("approach", "align_pickup", "close_servo", "confirm_grip", "retract")
    DROP_PHASES = ("carry", "align_drop", "release_servo", "confirm_clear", "retreat")

    def __init__(self, task: Task, piece: Piece, destination: Zone, *, start_s: float = 0.0,
                 phase_timeout_s: float = 4.0, feedback_max_age_s: float = 0.25,
                 allow_synthetic: bool = False) -> None:
        finite_values(start_s, phase_timeout_s, feedback_max_age_s)
        if not 0 <= start_s < MATCH_SECONDS or phase_timeout_s <= 0 or feedback_max_age_s <= 0:
            raise ValueError("Invalid start or timeout")
        if task.robot_id not in ROBOT_ROLES or task.piece_id != piece.id or task.destination_id != destination.id:
            raise ValueError("Task, robot, piece, and destination must agree")
        if (piece.kind == "disc") != (ROBOT_ROLES[task.robot_id] == "hamster"):
            raise ValueError("Hamsters handle discs; beavers handle cylinders and cubes")
        if piece.held_by not in {None, task.robot_id}:
            raise ValueError("Piece is held by another robot")
        if piece.kind == "cube" and piece.held_by != task.robot_id:
            raise ValueError("Cube hopper workflow requires a preloaded cube on this beaver")
        self.task, self.piece, self.destination = task, piece, destination
        self.current_object = piece
        self.phases = (("confirm_load",) if piece.kind == "cube" else self.PICK_PHASES) + self.DROP_PHASES
        self.index = 0
        self.entered_at_s = self.last_tick_s = start_s
        self.phase_timeout_s, self.feedback_max_age_s = phase_timeout_s, feedback_max_age_s
        self.allow_synthetic = allow_synthetic
        self.fault: str | None = None
        self.released_object: Piece | None = None
        self.events: list[dict[str, Any]] = []

    @property
    def phase(self) -> str:
        if self.fault:
            return "fault"
        return self.phases[self.index] if self.index < len(self.phases) else "done"

    @property
    def required_signals(self) -> tuple[str, ...]:
        presence = "optical_present" if self.piece.kind == "disc" else "gripper_present"
        clear = {"disc": "optical_clear", "cylinder": "gripper_clear", "cube": "hopper_clear"}[self.piece.kind]
        return {
            "approach": ("pickup_reached",), "align_pickup": ("pickup_aligned",),
            "close_servo": ("servo_closed",), "confirm_grip": (presence,),
            "retract": ("arm_retracted",), "confirm_load": ("hopper_loaded",),
            "carry": ("destination_reached",), "align_drop": ("drop_aligned",),
            "release_servo": ("servo_open",), "confirm_clear": (clear, "object_released", "object_settled"),
            "retreat": ("retreat_reached",),
        }.get(self.phase, ())

    def command(self) -> dict[str, Any]:
        phase = self.phase
        target = None
        if phase in {"approach", "align_pickup"}:
            target = {"x_mm": self.piece.x_mm, "y_mm": self.piece.y_mm, "purpose": "pickup"}
        elif phase in {"carry", "align_drop"}:
            target = {"x_mm": self.task.target_x_mm, "y_mm": self.task.target_y_mm, "purpose": "drop"}
        servo = "hold"
        if phase == "close_servo":
            servo = "disc_latch_close" if self.piece.kind == "disc" else "gripper_close"
        elif phase == "release_servo":
            servo = {"disc": "disc_latch_open", "cylinder": "gripper_open", "cube": "hopper_gate_open_one"}[self.piece.kind]
        elif phase == "retract":
            servo = "arm_retract"
        return {
            "robot_id": self.task.robot_id, "phase": phase,
            "wheel_velocity_rad_s": [0.0, 0.0, 0.0, 0.0],
            "motion_goal": target, "servo_intent": servo,
            "motion_intent": "retreat_from_released_piece" if phase == "retreat" else "navigate_to_goal" if target else "hold",
            "retreat_distance_mm": 110.0 if phase == "retreat" else None,
            "retreat_reference": {"x_mm": self.current_object.x_mm, "y_mm": self.current_object.y_mm} if phase == "retreat" else None,
            "required_signals": list(self.required_signals), "fault": self.fault,
            "device_io": False,
        }

    def stop(self, reason: str, now_s: float) -> dict[str, Any]:
        self.fault = reason
        self.events.append({"at_s": round(now_s, 3), "phase": "fault", "reason": reason})
        return self.command()

    def tick(self, now_s: float, feedback: Feedback | None = None) -> dict[str, Any]:
        finite_values(now_s)
        if now_s < self.last_tick_s:
            raise ValueError("Mission clock must be monotonic")
        self.last_tick_s = now_s
        if self.phase in {"done", "fault"}:
            return self.command()
        if now_s >= MATCH_SECONDS:
            return self.stop("match_timeout", now_s)
        if now_s - self.entered_at_s >= self.phase_timeout_s:
            return self.stop(f"sensor_timeout:{self.phase}", now_s)
        if feedback is None:
            return self.command()
        finite_values(feedback.observed_at_s)
        if feedback.synthetic and not self.allow_synthetic:
            return self.stop("synthetic_feedback_not_enabled", now_s)
        if (feedback.phase != self.phase or feedback.observed_at_s <= self.entered_at_s
                or not 0 <= now_s - feedback.observed_at_s <= self.feedback_max_age_s):
            return self.command()
        if not all(feedback.signals.get(signal) is True for signal in self.required_signals):
            return self.command()
        if self.phase == "confirm_clear":
            if feedback.piece_id != self.piece.id or feedback.piece_x_mm is None or feedback.piece_y_mm is None:
                return self.command()
            observed_piece = replace(self.piece, x_mm=feedback.piece_x_mm, y_mm=feedback.piece_y_mm,
                                     yaw_rad=feedback.piece_yaw_rad, released=True, held_by=None)
            if not self.destination.contains(observed_piece):
                return self.command()
            self.released_object = observed_piece
            self.current_object = observed_piece
        elif self.phase in {"confirm_grip", "confirm_load"}:
            self.current_object = replace(self.piece, released=False, held_by=self.task.robot_id)
        elif self.phase == "carry":
            self.current_object = replace(self.current_object, x_mm=self.task.target_x_mm, y_mm=self.task.target_y_mm)
        self.events.append({"at_s": round(now_s, 3), "phase": self.phase, "confirmed": True,
                            "synthetic": feedback.synthetic})
        self.index += 1
        self.entered_at_s = now_s
        return self.command()


def ground_conflicts(poses: dict[str, tuple[float, float, float]], *, clearance_mm: float = 10.0) -> list[tuple[str, str]]:
    """Conservative circular envelopes, including arm/load, for ALL ground pairs.

    This detects current conflicts, not swept trajectories. A path controller
    still needs time reservations/braking distances before actual motor output.
    """
    finite_values(clearance_mm)
    if clearance_mm < 0 or not poses.keys() <= ROBOT_ROLES.keys():
        raise ValueError("Use ground IDs H1, H2, B1, B2 and a non-negative clearance")
    for x, y, radius in poses.values():
        finite_values(x, y, radius)
        if radius <= 0:
            raise ValueError("Envelope radius must include the robot, arm, and load")
    ids = sorted(poses)
    conflicts = []
    for index, first in enumerate(ids):
        x1, y1, radius1 = poses[first]
        for second in ids[index + 1:]:
            x2, y2, radius2 = poses[second]
            if hypot(x1 - x2, y1 - y2) <= radius1 + radius2 + clearance_mm:
                conflicts.append((first, second))
    return conflicts


def load_scenario(path: str | Path) -> tuple[dict[str, Any], list[Piece], list[Zone]]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("rule_profile") != "senior_qualifier_draft":
        raise ValueError("This core implements the Senior qualifier draft only")
    pieces = [Piece(**piece) for piece in data["pieces"]]
    unique_pieces(pieces)
    zones = [Zone(**zone) for zone in data["destinations"]]
    return data, pieces, zones


def default_scenario_path() -> Path:
    """Prefer editable checkout config; use package data after wheel installation."""
    return default_data_path("qualifier_senior.json")


def run_mock(scenario_path: str | Path, *, fail_signal: str | None = None) -> dict[str, Any]:
    """Synthetic, serial state-machine rehearsal; no physical travel-time claim."""
    data, pieces, zones = load_scenario(scenario_path)
    inventory = unique_pieces(pieces)
    zone_map = {zone.id: zone for zone in zones}
    tasks = configured_tasks(pieces, zones, data.get("task_plan"))
    now_s = 0.0
    execution = []
    commands = []
    halted = False
    for task in tasks:
        machine = Manipulator(task, inventory[task.piece_id], zone_map[task.destination_id],
                              start_s=now_s, allow_synthetic=True)
        while machine.phase not in {"done", "fault"}:
            commands.append(machine.command())
            now_s = min(MATCH_SECONDS, round(now_s + 0.1, 3))
            signals = {name: name != fail_signal for name in machine.required_signals}
            feedback = Feedback(now_s, machine.phase, signals, task.piece_id,
                                task.target_x_mm, task.target_y_mm, synthetic=True)
            machine.tick(now_s, feedback)
            inventory[task.piece_id] = machine.current_object
        execution.append({"task": asdict(task), "final_phase": machine.phase,
                          "fault": machine.fault, "events": machine.events})
        if machine.fault:
            halted = True
            break
    stop_commands = [
        {"robot_id": robot_id, "wheel_velocity_rad_s": [0.0] * 4, "servo_intent": "hold", "device_io": False}
        for robot_id in ROBOT_ROLES
    ]
    return {
        "mode": "explicit_synthetic_feedback_demo", "device_io": False,
        "notice": "State-machine rehearsal only: 0.1 s per synthetic confirmation is NOT a physical mission-time estimate.",
        "layout_status": data["layout_status"],
        "coordinate_frame": data["coordinate_frame"], "units": data["units"],
        "robots": data["ground_robots"],
        "observer": {**data["observer"], "flight_control_implemented": False},
        "allocated_tasks": [asdict(task) for task in tasks], "execution": execution,
        "command_intentions": commands, "stop_commands": stop_commands,
        "halted": halted, "final_pieces": [asdict(piece) for piece in inventory.values()],
        "score": score_senior(inventory.values(), zones, elapsed_s=now_s),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--fail-signal", help="Leave one mock sensor false, e.g. optical_present or gripper_clear")
    parser.add_argument("--compact", action="store_true", help="Print compact JSON")
    args = parser.parse_args()
    print(json.dumps(run_mock(args.config or default_scenario_path(), fail_signal=args.fail_signal), ensure_ascii=True, indent=None if args.compact else 2))


if __name__ == "__main__":
    main()
