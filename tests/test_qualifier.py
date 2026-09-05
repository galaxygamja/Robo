from __future__ import annotations

import json
import math
import shutil
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from robo_control.qualifier import (
    Feedback, Manipulator, Piece, Task, Zone, allocate_tasks, configured_tasks, default_scenario_path, ground_conflicts,
    load_scenario, run_mock, score_senior,
)

CONFIG = Path(__file__).resolve().parents[1] / "config" / "qualifier_senior.json"


class ScoringTests(unittest.TestCase):
    def setUp(self) -> None:
        _, self.pieces, self.zones = load_scenario(CONFIG)
        self.zone_map = {zone.id: zone for zone in self.zones}
        self.final = [Piece(**piece) for piece in run_mock(CONFIG)["final_pieces"]]

    def score(self, pieces: list[Piece]) -> dict:
        return score_senior(pieces, self.zones, elapsed_s=120)

    def moved(self, pieces: list[Piece], piece_id: str, zone_id: str, **changes) -> list[Piece]:
        x, y = self.zone_map[zone_id].center
        changes.setdefault("released", True)
        changes.setdefault("held_by", None)
        return [replace(piece, x_mm=x, y_mm=y, **changes) if piece.id == piece_id else piece for piece in pieces]

    def test_initial_inventory_is_zero_and_completed_mock_is_160(self) -> None:
        self.assertEqual(self.score(self.pieces)["total"], 0)
        score = self.score(self.final)
        self.assertEqual(score["points"], {"discs": 30, "cubes": 40, "red": 30, "yellow": 30, "green": 30})
        self.assertEqual(score["total"], 160)

    def test_duplicate_physical_id_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "Duplicate physical"):
            self.score(self.final + [self.final[0]])

    def test_cylinder_wrong_colour_invalidates_only_its_destination_cylinders(self) -> None:
        score = self.score(self.moved(self.final, "G4", "H"))
        self.assertEqual(score["contaminated_destinations"], ["H"])
        self.assertEqual(score["points"]["red"], 0)
        self.assertEqual(score["points"]["cubes"], 40)
        self.assertEqual(score["points"]["green"], 30)
        self.assertEqual(score["total"], 130)

    def test_contaminated_pcc_loses_its_yellow_points_but_split_is_physical(self) -> None:
        score = self.score(self.moved(self.final, "R4", "PCC-L"))
        self.assertTrue(score["yellow_split_satisfied"])
        self.assertEqual(score["points"]["yellow"], 20)
        self.assertEqual(score["points"]["cubes"], 40)

    def test_senior_yellows_all_in_one_pcc_score_zero(self) -> None:
        score = self.score(self.moved(self.moved(self.final, "Y2", "PCC-L"), "Y3", "PCC-L"))
        self.assertFalse(score["yellow_split_satisfied"])
        self.assertEqual(score["points"]["yellow"], 0)

    def test_four_cylinders_each_colour_are_capped_to_three(self) -> None:
        final = self.moved(self.final, "R4", "H")
        final = self.moved(final, "Y4", "PCC-R")
        final = self.moved(final, "G4", "RZ")
        self.assertEqual(self.score(final)["total"], 160)

    def test_held_or_attached_objects_never_score(self) -> None:
        final = [replace(piece, released=False) if piece.id == "D1" else piece for piece in self.final]
        final = [replace(piece, held_by="B1") if piece.id == "C1" else piece for piece in final]
        score = self.score(final)
        self.assertEqual(score["points"]["discs"], 20)
        self.assertEqual(score["points"]["cubes"], 30)
        self.assertEqual(score["ignored_objects"]["C1"], "not_released")

    def test_exact_boundary_contact_is_excluded(self) -> None:
        hospital = self.zone_map["H"]
        final = [replace(piece, x_mm=hospital.x_mm + 10) if piece.id == "R1" else piece for piece in self.final]
        self.assertEqual(self.score(final)["points"]["red"], 20)

    def test_disc_requires_separate_slot_and_strict_two_mm_margin(self) -> None:
        final = self.moved(self.final, "D2", "LAB-1")
        self.assertEqual(self.score(final)["points"]["discs"], 20)
        lab = self.zone_map["LAB-1"]
        self.assertTrue(lab.contains(Piece("sample", "disc", lab.x_mm + 1.9, lab.y_mm)))
        self.assertFalse(lab.contains(Piece("sample", "disc", lab.x_mm + 2, lab.y_mm)))

    def test_rotated_cube_footprint_is_considered(self) -> None:
        hospital = self.zone_map["H"]
        cube = Piece("rotated", "cube", hospital.x_mm + 13, hospital.y_mm + 50)
        self.assertTrue(hospital.contains(cube))
        self.assertFalse(hospital.contains(replace(cube, yaw_rad=math.pi / 4)))

    def test_cube_destination_capacity_h2_and_each_pcc1(self) -> None:
        final = self.moved(self.final, "C4", "H")
        self.assertEqual(self.score(final)["points"]["cubes"], 30)

    def test_final_state_recomputed_and_not_previous_task_count(self) -> None:
        self.assertEqual(self.score(self.final)["total"], 160)
        final = [replace(piece, x_mm=500, y_mm=500) if piece.id == "D1" else piece for piece in self.final]
        self.assertEqual(self.score(final)["total"], 150)

    def test_post_deadline_snapshot_and_nonfinite_geometry_rejected(self) -> None:
        with self.assertRaises(ValueError):
            score_senior(self.final, self.zones, elapsed_s=120.001)
        with self.assertRaises(ValueError):
            Piece("bad", "disc", math.nan, 0)

    def test_release_confirmation_cannot_be_truthy_json_string(self) -> None:
        with self.assertRaises(ValueError):
            Piece("bad", "disc", 500, 500, released="false")


class AllocationTests(unittest.TestCase):
    def test_explicit_scene_plan_uses_web_assignments_and_positions(self) -> None:
        data, pieces, zones = load_scenario(CONFIG)
        tasks = {task.piece_id: task for task in configured_tasks(pieces, zones, data["task_plan"])}
        self.assertEqual((tasks["D3"].robot_id, tasks["D3"].target_x_mm, tasks["D3"].target_y_mm), ("H1", 580, 75))
        self.assertEqual((tasks["Y2"].robot_id, tasks["Y2"].destination_id), ("B2", "PCC-R"))
        bad = [dict(entry) for entry in data["task_plan"]]
        bad[0]["robot_id"] = "B1"
        with self.assertRaises(ValueError):
            configured_tasks(pieces, zones, bad)

    def test_four_ground_roles_and_three_unused_cylinders(self) -> None:
        _, pieces, zones = load_scenario(CONFIG)
        tasks = allocate_tasks(pieces, zones)
        objects = {piece.id: piece for piece in pieces}
        self.assertEqual(len(tasks), 16)
        self.assertEqual(len({task.piece_id for task in tasks}), 16)
        self.assertEqual({task.robot_id for task in tasks}, {"H1", "H2", "B1", "B2"})
        for task in tasks:
            self.assertEqual(task.robot_id.startswith("H"), objects[task.piece_id].kind == "disc")
            if objects[task.piece_id].kind == "cube":
                self.assertEqual(task.robot_id, objects[task.piece_id].held_by)
        unassigned = [piece for piece in pieces if piece.id not in {task.piece_id for task in tasks}]
        self.assertEqual({piece.colour for piece in unassigned}, {"red", "yellow", "green"})

    def test_loaded_items_cannot_switch_to_incompatible_robot(self) -> None:
        _, pieces, zones = load_scenario(CONFIG)
        pieces[0] = replace(pieces[0], held_by="B1", released=False)
        with self.assertRaises(ValueError):
            allocate_tasks(pieces, zones)

    def test_all_ground_pairs_including_hamster_beaver_are_checked(self) -> None:
        self.assertEqual(ground_conflicts({"H1": (100, 100, 50), "B1": (190, 100, 50)}), [("B1", "H1")])
        self.assertEqual(ground_conflicts({"B1": (100, 100, 50), "B2": (220, 100, 50)}), [])
        with self.assertRaises(ValueError):
            ground_conflicts({"BAT": (100, 100, 50)})


class ManipulatorTests(unittest.TestCase):
    def machine(self, *, kind="disc", synthetic=False, start=0.0) -> Manipulator:
        if kind == "disc":
            piece = Piece("sample", kind, 10, 10)
            zone = Zone("LAB-1", "circle", 500, 500, radius_mm=30)
            robot = "H1"
        else:
            piece = Piece("object", kind, 10, 10, colour="red" if kind == "cylinder" else None,
                          held_by="B1" if kind == "cube" else None, released=kind != "cube")
            zone = Zone("H", "rect", 450, 450, width_mm=100, height_mm=100)
            robot = "B1"
        task = Task("task", piece.id, robot, zone.id, 500, 500)
        return Manipulator(task, piece, zone, start_s=start, allow_synthetic=synthetic)

    def confirm(self, machine: Manipulator, **kwargs) -> None:
        now = round(machine.last_tick_s + 0.1, 3)
        feedback = Feedback(now, machine.phase, {key: True for key in machine.required_signals},
                            machine.piece.id, 500, 500, **kwargs)
        machine.tick(now, feedback)

    def advance_to(self, machine: Manipulator, phase: str) -> None:
        for _ in range(12):
            if machine.phase == phase:
                return
            self.confirm(machine)
        self.fail(f"Did not reach {phase}; at {machine.phase}")

    def test_disc_success_requires_optical_grip_and_confirmed_release_pose(self) -> None:
        machine = self.machine()
        self.advance_to(machine, "confirm_grip")
        self.assertEqual(machine.required_signals, ("optical_present",))
        self.confirm(machine)
        self.assertFalse(machine.current_object.free)
        self.advance_to(machine, "confirm_clear")
        self.assertIsNone(machine.released_object)
        self.confirm(machine)
        self.assertIsNotNone(machine.released_object)
        self.assertTrue(machine.current_object.free)
        self.assertEqual(machine.command()["motion_intent"], "retreat_from_released_piece")
        self.assertEqual(machine.command()["retreat_distance_mm"], 110)
        self.confirm(machine)
        self.assertEqual(machine.phase, "done")

    def test_beaver_cylinder_uses_gripper_sensor(self) -> None:
        machine = self.machine(kind="cylinder")
        self.advance_to(machine, "confirm_grip")
        self.assertEqual(machine.required_signals, ("gripper_present",))

    def test_preloaded_cube_uses_hopper_not_cylinder_gripper(self) -> None:
        machine = self.machine(kind="cube")
        self.assertEqual(machine.phase, "confirm_load")
        self.assertEqual(machine.required_signals, ("hopper_loaded",))
        self.advance_to(machine, "release_servo")
        self.assertEqual(machine.command()["servo_intent"], "hopper_gate_open_one")

    def test_missing_sensor_times_out_and_fault_remains_latched(self) -> None:
        machine = self.machine()
        self.advance_to(machine, "confirm_grip")
        command = machine.tick(machine.entered_at_s + 4.0)
        self.assertEqual(machine.phase, "fault")
        self.assertEqual(command["wheel_velocity_rad_s"], [0.0] * 4)
        self.assertEqual(command["servo_intent"], "hold")
        self.confirm(machine)
        self.assertEqual(machine.phase, "fault")
        self.assertIsNone(machine.released_object)

    def test_stale_future_and_wrong_phase_feedback_cannot_advance(self) -> None:
        machine = self.machine()
        for feedback in (Feedback(-1, "approach", {"pickup_reached": True}),
                         Feedback(2, "approach", {"pickup_reached": True}),
                         Feedback(1, "confirm_grip", {"pickup_reached": True})):
            machine.tick(1, feedback)
            self.assertEqual(machine.phase, "approach")

    def test_servo_open_alone_never_counts_as_release(self) -> None:
        machine = self.machine()
        self.advance_to(machine, "confirm_clear")
        now = machine.last_tick_s + 0.1
        machine.tick(now, Feedback(now, "confirm_clear", {"servo_open": True}, machine.piece.id, 500, 500))
        self.assertEqual(machine.phase, "confirm_clear")
        self.assertIsNone(machine.released_object)

    def test_clear_sensor_without_position_or_wrong_piece_cannot_pass(self) -> None:
        machine = self.machine()
        self.advance_to(machine, "confirm_clear")
        signals = {key: True for key in machine.required_signals}
        now = machine.last_tick_s + 0.1
        machine.tick(now, Feedback(now, "confirm_clear", signals))
        self.assertEqual(machine.phase, "confirm_clear")
        now += 0.1
        machine.tick(now, Feedback(now, "confirm_clear", signals, "wrong-piece", 500, 500))
        self.assertEqual(machine.phase, "confirm_clear")
        now += 0.1
        machine.tick(now, Feedback(now, "confirm_clear", signals, machine.piece.id, 503, 500))
        self.assertIsNone(machine.released_object)

    def test_deadline_stops_before_accepting_new_sensor_success(self) -> None:
        machine = self.machine(start=119.9)
        machine.tick(120, Feedback(120, "approach", {"pickup_reached": True}))
        self.assertEqual(machine.fault, "match_timeout")
        self.assertEqual(machine.command()["wheel_velocity_rad_s"], [0.0] * 4)

    def test_synthetic_feedback_requires_explicit_constructor_enable(self) -> None:
        machine = self.machine()
        self.confirm(machine, synthetic=True)
        self.assertEqual(machine.fault, "synthetic_feedback_not_enabled")
        mock = self.machine(synthetic=True)
        self.confirm(mock, synthetic=True)
        self.assertEqual(mock.phase, "align_pickup")

    def test_monotonic_clock_enforced(self) -> None:
        machine = self.machine()
        machine.tick(1)
        with self.assertRaises(ValueError):
            machine.tick(0.9)


class CommandLineTests(unittest.TestCase):
    def test_packaged_scene_equals_editable_source(self) -> None:
        packaged = CONFIG.parent.parent / "robo_control" / "data" / CONFIG.name
        self.assertEqual(CONFIG.read_bytes(), packaged.read_bytes())
        self.assertEqual(default_scenario_path(), CONFIG)

    def test_installed_layout_cli_falls_back_to_packaged_scene(self) -> None:
        # An isolated package directory has no repository config/ or pyproject.
        with tempfile.TemporaryDirectory() as folder:
            shutil.copytree(CONFIG.parent.parent / "robo_control", Path(folder) / "robo_control",
                            ignore=shutil.ignore_patterns("__pycache__"))
            result = subprocess.run([sys.executable, "-m", "robo_control.qualifier", "--compact"],
                                    cwd=folder, check=True, capture_output=True, text=True)
            report = json.loads(result.stdout)
            self.assertEqual(report["score"]["total"], 160)
            self.assertEqual(report["coordinate_frame"]["origin"], "bottom_left")

    def test_cli_produces_json_and_explicit_mock_label(self) -> None:
        result = subprocess.run([sys.executable, "-m", "robo_control.qualifier", "--compact"],
                                check=True, capture_output=True, text=True)
        report = json.loads(result.stdout)
        self.assertEqual(report["mode"], "explicit_synthetic_feedback_demo")
        self.assertFalse(report["device_io"])
        self.assertEqual(len(report["robots"]), 4)
        self.assertEqual(report["observer"]["id"], "BAT")
        self.assertEqual(report["score"]["total"], 160)

    def test_fault_injection_halts_all_ground_intents_without_free_score(self) -> None:
        report = run_mock(CONFIG, fail_signal="optical_present")
        self.assertTrue(report["halted"])
        self.assertEqual(report["score"]["total"], 0)
        self.assertEqual(len(report["stop_commands"]), 4)
        self.assertTrue(all(command["wheel_velocity_rad_s"] == [0.0] * 4 for command in report["stop_commands"]))


if __name__ == "__main__":
    unittest.main()
