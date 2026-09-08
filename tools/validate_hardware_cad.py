"""Validate the Hamster v0.9 and Beaver v0.7 hardware CAD exports."""
from __future__ import annotations

import hashlib
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import tempfile


REPO = Path(__file__).resolve().parents[1]
OPENSCAD = Path(r"C:\Program Files\OpenSCAD\openscad.com")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_triangles(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + count * 50 == len(data):
            triangles = []
            offset = 84
            for _ in range(count):
                values = struct.unpack_from("<12fH", data, offset)
                triangles.append((values[3:6], values[6:9], values[9:12]))
                offset += 50
            return triangles
    values = [tuple(map(float, match)) for match in re.findall(
        rb"vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)", data
    )]
    if not values or len(values) % 3:
        raise AssertionError(f"Invalid or empty STL: {path}")
    return [tuple(values[index:index + 3]) for index in range(0, len(values), 3)]


def cross(a: tuple[float, float, float], b: tuple[float, float, float]):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def dot(a: tuple[float, float, float], b: tuple[float, float, float]):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def mesh_properties(path: Path) -> dict:
    triangles = read_triangles(path)
    vertices: dict[tuple[float, float, float], int] = {}
    parent: list[int] = []
    edge_counts: dict[tuple[int, int], int] = {}
    signed_volume = 0.0
    centroid_sum = [0.0, 0.0, 0.0]

    def vertex_index(vertex: tuple[float, float, float]) -> int:
        if vertex not in vertices:
            vertices[vertex] = len(parent)
            parent.append(len(parent))
        return vertices[vertex]

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(a: int, b: int) -> None:
        root_a, root_b = find(a), find(b)
        if root_a != root_b:
            parent[root_a] = root_b

    for triangle in triangles:
        indices = [vertex_index(vertex) for vertex in triangle]
        for a, b in ((indices[0], indices[1]),
                     (indices[1], indices[2]),
                     (indices[2], indices[0])):
            edge = tuple(sorted((a, b)))
            edge_counts[edge] = edge_counts.get(edge, 0) + 1
            union(a, b)
        a, b, c = triangle
        volume = dot(a, cross(b, c)) / 6.0
        signed_volume += volume
        for axis in range(3):
            centroid_sum[axis] += volume * (a[axis] + b[axis] + c[axis]) / 4.0

    points = list(vertices)
    bounds_min = [min(point[axis] for point in points) for axis in range(3)]
    bounds_max = [max(point[axis] for point in points) for axis in range(3)]
    components = len({find(index) for index in range(len(parent))})
    assert abs(signed_volume) > 1e-7, f"Zero-volume STL: {path.name}"
    return {
        "file": path.name,
        "size_mm": [round(bounds_max[i] - bounds_min[i], 4) for i in range(3)],
        "bounds_mm": [[round(value, 4) for value in bounds_min],
                      [round(value, 4) for value in bounds_max]],
        "volume_mm3_each": round(abs(signed_volume), 3),
        "centroid_mm": [round(value / signed_volume, 4) for value in centroid_sum],
        "watertight": all(count == 2 for count in edge_counts.values()),
        "connected_components": components,
        "bottom_z_mm": round(bounds_min[2], 6),
        "sha256": sha256(path),
    }


def validate_parts(folder: Path, quantities: dict[str, int], expected: int):
    records = []
    volume = 0.0
    pieces = 0
    for path in sorted(folder.glob("*.stl")):
        record = mesh_properties(path)
        quantity = quantities.get(path.name, 1)
        record["quantity"] = quantity
        assert record["watertight"], record
        assert record["connected_components"] == 1, record
        assert abs(record["bottom_z_mm"]) < 1e-4, record
        assert max(record["size_mm"]) <= 200.0, record
        records.append(record)
        volume += record["volume_mm3_each"] * quantity
        pieces += quantity
    assert len(records) == expected, (folder, len(records), expected)
    return records, volume, pieces


def assignment(source: str, name: str) -> str:
    match = re.search(rf"^{re.escape(name)}\s*=.*?;", source, re.MULTILINE)
    if not match:
        raise AssertionError(f"Missing assignment: {name}")
    return "".join(match.group(0).split())


def preserved(old_path: Path, new_path: Path, names: list[str]) -> dict[str, bool]:
    old, new = old_path.read_text(encoding="utf-8"), new_path.read_text(encoding="utf-8")
    result = {name: assignment(old, name) == assignment(new, name) for name in names}
    assert all(result.values()), result
    return result


def point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > y) != (yj > y):
            boundary_x = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < boundary_x:
                inside = not inside
        j = i
    return inside


def coplanar_components(path: Path, tolerance: float = 1e-3) -> bool:
    """Return true when every connected STL component has zero solid thickness."""
    triangles = read_triangles(path)
    vertices: dict[tuple[float, float, float], int] = {}
    points: list[tuple[float, float, float]] = []
    parent: list[int] = []

    def index(point):
        if point not in vertices:
            vertices[point] = len(points)
            points.append(point)
            parent.append(len(parent))
        return vertices[point]

    def find(item):
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[a] = b

    for triangle in triangles:
        ids = [index(point) for point in triangle]
        union(ids[0], ids[1])
        union(ids[1], ids[2])

    groups: dict[int, list[tuple[float, float, float]]] = {}
    for item, point in enumerate(points):
        groups.setdefault(find(item), []).append(point)

    for group in groups.values():
        origin = group[0]
        vectors = [(point[0] - origin[0], point[1] - origin[1], point[2] - origin[2])
                   for point in group[1:]]
        first = next((vector for vector in vectors if dot(vector, vector) > tolerance ** 2), None)
        if first is None:
            continue
        normal = None
        for vector in vectors:
            candidate = cross(first, vector)
            if dot(candidate, candidate) > tolerance ** 2:
                normal = candidate
                break
        if normal is None:
            continue
        normal_length = math.sqrt(dot(normal, normal))
        if any(abs(dot(vector, normal)) / normal_length > tolerance for vector in vectors):
            return False
    return True


def empty_intersection(scad: Path, defines: dict[str, str], name: str) -> bool:
    with tempfile.TemporaryDirectory(prefix="robo-cad-check-") as temp_dir:
        output = Path(temp_dir) / f"{name}.stl"
        command = [str(OPENSCAD), "-o", str(output)]
        for key, value in defines.items():
            command.extend(["-D", f'{key}="{value}"'])
        command.append(str(scad))
        process = subprocess.run(command, capture_output=True, text=True, timeout=600)
        log = process.stdout + process.stderr
        empty = ("Current top level object is empty" in log and not output.exists())
        if output.exists():
            empty = coplanar_components(output)
        if not empty:
            raise AssertionError(f"Collision check failed ({name}):\n{log[-3000:]}")
        return True


def check_kinematics() -> bool:
    source = REPO / "v0.9" / "validation" / "kinematics.scad"
    with tempfile.TemporaryDirectory(prefix="robo-cad-kinematics-") as temp_dir:
        output = Path(temp_dir) / "kinematics.csg"
        process = subprocess.run(
            [str(OPENSCAD), "-o", str(output), "-D", 'part_to_render="none"', str(source)],
            capture_output=True, text=True, timeout=180,
        )
        log = process.stdout + process.stderr
        assert process.returncode == 0 and "KINEMATICS V0.9 PASS" in log, log[-3000:]
    return True


def collision_suite(scad: Path, specs: dict[str, tuple[dict[str, str], str]]) -> dict[str, bool]:
    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = {
            name: executor.submit(empty_intersection, scad, defines, artifact_name)
            for name, (defines, artifact_name) in specs.items()
        }
        return {name: future.result() for name, future in futures.items()}


def hamster_report() -> dict:
    folder = REPO / "v0.9"
    source = folder / "hamster_print_ready_v0.9.scad"
    records, volume, pieces = validate_parts(
        folder / "stl_print", {"05_guide_cap_x2.stl": 2}, 13
    )
    combined = mesh_properties(folder / "hamster_v0.9_printed_structure_REVIEW_ONLY.stl")
    support_polygon = [(0, -40), (60.45, -15), (47.25, 34),
                       (-47.25, 34), (-60.45, -15)]
    cx, cy, _cz = combined["centroid_mm"]
    capture_parameters = preserved(
        REPO / "v0.8" / "hamster_print_ready_v0.8.scad", source,
        ["sample_diameter", "sample_thickness", "sample_radial_clearance",
         "barrier_inner_radius", "rear_barrier_arc", "rear_left_center_angle",
         "rear_right_center_angle", "front_gate_arc", "front_gate_height",
         "front_gate_thickness", "front_gate_up_bottom_z", "funnel_front_width",
         "funnel_exit_width", "funnel_length", "funnel_wall_height",
         "funnel_wall_thickness", "funnel_tip_radius", "funnel_curve_strength"],
    )
    collision_scad = folder / "validation" / "collision_checks.scad"
    checks = collision_suite(collision_scad, {
        "gate_up_fixed_parts_empty_intersection":
            ({"part_to_render": "none", "check": "gate_endpoint",
              "front_gate_state": "UP"}, "hamster_gate_up"),
        "gate_down_fixed_parts_empty_intersection":
            ({"part_to_render": "none", "check": "gate_endpoint",
              "front_gate_state": "DOWN"}, "hamster_gate_down"),
        "offset_disk_to_skids_empty_intersection":
            ({"part_to_render": "none", "check": "offset_disk_to_skids"},
             "hamster_disk_skids"),
        "wheels_to_prints_empty_intersection":
            ({"part_to_render": "none", "check": "wheels_to_prints"},
             "hamster_wheels"),
        "motors_to_prints_empty_intersection":
            ({"part_to_render": "none", "check": "motors_to_prints"},
             "hamster_motors"),
        "battery_to_prints_empty_intersection":
            ({"part_to_render": "none", "check": "battery_to_prints"},
             "hamster_battery"),
        "battery_to_motors_empty_intersection":
            ({"part_to_render": "none", "check": "battery_to_motors"},
             "hamster_battery_motors"),
    })
    checks.update({
        "kinematics_179_samples_monotonic": check_kinematics(),
        "protected_capture_geometry_preserved": all(capture_parameters.values()),
        "all_print_stls_watertight_single_component_on_bed": True,
    })
    assert point_in_polygon((cx, cy), support_polygon)
    return {
        "version": "hamster-v0.9-hardware",
        "result": "PROTOTYPE_READY_AFTER_DELIVERY_FIT_CHECK",
        "source_sha256": sha256(source),
        "hardware": {
            "motor_body_mm": [33, 12, 10], "motor_shaft_mm": [3, 9],
            "wheel_mm": [43, 17.5], "shaft_insertion_mm": 8,
            "battery_nominal_body_mm": [56, 28, 13],
            "battery_clearance_each_side_mm": 1.0,
        },
        "clearances_mm": {
            "motor_to_barrier_vertical": 6.2, "battery_to_barrier_vertical": 5.2,
            "battery_to_motor_lateral": 3.7, "battery_to_gate": 2.0,
            "battery_to_cradle_lateral": 0.6, "battery_to_caster_vertical": 0.5,
            "battery_to_deck_vertical": 14.0,
        },
        "complete_envelope_mm": [138.4, 112, 100],
        "printed_structure": combined,
        "support_polygon_xy_mm": support_polygon,
        "printed_centroid_inside_support_polygon": True,
        "checks": checks,
        "protected_capture_parameters": capture_parameters,
        "printable_stls": records,
        "printed_part_types": len(records), "printed_piece_count": pieces,
        "approx_printed_volume_mm3": round(volume, 2),
        "approx_pla_mass_g_at_1_24g_cm3": round(volume / 1000 * 1.24, 2),
        "remaining_real_tests": [
            "Measure the delivered battery body, wrapping, plugs and wire bend radius before full production.",
            "Print one N20 mount first and verify body fit, D-flat orientation and 8 mm hub engagement.",
            "Verify loaded turning current and measure the complete assembled centre of mass.",
        ],
    }


def beaver_report() -> dict:
    folder = REPO / "beaver_v0.7_hardware"
    source = folder / "beaver_robot_v0.7_hardware.scad"
    records, volume, pieces = validate_parts(
        folder / "stl_print", {"05_link_x2_M25_CLEARANCE.stl": 2}, 13
    )
    combined = mesh_properties(folder / "beaver_v0.7_printed_structure_REVIEW_ONLY.stl")
    support_polygon = [(-43.5, -43), (43.5, -43), (57.75, -23),
                       (43.5, 16), (-43.5, 16), (-57.75, -23)]
    cx, cy, _cz = combined["centroid_mm"]
    gripper_parameters = preserved(
        REPO / "beaver_v0.6_physical" / "beaver_robot_v0.6_physical.scad", source,
        ["patient_diameter", "patient_height", "radial_clearance", "patient_center",
         "jaw_inner_radius", "jaw_outer_radius", "jaw_arc_start", "jaw_arc_end",
         "jaw_pivot", "jaw_open_angle", "jaw_close_angle", "jaw_tip_rounding_r",
         "jaw_lever_vector", "link_length", "yoke_link_x", "collar_outer_radius",
         "collar_start_angle", "collar_end_angle"],
    )
    collision_scad = folder / "validation" / "collision_checks.scad"
    checks = collision_suite(collision_scad, {
        "housing_close_empty_intersection":
            ({"part": "none", "check": "housing_pose", "front_test_state": "CLOSE",
              "gate_test_angle": "0"}, "beaver_housing_close"),
        "housing_open_empty_intersection":
            ({"part": "none", "check": "housing_pose", "front_test_state": "OPEN",
              "gate_test_angle": "105"}, "beaver_housing_open"),
        "wheel_to_prints_empty_intersection":
            ({"part": "none", "check": "wheel_to_prints"}, "beaver_wheels"),
        "patient_to_jaws_empty_intersection":
            ({"part": "none", "check": "patient_to_jaws"}, "beaver_patient_jaws"),
        "patient_to_skids_empty_intersection":
            ({"part": "none", "check": "patient_to_skids"}, "beaver_patient_skids"),
        "boxes_to_magazine_zero_volume_contact":
            ({"part": "none", "check": "boxes_to_magazine"}, "beaver_boxes"),
        "motors_to_prints_empty_intersection":
            ({"part": "none", "check": "motor_to_prints"}, "beaver_motors"),
        "battery_to_prints_empty_intersection":
            ({"part": "none", "check": "battery_to_prints"}, "beaver_battery"),
        "battery_to_motors_empty_intersection":
            ({"part": "none", "check": "battery_to_motors"},
             "beaver_battery_motors"),
    })
    checks.update({
        "protected_front_gripper_geometry_preserved": all(gripper_parameters.values()),
        "all_print_stls_watertight_single_component_on_bed": True,
    })
    assert point_in_polygon((cx, cy), support_polygon)
    return {
        "version": "beaver-v0.7-hardware",
        "result": "PROTOTYPE_READY_AFTER_DELIVERY_FIT_CHECK",
        "source_sha256": sha256(source),
        "hardware": {
            "motor_body_mm": [33, 12, 10], "motor_shaft_mm": [3, 9],
            "wheel_mm": [43, 17.5], "shaft_insertion_mm": 8,
            "battery_nominal_body_mm": [56, 28, 13],
            "battery_clearance_each_side_mm": 0.8,
        },
        "clearances_mm": {
            "wheel_to_chassis_axial": 2.0, "wheel_to_housing_axial": 13.0,
            "motor_body_pass_through": 0.6, "battery_to_motor_vertical": 0.3,
            "battery_to_motor_mount_lateral": 1.6,
        },
        "complete_closed_envelope_mm": [133, 98.2875, 94],
        "gate_open_y_envelope_mm": 144.92,
        "printed_structure": combined,
        "support_polygon_xy_mm": support_polygon,
        "printed_centroid_inside_support_polygon": True,
        "checks": checks,
        "protected_front_gripper_parameters": gripper_parameters,
        "printable_stls": records,
        "printed_part_types": len(records), "printed_piece_count": pieces,
        "approx_printed_volume_mm3": round(volume, 2),
        "approx_pla_mass_g_at_1_24g_cm3": round(volume / 1000 * 1.24, 2),
        "remaining_real_tests": [
            "Measure the delivered battery body, wrapping, plugs and wire bend radius before full production.",
            "Print one N20 mount first and verify body fit, D-flat orientation and 8 mm hub engagement.",
            "Run at least 20 loaded dump cycles and verify loaded turning current.",
            "Measure the complete assembled centre of mass with purchased hardware and payload installed.",
        ],
    }


def main() -> None:
    assert OPENSCAD.exists(), f"OpenSCAD not found: {OPENSCAD}"
    hamster = hamster_report()
    beaver = beaver_report()
    (REPO / "v0.9" / "verification_v0.9.json").write_text(
        json.dumps(hamster, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (REPO / "beaver_v0.7_hardware" / "verification_beaver_v0.7_hardware.json").write_text(
        json.dumps(beaver, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "hamster": {"result": hamster["result"], "checks": hamster["checks"],
                    "mass_g": hamster["approx_pla_mass_g_at_1_24g_cm3"]},
        "beaver": {"result": beaver["result"], "checks": beaver["checks"],
                   "mass_g": beaver["approx_pla_mass_g_at_1_24g_cm3"]},
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
