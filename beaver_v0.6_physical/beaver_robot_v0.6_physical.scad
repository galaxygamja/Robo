/*
  Beaver Robot v0.6-physical - stability and tolerance revision
  Units: millimetres. +Y is forward, Z=0 is the arena floor.

  The v0.4 integrated chassis, front gripper, wheels and cargo dumper are
  retained. This revision adds controlled front/rear anti-tip contacts, larger
  running clearances, a steeper assisted cargo chute and stronger housing
  bosses. The front jaw arcs, pivots and linkage geometry are unchanged.
*/

$fn = 64;
part = "physical_complete_close";
// Select: physical_complete_close, physical_gate_open,
// housing_complete_close, housing_gate_open, housing_only,
// housing_transparent, housing_motion_overlay, upper_housing,
// integrated_complete_close, integrated_gate_open,
// integrated_body_only, integrated_motion_overlay, integrated_top,
// integrated_chassis, wheel_reference,
// Legacy v0.3 selectors retained below for comparison only:
// frame_combined_close, frame_gate_open, frame_motion_overlay, frame_cam_detail,
// frame_only, outer_side_frame, frame_crossbar, frame_foot_bracket,
// cargo_loaded_1, cargo_loaded_2, cargo_combined_close, cargo_gate_open,
// cargo_motion_overlay, mounting_base_v02, cargo_magazine,
// rear_dump_gate, dump_servo_mount, box_reference,
// assembly_close, assembly_open, motion_overlay, mounting_base,
// jaw_left, jaw_right, slider_yoke, link, servo_mount, patient_reference

// ---------------- Patient and clearance ----------------
patient_diameter = 30;
patient_height = 20;
patient_radius = patient_diameter / 2;
radial_clearance = 1.5;          // easy v0.2 tuning: intended range 1..2
pocket_radius = patient_radius + radial_clearance;
patient_center = [0, 22];

// ---------------- Robot envelope ----------------
limit_x = 100;
limit_y = 100;
limit_z = 100;
base_half_width = 48;
base_rear_y = -48;
base_front_y = 24;
base_thickness = 3;

// Hamster-derived four-point mounting pattern.
hamster_mount_x = 44;
hamster_mount_front_y = 17;
hamster_mount_rear_y = -38;
hamster_mount_hole_d = 3.4;

// ---------------- Jaws ----------------
jaw_inner_radius = pocket_radius;
jaw_outer_radius = 21;
jaw_arc_start = 0;
jaw_arc_end = 75;
jaw_arc_steps = 18;
jaw_bottom_z = 3.6;
jaw_height = 15;
jaw_pivot = [30, 10];
jaw_pivot_boss_r = 5;
jaw_pivot_hole_d = 3.5;          // M3 clearance plus printed-fit allowance
jaw_open_angle = 26;
jaw_close_angle = 0;
jaw_tip_rounding_r = 2.2;

// Link pin boss is kept close enough to overlap the pivot boss.
jaw_lever_vector = [-8, 0];
link_pin_boss_d = 7.2;
link_pin_hole_d = 2.9;           // printable M2.5 running clearance
link_plane_z = 20.5;
link_thickness = 2.5;
link_width = 5;
link_length = 8;

// ---------------- Symmetric slider and servo crank ----------------
yoke_link_x = 22;
yoke_closed_y = jaw_pivot[1] - link_length;

function jaw_link_point_right(angle) = [
    jaw_pivot[0] + jaw_lever_vector[0] * cos(-angle)
                 - jaw_lever_vector[1] * sin(-angle),
    jaw_pivot[1] + jaw_lever_vector[0] * sin(-angle)
                 + jaw_lever_vector[1] * cos(-angle)
];

open_link_point = jaw_link_point_right(jaw_open_angle);
open_link_dx = open_link_point[0] - yoke_link_x;
yoke_open_y = open_link_point[1]
              - sqrt(link_length * link_length - open_link_dx * open_link_dx);
yoke_travel = yoke_open_y - yoke_closed_y;

servo_axis = [0, (yoke_closed_y + yoke_open_y) / 2, 21.5];
servo_crank_radius = 4;
servo_half_swing = asin(yoke_travel / (2 * servo_crank_radius));
crank_slot_center_x = servo_crank_radius * cos(servo_half_swing);
crank_slot_length = 3.2;
crank_slot_width = 3.3;

// Generic SG90-class reference only; measure the actual servo before printing.
servo_body_size = [23, 12.5, 22.5];
servo_body_center = [5.5, -1.5, 35.25];
servo_mount_wall = 2.5;
servo_body_clearance = 0.5;
servo_mount_bottom_z = base_thickness;

// Fixed rear collar plus moving jaws form a nearly closed horizontal pocket.
collar_outer_radius = 20.5;
collar_bottom_z = base_thickness;
collar_height = 15.5;
collar_steps = 24;
collar_start_angle = 205;
collar_end_angle = 335;

// ---------------- Box cargo add-on ----------------
box_size = [25,25,20];
box_side_clearance = 1.5;
cargo_inner_width = box_size[0] + 2*box_side_clearance; // 28 mm
cargo_inner_rear_y = -46;
cargo_inner_front_y = -14;
cargo_inner_length = cargo_inner_front_y - cargo_inner_rear_y;
cargo_wall = 2.4;                    // six 0.4 mm extrusion lines
cargo_deck_bottom_z = 40.5;          // rests on the v0.3 transverse frame bars
cargo_deck_thickness = 3;
cargo_floor_base_rise = 3;            // buries the hinge below the slide surface
cargo_floor_rear_z = cargo_deck_bottom_z + cargo_deck_thickness
                       + cargo_floor_base_rise;
cargo_slope_rise = 4.0;
cargo_slope_angle = atan(cargo_slope_rise/cargo_inner_length);
cargo_side_wall_top_z = 86.5;
cargo_front_wall_top_z = 94;
cargo_box_center_y = -28;
cargo_mount_hole_d = 3.4;
cargo_mount_points = [[-33.5,-42],[33.5,-42],[-33.5,-16],[33.5,-16]];
cargo_standoff_d = 5;

// The axis is 0.5 mm forward of the rear datum; the panel's inner face still
// closes exactly at cargo_inner_rear_y. The barrel sits flush below the floor.
gate_y = cargo_inner_rear_y + 0.5;
gate_axis_z = cargo_deck_bottom_z + cargo_deck_thickness - 0.5;
gate_width = 27;
gate_thickness = 3;
gate_panel_bottom = cargo_floor_rear_z - gate_axis_z - 0.4;
gate_height = cargo_front_wall_top_z - gate_axis_z - gate_panel_bottom;
gate_barrel_d = 7;
gate_axle_hole_d = 3.4;
gate_open_angle = 105;

// Gate-mounted cam. At CLOSE its top is flush just below the rear floor.
// Near 15 degrees it raises/pushes the box rear edge by about 1.5 mm,
// so discharge no longer depends on the 4.47 degree floor overcoming static
// friction on its own.
kick_cam_half_width = 11;
kick_cam_forward = 8;
kick_cam_running_clearance = 0.3;
kick_cam_local_z0 = 1.2;
kick_cam_local_top_rear = cargo_floor_rear_z
                          + cargo_slope_rise*(gate_y-cargo_inner_rear_y)
                            /cargo_inner_length
                          - gate_axis_z-kick_cam_running_clearance;
kick_cam_local_top_front = kick_cam_local_top_rear
                           + cargo_slope_rise*kick_cam_forward/cargo_inner_length;
kick_cam_pocket_extra = 0.55;

// ---------------- Removable external frame ----------------
frame_side_inner_x = 49;
frame_side_thickness = 3;
frame_bar_length = 2*frame_side_inner_x;
frame_bar_width = 8;
frame_bar_height = 4;
frame_crossbar_bottom_z = cargo_deck_bottom_z-frame_bar_height;
frame_top_bar_bottom_z = 92;
frame_side_member_width = 6;
frame_side_boss_d = 10;
frame_side_hole_d = 3.4;
frame_crossbar_pilot_d = 2.8;
frame_crossbar_pilot_depth = 12;
frame_foot_outer_r = 11;
frame_foot_plate_thickness = 3;
frame_foot_width_y = 10;
frame_foot_hole_d = 3.4;
frame_base_nodes_y = [hamster_mount_rear_y,hamster_mount_front_y];
frame_cargo_nodes_y = [-42,-16];
frame_top_nodes_y = [-49,-9];
frame_base_node_z = 9;
frame_cargo_node_z = frame_crossbar_bottom_z+frame_bar_height/2;
frame_top_node_z = frame_top_bar_bottom_z+frame_bar_height/2;

// ---------------- v0.4 compact integrated body ----------------
// The drive motor and battery remain part of the Hamster core, not separately
// retained by this add-on CAD. Electrical target: two 6 V brushed DC geared
// motors, 100..200 rpm loaded output speed, >=0.8 kgf.cm continuous torque and
// <=1.5 A stall current each. Battery target: protected 2S LiPo, 7.4 V nominal
// / 8.4 V full, 300..500 mAh, >=20 C and <=24x36x20 mm including wrapping and
// leads. Regulate the drive rail to 6 V at >=4 A and servo/control to 5 V at
// >=3 A. The actual core must retain both items; the translucent core reference
// below is only a 72x46x30 mm collision envelope.
chassis_ground_z = 1.2;
chassis_side_inner_x = 40;
chassis_side_thickness = 7;
chassis_side_outer_x = chassis_side_inner_x+chassis_side_thickness;
chassis_side_lower_z = chassis_ground_z+5;
chassis_side_upper_z = cargo_deck_bottom_z-5;
wheel_center_x = 42.5;
wheel_center_y = -23;
wheel_center_z = 17;
wheel_diameter = 34;
wheel_width = 9;
wheel_arch_clearance = 2.2;
wheel_arch_diameter = wheel_diameter+2*wheel_arch_clearance;
wheel_ground_clearance = wheel_center_z-wheel_diameter/2;
chassis_ground_clearance = chassis_ground_z;
cargo_shoulder_inner_x = 28;
cargo_shoulder_outer_x = 43;
cargo_shoulder_width_y = 10;
cargo_shoulder_bottom_z = cargo_deck_bottom_z-4;
front_module_z_shift = chassis_ground_z;

// v0.6 anti-tip contacts. The central rear dump path remains open between
// X=-14 and X=+14. Pads are deliberately short and accept PTFE/UHMW tape.
stability_skid_center_x = 43.5;
stability_skid_front_y = 16;
stability_skid_rear_y = -43;
stability_skid_length = 8;
stability_skid_width = 5;
stability_skid_contact_z = 0;
stability_skid_height = 1.6;
review_structural_cg_y = -11.45;

// ---------------- v0.5 removable rounded housing ----------------
housing_wall = 2.0;                 // five 0.4 mm perimeter lines
housing_outer_half_x = 36;
housing_rear_y = -40;
housing_front_y = 3;
// The integrated v0.4 side pods already guard the lower drive unit.  The
// removable part begins just above the Hamster-core keep-out, so it is a true
// upper cover and cannot be trapped between the core and the wheels.
housing_bottom_z = 34.7;
housing_top_z = 54.7;
housing_corner_r = 8;
housing_inner_half_x = housing_outer_half_x-housing_wall;
housing_inner_rear_y = housing_rear_y+housing_wall;
housing_inner_front_y = housing_front_y-housing_wall;
housing_inner_top_z = housing_top_z-housing_wall;
housing_front_window_half_x = 30;
housing_front_window_bottom_z = 7;
housing_rear_window_half_x = 30;
housing_rear_window_bottom_z = 7;
housing_cargo_open_front_y = -9.5;
housing_cargo_open_bottom_z = cargo_deck_bottom_z-1.5;
housing_mount_boss_inner_x = 35;
housing_mount_boss_outer_x = chassis_side_inner_x-0.5;
housing_mount_boss_d = 6.5;
housing_mount_pilot_d = 2.8;
housing_mount_clearance_d = 3.4;
housing_mount_yz = [[-33,38],[-2,38]];
housing_print_bottom_z = min(concat(
    [housing_bottom_z],
    [for (p=housing_mount_yz) p[1]-housing_mount_boss_d/2]));
function housing_mount_to_wheel_clearance(p) =
    sqrt((p[0]-wheel_center_y)*(p[0]-wheel_center_y)
         +(p[1]-wheel_center_z)*(p[1]-wheel_center_z))
    -housing_mount_boss_d/2-wheel_diameter/2;
housing_mount_wheel_clearances =
    [for (p=housing_mount_yz) housing_mount_to_wheel_clearance(p)];

dump_servo_axis = [cargo_inner_width/2 + cargo_wall + 3.2,
                   gate_y, gate_axis_z];
dump_servo_body_size = [12.5,23,22.5];
dump_servo_body_min = [dump_servo_axis[0]+0.8,
                       dump_servo_axis[1]+0.5,
                       cargo_deck_bottom_z+cargo_deck_thickness+2];

// Payload is deliberately just behind the nominal wheel axis Y=-24.
lower_box_center = [0,cargo_box_center_y,
    cargo_floor_rear_z
    + cargo_slope_rise*(cargo_box_center_y-cargo_inner_rear_y)/cargo_inner_length];
// True volume centroid of two stacked, floor-aligned boxes. The earlier
// centreline estimate omitted the rotation of each box about the floor datum.
payload_two_box_center_y = cargo_box_center_y
                           -box_size[2]*sin(cargo_slope_angle);
payload_two_box_center_z = lower_box_center[2]
                           +box_size[2]*cos(cargo_slope_angle);

// ---------------- Derived verification values ----------------
closed_tip_x = jaw_inner_radius * cos(jaw_arc_end);
closed_front_opening = 2 * closed_tip_x;
open_tip_right = [
    jaw_pivot[0] + (closed_tip_x - jaw_pivot[0]) * cos(-jaw_open_angle)
                 - (patient_center[1] + jaw_inner_radius * sin(jaw_arc_end)
                    - jaw_pivot[1]) * sin(-jaw_open_angle),
    jaw_pivot[1] + (closed_tip_x - jaw_pivot[0]) * sin(-jaw_open_angle)
                 + (patient_center[1] + jaw_inner_radius * sin(jaw_arc_end)
                    - jaw_pivot[1]) * cos(-jaw_open_angle)
];
open_entry_width = 2 * open_tip_right[0];

open_outer_tip_y = jaw_pivot[1]
    + (patient_center[0] + jaw_outer_radius * cos(jaw_arc_end) - jaw_pivot[0])
      * sin(-jaw_open_angle)
    + (patient_center[1] + jaw_outer_radius * sin(jaw_arc_end) - jaw_pivot[1])
      * cos(-jaw_open_angle);

// The rounded nose cap projects slightly beyond the raw outer-arc endpoint.
// Include it explicitly so the 100 mm assertion covers the real solid.
jaw_mid_radius = (jaw_inner_radius + jaw_outer_radius) / 2;
open_rounded_tip_center_y = jaw_pivot[1]
    + (patient_center[0] + jaw_mid_radius * cos(jaw_arc_end) - jaw_pivot[0])
      * sin(-jaw_open_angle)
    + (patient_center[1] + jaw_mid_radius * sin(jaw_arc_end) - jaw_pivot[1])
      * cos(-jaw_open_angle);
open_rounded_tip_max_y = open_rounded_tip_center_y + jaw_tip_rounding_r;

robot_min_x = -base_half_width;
robot_max_x = base_half_width;
robot_min_y = base_rear_y;
robot_max_y = max(open_outer_tip_y, open_rounded_tip_max_y);
robot_min_z = 0;
robot_max_z = servo_mount_bottom_z + 45.5;
robot_size = [robot_max_x - robot_min_x,
              robot_max_y - robot_min_y,
              robot_max_z - robot_min_z];

existing_servo_mount_rear_y = servo_body_center[1]-servo_body_size[1]/2
                              -servo_body_clearance-servo_mount_wall;
cargo_front_gap = existing_servo_mount_rear_y
                  -(cargo_inner_front_y+cargo_wall);
box_stack_max_z = lower_box_center[2]
                  + box_size[1]/2*sin(cargo_slope_angle)
                  + 2*box_size[2]*cos(cargo_slope_angle);
cargo_closed_min_y = min(base_rear_y,gate_y-4); // includes the D8 hinge boss
cargo_closed_size = [96,robot_max_y-cargo_closed_min_y,cargo_front_wall_top_z];
cargo_gate_open_corner_y = [
    for (yy=[-gate_barrel_d/2,-gate_barrel_d/2+gate_thickness])
        for (zz=[gate_panel_bottom,gate_panel_bottom+gate_height])
            gate_y + yy*cos(gate_open_angle) - zz*sin(gate_open_angle)
];
cargo_gate_open_min_y = min(cargo_closed_min_y,
                            min(cargo_gate_open_corner_y),gate_y-5.5);
cargo_gate_open_size_y = robot_max_y-min(base_rear_y,cargo_gate_open_min_y);

assert(patient_diameter == 30 && patient_height == 20,
       "Patient reference must remain 30 x 20 mm");
assert(radial_clearance >= 1 && radial_clearance <= 2,
       "Default radial clearance should remain within the requested 1..2 mm range");
assert(open_entry_width >= patient_diameter + 6,
       "OPEN entrance does not provide enough off-centre allowance");
assert(closed_front_opening < patient_diameter,
       "CLOSE front throat is wide enough for the patient to escape");
assert(open_link_dx * open_link_dx < link_length * link_length,
       "Link geometry has no real OPEN solution");
assert(yoke_travel > 3 && yoke_travel < 6,
       "Yoke travel is outside the intended compact range");
assert(link_pin_hole_d >= 2.8 && crank_slot_width >= 3.2,
       "Printed M2.5 joints do not have enough running clearance");
assert(2 * servo_half_swing < 90,
       "Servo swing is excessive");
assert(max(robot_size) <= 100,
       "Robot mechanism exceeds the 100 mm cube");
assert(cargo_inner_width-box_size[0] >= 3,
       "Box side clearance is below 1.5 mm per side");
assert(cargo_front_gap >= 0.8,
       "Cargo front wall interferes with the unchanged v0.1 servo mount");
assert(gate_width-box_size[0] >= 2,
       "Open rear gate is too narrow for the 25 mm box");
assert(cargo_front_wall_top_z-box_stack_max_z > 1,
       "Closed gate/front wall does not cover the two-box stack");
assert(payload_two_box_center_y < -24,
       "Two-box payload is not behind the nominal wheel axis");
assert(frame_side_inner_x-(42+4) >= 3,
       "External side frame is too close to the nominal wheel envelope");
assert(frame_crossbar_bottom_z-(base_thickness+32) >= 1.5,
       "Lower frame crossbar enters the drivetrain keep-out");
assert(frame_top_node_z+frame_side_boss_d/2 <= 100,
       "Protective frame exceeds the intended 100 mm height");
assert(kick_cam_local_top_front-kick_cam_local_z0 >= 2,
       "Kick cam is too thin for FDM printing");
assert(wheel_arch_clearance >= 2,
       "Wheel arch needs at least 2 mm radial running clearance");
assert(abs(wheel_ground_clearance) < 0.001,
       "Wheel contact plane must be Z=0");
assert(review_structural_cg_y > stability_skid_rear_y
       && review_structural_cg_y < stability_skid_front_y,
       "Reviewed empty-structure CG is outside the anti-tip support span");
assert(payload_two_box_center_y > stability_skid_rear_y
       && payload_two_box_center_y < stability_skid_front_y,
       "Two-box payload centre is outside the anti-tip support span");
assert(stability_skid_center_x+stability_skid_width/2
       <= chassis_side_outer_x,
       "Stability skid exceeds the integrated chassis width");
assert(cargo_shoulder_bottom_z-(wheel_center_z+wheel_diameter/2) >= 2,
       "Cargo shoulder enters the nominal wheel envelope");
assert(chassis_side_inner_x-(jaw_pivot[0]+jaw_pivot_boss_r) >= 5,
       "Compact body enters the unchanged jaw pivot envelope");
assert(chassis_side_inner_x
       -(dump_servo_body_min[0]+dump_servo_body_size[0]+2.2) > 4,
       "Compact body enters the dump-servo envelope");
assert(housing_wall >= 2,
       "Housing wall is below five 0.4 mm extrusion lines");
assert(housing_bottom_z-(front_module_z_shift+base_thickness+30) >= 0.5,
       "Housing lower edge enters the Hamster-core keep-out");
assert(housing_print_bottom_z
       -(front_module_z_shift+base_thickness+30) >= 0.5,
       "Housing mounting bosses enter the Hamster-core keep-out");
assert(wheel_center_x-wheel_width/2-housing_outer_half_x >= 2,
       "Housing is too close to the wheel inner faces");
assert(housing_inner_half_x-25.5 >= 8,
       "Housing side wall enters the unchanged front-servo bracket");
assert(housing_inner_top_z
       -(front_module_z_shift+servo_mount_bottom_z+45.5) >= 0.8,
       "Housing roof enters the unchanged front-servo bracket");
assert((patient_center[1]-patient_radius)-housing_front_y >= 4,
       "Housing front lip enters the patient/jaw approach region");
assert(housing_cargo_open_front_y-(frame_cargo_nodes_y[1]+5.2) >= 1.3,
       "Housing cargo-top opening enters the unchanged cargo deck");
assert(min(housing_mount_wheel_clearances) >= 1.5,
       "Housing mounting boss enters the nominal wheel envelope");

echo(str("VERIFY patient = D", patient_diameter, " x H", patient_height));
echo(str("VERIFY radial clearance = ", radial_clearance));
echo(str("VERIFY OPEN entrance width = ", open_entry_width));
echo(str("VERIFY CLOSE front throat = ", closed_front_opening));
echo(str("VERIFY yoke travel = ", yoke_travel));
echo(str("VERIFY servo total swing = ", 2 * servo_half_swing));
echo(str("VERIFY robot envelope = ", robot_size));
echo(str("CARGO box = ", box_size, ", capacity = 2"));
echo(str("CARGO internal channel = ", cargo_inner_width,
         " x ", cargo_inner_length));
echo(str("CARGO slope = ", cargo_slope_angle, " deg"));
echo(str("CARGO gravity-only static friction threshold = ",
         tan(cargo_slope_angle), " (kick cam provides initial release)"));
echo(str("CARGO two-box payload Y = ",payload_two_box_center_y));
echo(str("CARGO two-box payload Z = ",payload_two_box_center_z));
echo(str("CARGO stack max Z = ",box_stack_max_z));
echo(str("CARGO front mechanism gap = ",cargo_front_gap));
echo(str("CARGO CLOSED envelope = ",cargo_closed_size));
echo(str("CARGO dump-open Y envelope = ",cargo_gate_open_size_y));
echo(str("FRAME nominal closed envelope = ",
         [2*(hamster_mount_x+frame_foot_outer_r),
          max(robot_max_y,frame_top_nodes_y[1]+frame_side_boss_d/2)
            -min(frame_top_nodes_y[0]-frame_side_boss_d/2,base_rear_y),
          frame_top_node_z+frame_side_boss_d/2]));
echo(str("FRAME wheel lateral clearance = ",frame_side_inner_x-(42+4)));
echo(str("FRAME drivetrain vertical clearance = ",
         frame_crossbar_bottom_z-(base_thickness+32)));
echo(str("V0.6 wheel centres = +/-",wheel_center_x,", Y",wheel_center_y,
         ", Z",wheel_center_z));
echo(str("V0.6 wheel arch radial clearance = ",wheel_arch_clearance));
echo(str("V0.6 chassis ground margin below body = ",
         chassis_ground_clearance-wheel_ground_clearance));
echo(str("V0.6 anti-tip support Y span = ",stability_skid_rear_y,
         " to ",stability_skid_front_y,
         "; reviewed structure CG Y = ",review_structural_cg_y));
echo(str("V0.6 CLOSED envelope = ",
         [2*max(chassis_side_outer_x,hamster_mount_x+4.5),
          robot_max_y-min(gate_y-gate_barrel_d/2,-48),
          cargo_front_wall_top_z]));
echo(str("V0.6 housing main shell = ",
         [2*housing_outer_half_x,housing_front_y-housing_rear_y,
          housing_top_z-housing_bottom_z]));
echo(str("V0.6 housing including bosses = ",
         [2*housing_mount_boss_outer_x,
          housing_front_y-min(housing_rear_y,
                              housing_mount_yz[0][0]-housing_mount_boss_d/2),
          housing_top_z-housing_bottom_z]));
echo(str("V0.6 wheel axial clearance = ",
         wheel_center_x-wheel_width/2-housing_outer_half_x));
echo(str("V0.6 servo roof clearance = ",
         housing_inner_top_z
         -(front_module_z_shift+servo_mount_bottom_z+45.5)));

// ---------------- Geometry helpers ----------------
function arc_points(center, radius, a0, a1, steps) =
    [for (i = [0:steps])
        [center[0] + radius * cos(a0 + (a1-a0)*i/steps),
         center[1] + radius * sin(a0 + (a1-a0)*i/steps)]];

function rotate_point(point, center, angle) = [
    center[0] + (point[0]-center[0])*cos(angle)
              - (point[1]-center[1])*sin(angle),
    center[1] + (point[0]-center[0])*sin(angle)
              + (point[1]-center[1])*cos(angle)
];

module capsule_2d(p1, p2, width) {
    hull() {
        translate(p1) circle(d=width);
        translate(p2) circle(d=width);
    }
}

module ring_segment_2d(center, inner_r, outer_r, a0, a1, steps) {
    polygon(concat(
        arc_points(center, outer_r, a0, a1, steps),
        arc_points(center, inner_r, a1, a0, steps)
    ));
}

module hex_socket(af, h) {
    cylinder(d=af / cos(30), h=h, $fn=6);
}

// ---------------- Patient and keep-out references ----------------
module patient_reference(alpha=0.55) {
    color([0.95, 0.48, 0.12, alpha])
        translate([patient_center[0], patient_center[1], 0])
            cylinder(d=patient_diameter, h=patient_height);
}

module approach_patient_reference(offset_x=0, y=43, alpha=0.2) {
    color([1.0, 0.65, 0.15, alpha])
        translate([offset_x, y, 0])
            cylinder(d=patient_diameter, h=patient_height);
}

module drivetrain_keepout_reference() {
    color([0.2, 0.45, 0.85, 0.12])
        translate([-38, -45, base_thickness]) cube([76, 43, 32]);
    color([0.12, 0.12, 0.14, 0.2]) {
        for (sx=[-1,1])
            translate([sx*42, -24, 18])
                rotate([0,90,0]) cylinder(d=34, h=8, center=true);
    }
}

module servo_reference() {
    color([0.15,0.3,0.75,0.5])
        translate(servo_body_center - servo_body_size/2)
            cube(servo_body_size);
    color([0.92,0.92,0.95,0.65])
        translate(servo_axis) cylinder(d=5, h=3);
}

// ---------------- Mounting base and rear pocket ----------------
module rear_collar() {
    linear_extrude(height=collar_height)
        ring_segment_2d(patient_center, pocket_radius,
                        collar_outer_radius,
                        collar_start_angle, collar_end_angle, collar_steps);
}

module base_plan_2d() {
    union() {
        // Two hamster-derived longitudinal rails and a rear brace.
        translate([-48,-48]) square([8,72]);
        translate([40,-48]) square([8,72]);
        translate([-48,-48]) square([96,8]);

        // Front cheeks carry the jaw pivots while leaving the patient floor open.
        translate([-48,7]) square([30,17]);
        translate([18,7]) square([30,17]);
        for (sx=[-1,1]) translate([sx*jaw_pivot[0],jaw_pivot[1]]) circle(r=9);

        // Rear servo-mount bridge, kept behind the patient's rear tangent.
        translate([-24,-8]) square([48,11]);
    }
}

module mounting_base_assembly() {
    difference() {
        union() {
            linear_extrude(height=base_thickness) base_plan_2d();
            translate([0,0,collar_bottom_z]) rear_collar();

            // Tall outer fences guide the yoke without a captive printed channel.
            for (sx=[-1,1])
                translate([sx*26 - (sx<0 ? 2.5 : 0), -1.5, base_thickness])
                    cube([2.5, 12.5, link_plane_z-base_thickness+2.8]);
        }

        // Four M3 holes preserve the hamster v0.7 mounting pattern.
        for (x=[-hamster_mount_x,hamster_mount_x])
            for (y=[hamster_mount_front_y,hamster_mount_rear_y]) {
                translate([x,y,-0.1]) cylinder(d=hamster_mount_hole_d,h=base_thickness+0.2);
                translate([x,y,-0.01])
                    cylinder(d1=6.6,d2=hamster_mount_hole_d,h=1.7);
            }

        // M3 jaw pivots.
        for (sx=[-1,1]) {
            translate([sx*jaw_pivot[0],jaw_pivot[1],-0.1])
                cylinder(d=jaw_pivot_hole_d,h=base_thickness+0.2);
            translate([sx*jaw_pivot[0],jaw_pivot[1],-0.01])
                cylinder(d1=6.6,d2=jaw_pivot_hole_d,h=1.7);
        }

        // Servo bracket mounting holes.
        for (x=[-18,18]) {
            translate([x,-5,-0.1]) cylinder(d=3.4,h=base_thickness+0.2);
            translate([x,-5,-0.01]) cylinder(d1=6.6,d2=3.4,h=1.7);
        }
    }
}

// The v0.1 base remains available above. v0.2 adds only four cargo mounting
// points and two short front tabs; the pocket, pivots and gripper geometry are
// not altered.
module cargo_base_tabs_2d() {
    for (sx=[-1,1])
        hull() {
            translate([sx*33.5,-16]) circle(r=5.2);
            translate([sx*42,-16]) circle(r=4);
        }
}

module mounting_base_v02_assembly() {
    difference() {
        union() {
            mounting_base_assembly();
            linear_extrude(height=base_thickness) cargo_base_tabs_2d();
        }
        for (p=cargo_mount_points) {
            translate([p[0],p[1],-0.1])
                cylinder(d=cargo_mount_hole_d,h=base_thickness+0.2);
            translate([p[0],p[1],-0.01])
                cylinder(d1=6.6,d2=cargo_mount_hole_d,h=1.7);
        }
    }
}

// ---------------- Cargo magazine ----------------
module cargo_deck_plan_2d() {
    outer_half = cargo_inner_width/2 + cargo_wall;
    union() {
        translate([-outer_half,cargo_inner_rear_y])
            square([2*outer_half,
                    cargo_inner_front_y+cargo_wall-cargo_inner_rear_y]);
        for (p=cargo_mount_points)
            hull() {
                translate(p) circle(r=5.2);
                translate([sign(p[0])*outer_half,
                           min(max(p[1],cargo_inner_rear_y+3),
                               cargo_inner_front_y-1)]) circle(r=3.2);
            }
    }
}

module cargo_floor_wedge() {
    x0 = -cargo_inner_width/2;
    x1 =  cargo_inner_width/2;
    y0 = cargo_inner_rear_y;
    y1 = cargo_inner_front_y;
    zb = cargo_deck_bottom_z+cargo_deck_thickness-0.2;
    z0 = cargo_floor_rear_z;
    z1 = cargo_floor_rear_z+cargo_slope_rise;
    polyhedron(
        points=[[x0,y0,zb],[x1,y0,zb],[x1,y1,zb],[x0,y1,zb],
                [x0,y0,z0],[x1,y0,z0],[x1,y1,z1],[x0,y1,z1]],
        faces=[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],
               [2,3,7,6],[3,0,4,7]],
        convexity=4);
}

module cargo_magazine_assembly(local_z0=false) {
    zshift = local_z0 ? -cargo_deck_bottom_z : 0;
    outer_half = cargo_inner_width/2 + cargo_wall;
    side_len = cargo_inner_front_y+cargo_wall
               -(cargo_inner_rear_y-cargo_wall);
    difference() {
        translate([0,0,zshift]) union() {
            translate([0,0,cargo_deck_bottom_z])
                linear_extrude(height=cargo_deck_thickness)
                    cargo_deck_plan_2d();
            cargo_floor_wedge();

            // Side walls reach above the upper box centre but remain open-top.
            for (sx=[-1,1])
                translate([sx>0 ? cargo_inner_width/2 : -outer_half,
                           cargo_inner_rear_y-cargo_wall,
                           cargo_deck_bottom_z])
                    cube([cargo_wall,side_len,
                          cargo_side_wall_top_z-cargo_deck_bottom_z]);

            // Full-height forward wall prevents boxes moving toward the gripper.
            translate([-outer_half,cargo_inner_front_y,cargo_deck_bottom_z])
                cube([2*outer_half,cargo_wall,
                      cargo_front_wall_top_z-cargo_deck_bottom_z]);

            // Gate bearing bosses are outside the clear 28 mm box channel.
            for (sx=[-1,1])
                intersection() {
                    translate([sx*(cargo_inner_width/2+cargo_wall/2),
                               gate_y,gate_axis_z])
                        rotate([0,90,0]) cylinder(d=8,h=cargo_wall,center=true);
                    translate([-50,-55,cargo_deck_bottom_z])
                        cube([100,25,15]);
                }

            // Local reinforcement around the two removable servo-bracket bolts.
            for (y=[-32,-18])
                translate([cargo_inner_width/2,y,47])
                    cube([cargo_wall,7,7]);
        }

        // Four M3 holes attach the deck through 35 mm standoffs.
        for (p=cargo_mount_points)
            translate([p[0],p[1],cargo_deck_bottom_z-0.1+zshift])
                cylinder(d=cargo_mount_hole_d,h=cargo_deck_thickness+0.4);

        // Relief for the gate-mounted kick cam. The cam fills this shallow
        // centre pocket at CLOSE and rotates up/rearward with the gate.
        translate([-kick_cam_half_width-kick_cam_pocket_extra,
                   gate_y-0.4,
                   gate_axis_z+0.9+zshift])
            cube([2*(kick_cam_half_width+kick_cam_pocket_extra),
                  kick_cam_forward+0.9,
                  cargo_floor_rear_z-gate_axis_z+1.4]);

        // Left: removable M3 pivot. Right: rear-open 4 mm printed drive-pin
        // slot; the installed servo horn retains this side.
        translate([-(cargo_inner_width/2+cargo_wall/2),
                   gate_y,gate_axis_z+zshift])
            rotate([0,90,0]) cylinder(d=3.4,h=cargo_wall+0.8,center=true);
        translate([(cargo_inner_width/2+cargo_wall/2),
                   gate_y,gate_axis_z+zshift])
            rotate([0,90,0]) cylinder(d=4.6,h=cargo_wall+0.8,center=true);
        translate([cargo_inner_width/2-0.1,
                   cargo_inner_rear_y-cargo_wall-3,
                   gate_axis_z-2.3+zshift])
            cube([cargo_wall+0.8,
                  gate_y-(cargo_inner_rear_y-cargo_wall-3),4.6]);

        // Two transverse M3 holes retain the replaceable dump-servo cradle.
        for (y=[-32,-18])
            translate([cargo_inner_width/2-0.1,y,50+zshift])
                rotate([0,90,0]) cylinder(d=3.4,h=cargo_wall+5);
    }
}

module cargo_standoffs_reference() {
    color([0.65,0.67,0.7,0.55])
        for (p=cargo_mount_points)
            translate([p[0],p[1],base_thickness])
                cylinder(d=cargo_standoff_d,
                         h=cargo_deck_bottom_z-base_thickness);
}

// ---------------- Jaw ----------------
module right_jaw_local_2d() {
    center_local = patient_center - jaw_pivot;
    arc_start_center = center_local
        + [(jaw_inner_radius+jaw_outer_radius)/2, 0];
    union() {
        ring_segment_2d(center_local, jaw_inner_radius, jaw_outer_radius,
                        jaw_arc_start, jaw_arc_end, jaw_arc_steps);
        hull() {
            circle(r=jaw_pivot_boss_r);
            translate(arc_start_center) circle(r=(jaw_outer_radius-jaw_inner_radius)/2);
        }
        // Rounded nose reduces snagging during a misaligned approach.
        translate(center_local + [((jaw_inner_radius+jaw_outer_radius)/2)*cos(jaw_arc_end),
                                  ((jaw_inner_radius+jaw_outer_radius)/2)*sin(jaw_arc_end)])
            circle(r=jaw_tip_rounding_r);
    }
}

module right_jaw_local(print_z0=false) {
    z0 = print_z0 ? 0 : jaw_bottom_z;
    difference() {
        union() {
            translate([0,0,z0])
                linear_extrude(height=jaw_height) right_jaw_local_2d();
            // Link post overlaps the pivot hub for a single strong printed part.
            translate([jaw_lever_vector[0],jaw_lever_vector[1],z0])
                cylinder(d=link_pin_boss_d,
                         h=link_plane_z+link_thickness-z0);
        }
        translate([0,0,z0-0.1])
            cylinder(d=jaw_pivot_hole_d,
                     h=link_plane_z+link_thickness-z0+0.2);
        translate([jaw_lever_vector[0],jaw_lever_vector[1],z0-0.1])
            cylinder(d=link_pin_hole_d,
                     h=link_plane_z+link_thickness-z0+0.2);
    }
}

module jaw_right_assembly(angle) {
    translate([jaw_pivot[0],jaw_pivot[1],0])
        rotate([0,0,-angle]) right_jaw_local(false);
}

module jaw_left_assembly(angle) {
    mirror([1,0,0]) jaw_right_assembly(angle);
}

// ---------------- Yoke and links ----------------
module yoke_shape(z0=link_plane_z) {
    difference() {
        translate([0,0,z0])
            linear_extrude(height=link_thickness)
                capsule_2d([-yoke_link_x,0],[yoke_link_x,0],link_width);
        for (x=[-yoke_link_x,yoke_link_x])
            translate([x,0,z0-0.1])
                cylinder(d=link_pin_hole_d,h=link_thickness+0.2);
        translate([0,0,z0-0.1])
            linear_extrude(height=link_thickness+0.2)
                capsule_2d([crank_slot_center_x-crank_slot_length/2,0],
                           [crank_slot_center_x+crank_slot_length/2,0],
                           crank_slot_width);
    }
}

module slider_yoke_assembly(yoke_y) {
    translate([0,yoke_y,0]) yoke_shape();
}

module link_shape(p1, p2, z0=link_plane_z) {
    difference() {
        translate([0,0,z0])
            linear_extrude(height=link_thickness)
                capsule_2d(p1,p2,link_width);
        for (p=[p1,p2])
            translate([p[0],p[1],z0-0.1])
                cylinder(d=link_pin_hole_d,h=link_thickness+0.2);
    }
}

module links_assembly(angle,yoke_y) {
    right_point = jaw_link_point_right(angle);
    link_shape([yoke_link_x,yoke_y],right_point);
    mirror([1,0,0]) link_shape([yoke_link_x,yoke_y],right_point);
}

module servo_horn_reference(servo_angle) {
    pin = [servo_axis[0]+servo_crank_radius*cos(servo_angle),
           servo_axis[1]+servo_crank_radius*sin(servo_angle)];
    color([0.95,0.9,0.25,0.75]) {
        translate([0,0,link_plane_z+0.3])
            linear_extrude(height=1.4)
                capsule_2d([servo_axis[0],servo_axis[1]],pin,3.2);
        translate([pin[0],pin[1],link_plane_z])
            cylinder(d=2.5,h=link_thickness+2);
    }
}

// ---------------- Servo mount ----------------
module servo_mount_assembly(local_z0=false) {
    z_shift = local_z0 ? 0 : servo_mount_bottom_z;
    body_min_x = servo_body_center[0]-servo_body_size[0]/2-servo_body_clearance;
    body_max_x = servo_body_center[0]+servo_body_size[0]/2+servo_body_clearance;
    body_min_y = servo_body_center[1]-servo_body_size[1]/2-servo_body_clearance;
    body_max_y = servo_body_center[1]+servo_body_size[1]/2+servo_body_clearance;
    mount_height = 45.5;

    translate([0,0,z_shift]) difference() {
        union() {
            // Two bolted feet, vertical side cheeks and rear tie wall.
            translate([-22,-9,0]) cube([15.5,8,3]);
            translate([14,-9,0]) cube([8,8,3]);
            translate([body_min_x-servo_mount_wall,body_min_y-servo_mount_wall,0])
                cube([servo_mount_wall,
                      body_max_y-body_min_y+servo_mount_wall,mount_height]);
            translate([body_max_x,body_min_y-servo_mount_wall,0])
                cube([servo_mount_wall,
                      body_max_y-body_min_y+servo_mount_wall,mount_height]);
            translate([body_min_x-servo_mount_wall,body_min_y-servo_mount_wall,0])
                cube([body_max_x-body_min_x+2*servo_mount_wall,
                      servo_mount_wall,mount_height]);
            // Nominal ear-hole bosses; adjust these after measuring the actual servo.
            for (x=[servo_body_center[0]-16,servo_body_center[0]+16])
                translate([x-4,body_min_y-servo_mount_wall,36])
                    cube([8,servo_mount_wall,9]);
        }

        for (x=[-18,18])
            translate([x,-5,-0.1]) cylinder(d=3.4,h=3.2);

        // Passage for the moving yoke and its links.
        translate([body_min_x-servo_mount_wall-0.1,-2,16.8])
            cube([body_max_x-body_min_x+2*servo_mount_wall+0.2,12,5]);

        // Servo flange screw pilot holes; actual ear geometry must be measured.
        for (x=[servo_body_center[0]-16,servo_body_center[0]+16])
            translate([x,body_min_y-servo_mount_wall-0.1,40])
                rotate([-90,0,0]) cylinder(d=2.1,h=servo_mount_wall+0.2);
    }
}

// ---------------- Rear dump gate ----------------
module gate_kick_cam_local() {
    // Rounded transverse rails joined by a hull form a printable, gently
    // rising cam. It is only 20 mm wide, leaving floor ledges under both
    // sides of the 25 mm box at CLOSE.
    hull() {
        translate([0,0,(kick_cam_local_z0+kick_cam_local_top_rear)/2])
            rotate([0,90,0])
                cylinder(d=kick_cam_local_top_rear-kick_cam_local_z0,
                         h=2*kick_cam_half_width,center=true,$fn=32);
        translate([0,kick_cam_forward,
                   (kick_cam_local_z0+kick_cam_local_top_front)/2])
            rotate([0,90,0])
                cylinder(d=kick_cam_local_top_front-kick_cam_local_z0,
                         h=2*kick_cam_half_width,center=true,$fn=32);
    }
}

module rear_dump_gate_local() {
    pin_d = 4;
    pin_len = 3.5;
    drive_flange_x = gate_width/2 + pin_len;
    difference() {
        union() {
            translate([-gate_width/2,-gate_barrel_d/2,gate_panel_bottom])
                cube([gate_width,gate_thickness,gate_height]);
            rotate([0,90,0])
                cylinder(d=gate_barrel_d,h=gate_width,center=true);
            gate_kick_cam_local();

            // Short printed bearing pins. The right pin terminates in a flange
            // that is screwed to the stock servo horn.
            translate([gate_width/2,0,0]) rotate([0,90,0])
                cylinder(d=pin_d,h=pin_len);
            intersection() {
                translate([drive_flange_x,0,0]) rotate([0,90,0])
                    cylinder(d=11,h=2.2,center=true);
                translate([drive_flange_x-2,-gate_barrel_d/2,-7])
                    cube([4,14,14]);
            }

            // Two ribs prevent the panel tearing away from the hinge barrel.
            for (sx=[-1,1])
                translate([sx*(gate_width/2-3),-gate_barrel_d/2,0])
                    cube([3,gate_thickness,gate_panel_bottom+8]);
        }
        // Nominal M2 horn screws; tune to the supplied servo horn.
        for (dz=[-3.2,3.2])
            translate([drive_flange_x-1.2,0,dz]) rotate([0,90,0])
                cylinder(d=2.2,h=2.6);

        // Blind left bearing accepts a removable M3 pivot screw after the
        // right drive pin has been slid into its rear-open slot.
        translate([-gate_width/2-0.1,0,0]) rotate([0,90,0])
            cylinder(d=3.4,h=10);
    }
}

module rear_dump_gate_assembly(angle=0) {
    translate([0,gate_y,gate_axis_z])
        rotate([angle,0,0]) rear_dump_gate_local();
}

module rear_dump_gate_print() {
    // Back face on the bed; the short bearing pins remain parallel to it.
    translate([0,gate_panel_bottom+gate_height,gate_barrel_d/2])
        rotate([90,0,0]) rear_dump_gate_local();
}

// ---------------- Dump servo and replaceable cradle ----------------
module dump_servo_reference() {
    color([0.15,0.3,0.75,0.5])
        translate(dump_servo_body_min) cube(dump_servo_body_size);
    color([0.92,0.92,0.95,0.7])
        translate(dump_servo_axis) rotate([0,90,0])
            cylinder(d=5,h=2.5,center=true);
    color([0.92,0.82,0.18,0.7])
        translate(dump_servo_axis) rotate([0,90,0])
            cylinder(d=12,h=1.5,center=true);
}

module dump_servo_mount_assembly(local_z0=false) {
    mount_bottom_z = dump_servo_body_min[2]-2;
    zshift = local_z0 ? -mount_bottom_z : 0;
    inner_x = cargo_inner_width/2+cargo_wall;
    outer_x = dump_servo_body_min[0]+dump_servo_body_size[0]+2.2;
    body_y0 = dump_servo_body_min[1]-0.8;
    body_y1 = dump_servo_body_min[1]+dump_servo_body_size[1]+0.8;
    difference() {
        translate([0,0,zshift]) union() {
            // Side plate bolts to the magazine; lower tray supports the servo.
            translate([inner_x,body_y0,mount_bottom_z])
                cube([2.4,body_y1-body_y0,32]);
            translate([inner_x,body_y0,mount_bottom_z])
                cube([outer_x-inner_x,body_y1-body_y0,2]);
            translate([outer_x-2.2,body_y0,mount_bottom_z])
                cube([2.2,body_y1-body_y0,8]);
            for (y=[body_y0,body_y1-2.2])
                translate([inner_x,y,mount_bottom_z])
                    cube([outer_x-inner_x,2.2,8]);
        }

        // Magazine attachment bolts.
        for (y=[-32,-18])
            translate([inner_x-0.1,y,50+zshift]) rotate([0,90,0])
                cylinder(d=3.4,h=3);

        // Clearance around the coaxial gate flange and servo horn.
        translate([inner_x-0.1,gate_y,gate_axis_z+zshift]) rotate([0,90,0])
            cylinder(d=14,h=outer_x-inner_x+0.2);

        // Two slots accept a reusable cable tie or metal servo strap.
        for (y=[-38,-24])
            translate([dump_servo_body_min[0]-0.2,y,mount_bottom_z-0.1+zshift])
                cube([2.2,4,2.4]);
    }
}

// ---------------- Removable external frame ----------------
module outer_side_frame_profile_2d() {
    b_rear = [frame_base_nodes_y[0],frame_base_node_z];
    b_front = [frame_base_nodes_y[1],frame_base_node_z];
    c_rear = [frame_cargo_nodes_y[0],frame_cargo_node_z];
    c_front = [frame_cargo_nodes_y[1],frame_cargo_node_z];
    t_rear = [frame_top_nodes_y[0],frame_top_node_z];
    t_front = [frame_top_nodes_y[1],frame_top_node_z];
    nodes = [b_rear,b_front,c_rear,c_front,t_rear,t_front];
    union() {
        // Perimeter rails.
        capsule_2d(b_rear,b_front,frame_side_member_width);
        capsule_2d(b_rear,c_rear,frame_side_member_width);
        capsule_2d(b_front,c_front,frame_side_member_width);
        capsule_2d(c_rear,t_rear,frame_side_member_width);
        capsule_2d(c_front,t_front,frame_side_member_width);
        capsule_2d(t_rear,t_front,frame_side_member_width);
        // Triangulation resists fore/aft racking with little material.
        capsule_2d(b_rear,c_front,frame_side_member_width);
        capsule_2d(b_front,c_rear,frame_side_member_width);
        capsule_2d(c_rear,t_front,frame_side_member_width);
        capsule_2d(c_front,t_rear,frame_side_member_width);
        for (p=nodes) translate(p) circle(d=frame_side_boss_d);
    }
}

module outer_side_frame_print() {
    difference() {
        linear_extrude(height=frame_side_thickness)
            outer_side_frame_profile_2d();
        for (p=concat(
            [[frame_base_nodes_y[0],frame_base_node_z],
             [frame_base_nodes_y[1],frame_base_node_z]],
            [[frame_cargo_nodes_y[0],frame_cargo_node_z],
             [frame_cargo_nodes_y[1],frame_cargo_node_z]],
            [[frame_top_nodes_y[0],frame_top_node_z],
             [frame_top_nodes_y[1],frame_top_node_z]]))
            translate([p[0],p[1],-0.1])
                cylinder(d=frame_side_hole_d,h=frame_side_thickness+0.2);
    }
}

module outer_side_frame_assembly(side=1) {
    if (side > 0)
        multmatrix([[0,0,1,frame_side_inner_x],
                    [1,0,0,0],
                    [0,1,0,0],
                    [0,0,0,1]]) outer_side_frame_print();
    else
        multmatrix([[0,0,-1,-frame_side_inner_x],
                    [1,0,0,0],
                    [0,1,0,0],
                    [0,0,0,1]]) outer_side_frame_print();
}

module frame_crossbar_print() {
    difference() {
        translate([-frame_bar_length/2,-frame_bar_width/2,0])
            cube([frame_bar_length,frame_bar_width,frame_bar_height]);

        // M3 thread-forming pilots or heat-set insert pilots from both ends.
        translate([-frame_bar_length/2-0.1,0,frame_bar_height/2])
            rotate([0,90,0])
                cylinder(d=frame_crossbar_pilot_d,
                         h=frame_crossbar_pilot_depth+0.2,$fn=32);
        translate([frame_bar_length/2+0.1,0,frame_bar_height/2])
            rotate([0,-90,0])
                cylinder(d=frame_crossbar_pilot_d,
                         h=frame_crossbar_pilot_depth+0.2,$fn=32);

        // The lower pair also secures the cargo magazine at its four holes.
        for (x=[-33.5,33.5])
            translate([x,0,-0.1])
                cylinder(d=cargo_mount_hole_d,h=frame_bar_height+0.2);
    }
}

module frame_crossbar_assembly(y,z0) {
    translate([0,y,z0]) frame_crossbar_print();
}

module frame_foot_bracket_print() {
    difference() {
        union() {
            // Shared mounting foot and outside clamp plate.
            translate([-4,-frame_foot_width_y/2,0])
                cube([frame_foot_outer_r+4,
                      frame_foot_width_y,frame_foot_plate_thickness]);
            translate([8,-frame_foot_width_y/2,0])
                cube([3,frame_foot_width_y,15]);
            // Two support-free triangular gussets.
            for (yy=[-frame_foot_width_y/2,
                     frame_foot_width_y/2-2])
                hull() {
                    translate([4,yy,frame_foot_plate_thickness])
                        cube([4,2,2]);
                    translate([8,yy,frame_foot_plate_thickness])
                        cube([2,2,11]);
                }
        }
        // Vertical M3 bolt shares an existing v0.1 Hamster mounting hole.
        translate([0,0,-0.1])
            cylinder(d=frame_foot_hole_d,h=frame_foot_plate_thickness+0.2);
        // Horizontal M3 bolt clamps the side truss at the base node.
        translate([7.8,0,6]) rotate([0,90,0])
            cylinder(d=frame_side_hole_d,h=3.4,$fn=32);
    }
}

module frame_foot_bracket_assembly(side=1,y=0) {
    if (side > 0)
        translate([hamster_mount_x,y,base_thickness])
            frame_foot_bracket_print();
    else
        translate([-hamster_mount_x,y,base_thickness])
            mirror([1,0,0]) frame_foot_bracket_print();
}

module outer_frame_assembly() {
    color([0.18,0.24,0.30]) {
        outer_side_frame_assembly(-1);
        outer_side_frame_assembly(1);
        for (y=frame_cargo_nodes_y)
            frame_crossbar_assembly(y,frame_crossbar_bottom_z);
        for (y=frame_top_nodes_y)
            frame_crossbar_assembly(y,frame_top_bar_bottom_z);
    }
    color([0.28,0.32,0.37])
        for (side=[-1,1])
            for (y=frame_base_nodes_y)
                frame_foot_bracket_assembly(side,y);
}

// ---------------- v0.4 compact hamster-style body ----------------
module integrated_side_profile_2d() {
    difference() {
        hull() {
            // Rounded lower sill and upper shoulder make the body read as a
            // compact wheel pod rather than an added rectangular cage.
            translate([-43,chassis_side_lower_z]) circle(r=5);
            translate([15,chassis_side_lower_z]) circle(r=5);
            translate([-42,chassis_side_upper_z]) circle(r=5);
            translate([14,chassis_side_upper_z]) circle(r=5);
        }
        translate([wheel_center_y,wheel_center_z])
            circle(d=wheel_arch_diameter);
    }
}

module integrated_side_shell(side=1) {
    if (side > 0)
        multmatrix([[0,0,1,chassis_side_inner_x],
                    [1,0,0,0],
                    [0,1,0,0],
                    [0,0,0,1]])
            linear_extrude(height=chassis_side_thickness)
                integrated_side_profile_2d();
    else
        multmatrix([[0,0,-1,-chassis_side_inner_x],
                    [1,0,0,0],
                    [0,1,0,0],
                    [0,0,0,1]])
            linear_extrude(height=chassis_side_thickness)
                integrated_side_profile_2d();
}

module compact_front_carrier_plan_2d() {
    union() {
        // Only local functional pads remain; the old full ladder floor and
        // 96 mm rear brace are deliberately absent.
        translate([-24,-8]) square([48,15]);
        for (sx=[-1,1]) {
            hull() {
                translate([sx*jaw_pivot[0],jaw_pivot[1]]) circle(r=9);
                translate([sx*hamster_mount_x,hamster_mount_front_y]) circle(r=4.5);
            }
            translate([sx*hamster_mount_x,hamster_mount_rear_y]) circle(r=4.5);
        }
    }
}

module cargo_shoulder_right(yc) {
    // A short 12 mm inward cantilever replaces each full-width frame bar.
    // Three millimetres overlap the side shell for a strong printed joint.
    translate([cargo_shoulder_inner_x,
               yc-cargo_shoulder_width_y/2,
               cargo_shoulder_bottom_z])
        cube([cargo_shoulder_outer_x-cargo_shoulder_inner_x,
              cargo_shoulder_width_y,4]);

    // Small 45-degree-style rib under the shelf; wheel relief is subtracted
    // later from the complete chassis.
    hull() {
        translate([cargo_shoulder_inner_x,
                   yc-3,cargo_shoulder_bottom_z-0.2]) cube([2,6,1.8]);
        translate([cargo_shoulder_outer_x-4,
                   yc-3,cargo_shoulder_bottom_z-10]) cube([4,6,2]);
    }
}

module cargo_shoulders() {
    for (sx=[-1,1])
        for (yc=frame_cargo_nodes_y)
            if (sx > 0) cargo_shoulder_right(yc);
            else mirror([1,0,0]) cargo_shoulder_right(yc);
}

module stability_skids() {
    for (sx=[-1,1])
        for (yc=[stability_skid_rear_y,stability_skid_front_y])
            translate([sx*stability_skid_center_x,yc,
                       stability_skid_contact_z])
                linear_extrude(height=stability_skid_height)
                    capsule_2d(
                        [0,-(stability_skid_length-stability_skid_width)/2],
                        [0, (stability_skid_length-stability_skid_width)/2],
                        stability_skid_width);
}

module integrated_chassis_raw() {
    difference() {
        union() {
            integrated_side_shell(-1);
            integrated_side_shell(1);
            cargo_shoulders();
            stability_skids();

            translate([0,0,front_module_z_shift])
                linear_extrude(height=base_thickness)
                    compact_front_carrier_plan_2d();
            translate([0,0,front_module_z_shift+collar_bottom_z])
                rear_collar();

            // Original open yoke guide positions retained on the compact pad.
            for (sx=[-1,1])
                translate([sx*26-(sx<0 ? 2.5 : 0),-1.5,
                           front_module_z_shift+base_thickness])
                    cube([2.5,12.5,link_plane_z-base_thickness+2.8]);
        }

        // The wheel cavities cut through the body pods and any local ribs.
        for (sx=[-1,1])
            translate([sx*wheel_center_x,wheel_center_y,wheel_center_z])
                rotate([0,90,0])
                    cylinder(d=wheel_arch_diameter,
                             h=chassis_side_thickness+4,center=true);

        // Existing four-point Hamster attachment pattern.
        for (x=[-hamster_mount_x,hamster_mount_x])
            for (y=[hamster_mount_front_y,hamster_mount_rear_y]) {
                translate([x,y,chassis_ground_z-0.1])
                    cylinder(d=hamster_mount_hole_d,h=base_thickness+0.2);
                translate([x,y,chassis_ground_z-0.01])
                    cylinder(d1=6.6,d2=hamster_mount_hole_d,h=1.7);
            }

        // Unchanged jaw pivot and front-servo bracket coordinates.
        for (sx=[-1,1]) {
            translate([sx*jaw_pivot[0],jaw_pivot[1],
                       front_module_z_shift-0.1])
                cylinder(d=jaw_pivot_hole_d,h=base_thickness+0.2);
            translate([sx*jaw_pivot[0],jaw_pivot[1],
                       front_module_z_shift-0.01])
                cylinder(d1=6.6,d2=jaw_pivot_hole_d,h=1.7);
        }
        for (x=[-18,18]) {
            translate([x,-5,front_module_z_shift-0.1])
                cylinder(d=3.4,h=base_thickness+0.2);
            translate([x,-5,front_module_z_shift-0.01])
                cylinder(d1=6.6,d2=3.4,h=1.7);
        }

        // Four upper shoulders directly receive the unchanged cargo deck.
        for (p=cargo_mount_points)
            translate([p[0],p[1],cargo_shoulder_bottom_z-0.1])
                cylinder(d=cargo_mount_hole_d,h=4.2);

        // Four horizontal M3 clearances receive the removable upper shell.
        for (sx=[-1,1])
            for (p=housing_mount_yz)
                translate([sx*(chassis_side_inner_x+chassis_side_thickness/2),
                           p[0],p[1]])
                    rotate([0,90,0])
                        cylinder(d=housing_mount_clearance_d,
                                 h=chassis_side_thickness+0.4,
                                 center=true,$fn=32);
    }
}

module integrated_chassis_assembly(print_z0=false) {
    // The v0.6 skid contacts already define the printable Z=0 datum.
    translate([0,0,print_z0 ? -stability_skid_contact_z : 0])
        integrated_chassis_raw();
}

module wheel_reference_v04(alpha=0.52) {
    color([0.08,0.09,0.10,alpha])
        for (sx=[-1,1])
            translate([sx*wheel_center_x,wheel_center_y,wheel_center_z])
                rotate([0,90,0])
                    cylinder(d=wheel_diameter,h=wheel_width,center=true);
    color([0.48,0.5,0.52,alpha+0.12])
        for (sx=[-1,1])
            translate([sx*wheel_center_x,wheel_center_y,wheel_center_z])
                rotate([0,90,0]) cylinder(d=8,h=wheel_width+1,center=true);
}

module hamster_core_reference_v04() {
    color([0.16,0.35,0.62,0.18])
        translate([-36,-43,front_module_z_shift+base_thickness])
            cube([72,46,30]);
}

// ---------------- v0.5 removable rounded upper housing ----------------
module rounded_rectangle_centered_2d(size=[10,10],r=2) {
    offset(r=r)
        square([size[0]-2*r,size[1]-2*r],center=true);
}

module housing_outer_solid() {
    ycenter = (housing_rear_y+housing_front_y)/2;
    translate([0,ycenter,housing_bottom_z])
        linear_extrude(height=housing_top_z-housing_bottom_z)
            rounded_rectangle_centered_2d(
                [2*housing_outer_half_x,housing_front_y-housing_rear_y],
                housing_corner_r);
}

module housing_inner_void() {
    ycenter = (housing_inner_rear_y+housing_inner_front_y)/2;
    translate([0,ycenter,housing_bottom_z-0.2])
        linear_extrude(height=housing_inner_top_z-housing_bottom_z+0.2)
            rounded_rectangle_centered_2d(
                [2*housing_inner_half_x,
                 housing_inner_front_y-housing_inner_rear_y],
                housing_corner_r-housing_wall);
}

module housing_mount_boss(side=1,p=[0,0]) {
    boss_len = housing_mount_boss_outer_x-housing_mount_boss_inner_x;
    boss_center_x = (housing_mount_boss_inner_x+housing_mount_boss_outer_x)/2;
    translate([side*boss_center_x,p[0],p[1]])
        rotate([0,90,0])
            cylinder(d=housing_mount_boss_d,h=boss_len,center=true,$fn=40);
}

module upper_housing_raw() {
    difference() {
        union() {
            housing_outer_solid();
            for (sx=[-1,1])
                for (p=housing_mount_yz)
                    housing_mount_boss(sx,p);
        }

        // Hollow shell; extending below the outer solid leaves no floor.
        housing_inner_void();

        // Front window leaves the jaw, yoke and manual access unobstructed.
        translate([-housing_front_window_half_x,housing_inner_front_y-1,
                   housing_front_window_bottom_z])
            cube([2*housing_front_window_half_x,
                  housing_front_y-housing_inner_front_y+6,
                  housing_top_z-housing_front_window_bottom_z+1]);

        // Rear window spans the gate, box fall path and cable exit.
        translate([-housing_rear_window_half_x,housing_rear_y-6,
                   housing_rear_window_bottom_z])
            cube([2*housing_rear_window_half_x,
                  housing_inner_rear_y-housing_rear_y+7,
                  housing_top_z-housing_rear_window_bottom_z+1]);

        // Cargo-side roof/upper walls stop 0.8 mm below the unchanged deck.
        translate([-housing_outer_half_x-2,housing_rear_y-2,
                   housing_cargo_open_bottom_z])
            cube([2*housing_outer_half_x+4,
                  housing_cargo_open_front_y-housing_rear_y+2,
                  housing_top_z-housing_cargo_open_bottom_z+2]);

        // M3 thread-forming pilots in the four housing bosses.
        for (sx=[-1,1])
            for (p=housing_mount_yz) {
                boss_len = housing_mount_boss_outer_x-housing_mount_boss_inner_x;
                boss_center_x = (housing_mount_boss_inner_x
                                 +housing_mount_boss_outer_x)/2;
                translate([sx*boss_center_x,p[0],p[1]])
                    rotate([0,90,0])
                        cylinder(d=housing_mount_pilot_d,
                                 h=boss_len+0.4,center=true,$fn=32);
            }
    }
}

module upper_housing_assembly(print_z0=false) {
    translate([0,0,print_z0 ? -housing_print_bottom_z : 0])
        upper_housing_raw();
}

// ---------------- Cargo reference bodies ----------------
module box_reference_body(layer=0,alpha=0.55) {
    color(layer == 0 ? [0.95,0.48,0.10,alpha]
                     : [0.98,0.75,0.18,alpha])
        translate(lower_box_center)
            rotate([cargo_slope_angle,0,0])
                translate([-box_size[0]/2,-box_size[1]/2,
                           layer*box_size[2]])
                    cube(box_size);
}

module loaded_boxes(count=2,alpha=0.55) {
    if (count >= 1) box_reference_body(0,alpha);
    if (count >= 2) box_reference_body(1,alpha);
}

module released_boxes_reference() {
    // Three translucent poses demonstrate the unobstructed rear/fall path.
    color([1,0.58,0.1,0.26]) {
        translate([-box_size[0]/2,-62,25])
            rotate([12,0,0]) cube(box_size);
        translate([-box_size[0]/2,-78,7])
            rotate([28,0,0]) cube(box_size);
    }
}

// ---------------- Assembly ----------------
module assembly(state="CLOSE",show_approach=false) {
    angle = state == "OPEN" ? jaw_open_angle : jaw_close_angle;
    yoke_y = state == "OPEN" ? yoke_open_y : yoke_closed_y;
    servo_angle = state == "OPEN" ? servo_half_swing : -servo_half_swing;

    color([0.42,0.46,0.5]) mounting_base_assembly();
    color([0.72,0.45,0.18]) jaw_right_assembly(angle);
    color([0.72,0.45,0.18]) jaw_left_assembly(angle);
    color([0.85,0.68,0.22]) slider_yoke_assembly(yoke_y);
    color([0.82,0.57,0.18]) links_assembly(angle,yoke_y);
    color([0.35,0.4,0.46]) servo_mount_assembly(false);
    servo_reference();
    servo_horn_reference(servo_angle);
    patient_reference();
    drivetrain_keepout_reference();
    if (show_approach) {
        approach_patient_reference(0,43,0.17);
        approach_patient_reference(4,43,0.10);
    }
}

module front_assembly_v02(state="CLOSE",show_patient=true) {
    angle = state == "OPEN" ? jaw_open_angle : jaw_close_angle;
    yoke_y = state == "OPEN" ? yoke_open_y : yoke_closed_y;
    servo_angle = state == "OPEN" ? servo_half_swing : -servo_half_swing;

    color([0.42,0.46,0.5]) mounting_base_v02_assembly();
    color([0.72,0.45,0.18]) jaw_right_assembly(angle);
    color([0.72,0.45,0.18]) jaw_left_assembly(angle);
    color([0.85,0.68,0.22]) slider_yoke_assembly(yoke_y);
    color([0.82,0.57,0.18]) links_assembly(angle,yoke_y);
    color([0.35,0.4,0.46]) servo_mount_assembly(false);
    servo_reference();
    servo_horn_reference(servo_angle);
    if (show_patient) patient_reference();
    drivetrain_keepout_reference();
}

module cargo_system(gate_angle=0,box_count=2,show_release=false) {
    color([0.34,0.50,0.38,0.78]) cargo_magazine_assembly(false);
    color([0.80,0.38,0.12]) rear_dump_gate_assembly(gate_angle);
    color([0.28,0.32,0.37]) dump_servo_mount_assembly(false);
    dump_servo_reference();
    cargo_standoffs_reference();
    if (box_count > 0) loaded_boxes(box_count);
    if (show_release) released_boxes_reference();
}

// v0.3 assembly path: the original v0.1 base/front gripper is used verbatim.
// Cargo is carried by the new frame crossbars, so neither cargo tabs nor tall
// free-standing posts are needed on the original base.
module front_assembly_v03(state="CLOSE",show_patient=true) {
    angle = state == "OPEN" ? jaw_open_angle : jaw_close_angle;
    yoke_y = state == "OPEN" ? yoke_open_y : yoke_closed_y;
    servo_angle = state == "OPEN" ? servo_half_swing : -servo_half_swing;

    color([0.42,0.46,0.5]) mounting_base_assembly();
    color([0.72,0.45,0.18]) jaw_right_assembly(angle);
    color([0.72,0.45,0.18]) jaw_left_assembly(angle);
    color([0.85,0.68,0.22]) slider_yoke_assembly(yoke_y);
    color([0.82,0.57,0.18]) links_assembly(angle,yoke_y);
    color([0.35,0.4,0.46]) servo_mount_assembly(false);
    servo_reference();
    servo_horn_reference(servo_angle);
    if (show_patient) patient_reference();
    drivetrain_keepout_reference();
}

module cargo_system_v03(gate_angle=0,box_count=2,show_release=false) {
    color([0.34,0.50,0.38,0.82]) cargo_magazine_assembly(false);
    color([0.90,0.34,0.08]) rear_dump_gate_assembly(gate_angle);
    color([0.28,0.32,0.37]) dump_servo_mount_assembly(false);
    dump_servo_reference();
    if (box_count > 0) loaded_boxes(box_count);
    if (show_release) released_boxes_reference();
}

module frame_complete(gate_angle=0,box_count=2,show_release=false) {
    front_assembly_v03("CLOSE",true);
    outer_frame_assembly();
    cargo_system_v03(gate_angle,box_count,show_release);
}

module front_assembly_v04(state="CLOSE",show_patient=true) {
    angle = state == "OPEN" ? jaw_open_angle : jaw_close_angle;
    yoke_y = state == "OPEN" ? yoke_open_y : yoke_closed_y;
    servo_angle = state == "OPEN" ? servo_half_swing : -servo_half_swing;

    color([0.31,0.40,0.47]) integrated_chassis_assembly(false);
    translate([0,0,front_module_z_shift]) {
        color([0.72,0.45,0.18]) jaw_right_assembly(angle);
        color([0.72,0.45,0.18]) jaw_left_assembly(angle);
        color([0.85,0.68,0.22]) slider_yoke_assembly(yoke_y);
        color([0.82,0.57,0.18]) links_assembly(angle,yoke_y);
        color([0.35,0.4,0.46]) servo_mount_assembly(false);
        servo_reference();
        servo_horn_reference(servo_angle);
    }
    wheel_reference_v04();
    hamster_core_reference_v04();
    if (show_patient) patient_reference();
}

module cargo_system_v04(gate_angle=0,box_count=2,show_release=false) {
    // These three printable solids are geometry-identical to v0.3.
    color([0.34,0.50,0.38,0.84]) cargo_magazine_assembly(false);
    color([0.90,0.34,0.08]) rear_dump_gate_assembly(gate_angle);
    color([0.28,0.32,0.37]) dump_servo_mount_assembly(false);
    dump_servo_reference();
    if (box_count > 0) loaded_boxes(box_count);
    if (show_release) released_boxes_reference();
}

module integrated_complete(gate_angle=0,box_count=2,show_release=false) {
    front_assembly_v04("CLOSE",true);
    cargo_system_v04(gate_angle,box_count,show_release);
}

module housing_complete(gate_angle=0,box_count=2,show_release=false,
                        shell_alpha=0.88) {
    integrated_complete(gate_angle,box_count,show_release);
    color([0.82,0.86,0.88,shell_alpha]) upper_housing_assembly(false);
}

// Printable solids only, in installed coordinates, for mass-property review.
module printed_structure_review() {
    union() {
        integrated_chassis_assembly(false);
        translate([0,0,front_module_z_shift]) {
            jaw_right_assembly(jaw_close_angle);
            jaw_left_assembly(jaw_close_angle);
            slider_yoke_assembly(yoke_closed_y);
            links_assembly(jaw_close_angle,yoke_closed_y);
            servo_mount_assembly(false);
        }
        cargo_magazine_assembly(false);
        rear_dump_gate_assembly(0);
        dump_servo_mount_assembly(false);
        upper_housing_assembly(false);
    }
}

// Printable/reference solids used only by the empty-intersection regression
// checks below.  Keeping this separate from the coloured display assembly
// prevents the chassis itself from being counted as a housing collision.
module housing_clearance_targets(front_state="CLOSE",gate_angle=0) {
    angle = front_state == "OPEN" ? jaw_open_angle : jaw_close_angle;
    yoke_y = front_state == "OPEN" ? yoke_open_y : yoke_closed_y;
    servo_angle = front_state == "OPEN" ? servo_half_swing : -servo_half_swing;

    translate([0,0,front_module_z_shift]) {
        jaw_right_assembly(angle);
        jaw_left_assembly(angle);
        slider_yoke_assembly(yoke_y);
        links_assembly(angle,yoke_y);
        servo_mount_assembly(false);
        servo_reference();
        servo_horn_reference(servo_angle);
    }
    cargo_magazine_assembly(false);
    rear_dump_gate_assembly(gate_angle);
    dump_servo_mount_assembly(false);
    dump_servo_reference();
    wheel_reference_v04(0.88);
    hamster_core_reference_v04();
    patient_reference();
    loaded_boxes(2,1);
}

module housing_interference_test(front_state="CLOSE",gate_angle=0) {
    intersection() {
        upper_housing_raw();
        housing_clearance_targets(front_state,gate_angle);
    }
}

// ---------------- Part selector ----------------
if (part == "printed_structure_review") {
    printed_structure_review();
} else if (part == "physical_complete_close") {
    housing_complete(0,2,false,0.88);
} else if (part == "physical_gate_open") {
    housing_complete(gate_open_angle,0,true,0.88);
} else if (part == "housing_complete_close") {
    housing_complete(0,2,false,0.88);
} else if (part == "housing_gate_open") {
    housing_complete(gate_open_angle,0,true,0.88);
} else if (part == "housing_only") {
    color([0.82,0.86,0.88]) upper_housing_assembly(false);
} else if (part == "housing_transparent") {
    housing_complete(0,2,false,0.28);
} else if (part == "housing_motion_overlay") {
    front_assembly_v04("CLOSE",true);
    color([0.34,0.50,0.38,0.72]) cargo_magazine_assembly(false);
    color([0.2,0.65,0.25,0.62]) rear_dump_gate_assembly(0);
    color([0.9,0.35,0.15,0.28]) rear_dump_gate_assembly(gate_open_angle);
    color([0.82,0.86,0.88,0.32]) upper_housing_assembly(false);
    loaded_boxes(2,0.34);
    released_boxes_reference();
} else if (part == "upper_housing") {
    upper_housing_assembly(true);
} else if (part == "housing_collision_close") {
    housing_interference_test("CLOSE",0);
} else if (part == "housing_collision_open") {
    housing_interference_test("OPEN",gate_open_angle);
} else if (part == "integrated_complete_close") {
    integrated_complete(0,2,false);
} else if (part == "integrated_gate_open") {
    integrated_complete(gate_open_angle,0,true);
} else if (part == "integrated_body_only") {
    color([0.31,0.40,0.47]) integrated_chassis_assembly(false);
    wheel_reference_v04();
    hamster_core_reference_v04();
} else if (part == "integrated_motion_overlay") {
    front_assembly_v04("CLOSE",true);
    color([0.2,0.65,0.25,0.42]) cargo_magazine_assembly(false);
    color([0.2,0.65,0.25,0.62]) rear_dump_gate_assembly(0);
    color([0.95,0.18,0.04,0.62]) rear_dump_gate_assembly(15);
    color([0.9,0.35,0.15,0.28]) rear_dump_gate_assembly(gate_open_angle);
    loaded_boxes(2,0.36);
    released_boxes_reference();
} else if (part == "integrated_top") {
    integrated_complete(0,2,false);
} else if (part == "integrated_chassis") {
    integrated_chassis_assembly(true);
} else if (part == "wheel_reference") {
    cylinder(d=wheel_diameter,h=wheel_width);
} else if (part == "frame_combined_close") {
    frame_complete(0,2,false);
} else if (part == "frame_gate_open") {
    frame_complete(gate_open_angle,0,true);
} else if (part == "frame_motion_overlay") {
    front_assembly_v03("CLOSE",true);
    outer_frame_assembly();
    color([0.2,0.65,0.25,0.55]) cargo_magazine_assembly(false);
    color([0.2,0.65,0.25,0.65]) rear_dump_gate_assembly(0);
    color([0.95,0.2,0.05,0.38]) rear_dump_gate_assembly(30);
    color([0.9,0.35,0.15,0.30]) rear_dump_gate_assembly(gate_open_angle);
    loaded_boxes(2,0.42);
    released_boxes_reference();
} else if (part == "frame_cam_detail") {
    color([0.2,0.55,0.3,0.34]) cargo_magazine_assembly(false);
    color([0.15,0.7,0.25,0.58]) rear_dump_gate_assembly(0);
    color([0.95,0.18,0.04,0.72]) rear_dump_gate_assembly(15);
    loaded_boxes(1,0.28);
} else if (part == "frame_only") {
    outer_frame_assembly();
} else if (part == "outer_side_frame") {
    outer_side_frame_print();
} else if (part == "frame_crossbar") {
    frame_crossbar_print();
} else if (part == "frame_foot_bracket") {
    frame_foot_bracket_print();
} else if (part == "cargo_loaded_1") {
    front_assembly_v03("CLOSE",false);
    outer_frame_assembly();
    cargo_system_v03(0,1,false);
} else if (part == "cargo_loaded_2") {
    front_assembly_v03("CLOSE",false);
    outer_frame_assembly();
    cargo_system_v03(0,2,false);
} else if (part == "cargo_combined_close") {
    frame_complete(0,2,false);
} else if (part == "cargo_gate_open") {
    frame_complete(gate_open_angle,0,true);
} else if (part == "cargo_motion_overlay") {
    front_assembly_v03("CLOSE",true);
    outer_frame_assembly();
    color([0.2,0.65,0.25,0.55]) cargo_magazine_assembly(false);
    color([0.2,0.65,0.25,0.55]) rear_dump_gate_assembly(0);
    color([0.9,0.35,0.15,0.38]) rear_dump_gate_assembly(gate_open_angle);
    loaded_boxes(2,0.42);
    released_boxes_reference();
} else if (part == "mounting_base_v02") {
    mounting_base_v02_assembly();
} else if (part == "cargo_magazine") {
    cargo_magazine_assembly(true);
} else if (part == "rear_dump_gate") {
    rear_dump_gate_print();
} else if (part == "dump_servo_mount") {
    dump_servo_mount_assembly(true);
} else if (part == "box_reference") {
    cube(box_size);
} else if (part == "assembly_close") {
    assembly("CLOSE",false);
} else if (part == "assembly_open") {
    assembly("OPEN",true);
} else if (part == "motion_overlay") {
    color([0.2,0.65,0.25,0.55]) assembly("CLOSE",false);
    color([0.9,0.35,0.15,0.32]) {
        jaw_right_assembly(jaw_open_angle);
        jaw_left_assembly(jaw_open_angle);
        slider_yoke_assembly(yoke_open_y);
        links_assembly(jaw_open_angle,yoke_open_y);
    }
} else if (part == "mounting_base") {
    mounting_base_assembly();
} else if (part == "jaw_right") {
    right_jaw_local(true);
} else if (part == "jaw_left") {
    mirror([1,0,0]) right_jaw_local(true);
} else if (part == "slider_yoke") {
    translate([0,0,-link_plane_z]) yoke_shape();
} else if (part == "link") {
    link_shape([0,0],[link_length,0],0);
} else if (part == "servo_mount") {
    servo_mount_assembly(true);
} else if (part == "patient_reference") {
    translate([-patient_center[0],-patient_center[1],0]) patient_reference(1);
} else if (part == "none") {
    // Library/validation mode; emit no top-level geometry.
} else {
    assert(false,str("Unknown part selector: ",part));
}
