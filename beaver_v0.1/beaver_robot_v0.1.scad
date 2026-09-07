/*
  Beaver Robot v0.1 - mechanism-first patient-cylinder carrier
  Units: millimetres. +Y is forward, Z=0 is the arena floor.

  One centrally mounted micro-servo drives a fore/aft slider yoke through a
  crank pin in a transverse slot. Two identical links move mirrored C-jaws.
  The patient remains on the floor; the mechanism blocks escape geometrically.
*/

$fn = 64;
part = "assembly_close";
// Select: assembly_close, assembly_open, motion_overlay, mounting_base,
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
link_pin_hole_d = 2.6;           // M2.5 clearance
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
crank_slot_width = 2.9;

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
assert(2 * servo_half_swing < 90,
       "Servo swing is excessive");
assert(max(robot_size) <= 100,
       "Robot mechanism exceeds the 100 mm cube");

echo(str("VERIFY patient = D", patient_diameter, " x H", patient_height));
echo(str("VERIFY radial clearance = ", radial_clearance));
echo(str("VERIFY OPEN entrance width = ", open_entry_width));
echo(str("VERIFY CLOSE front throat = ", closed_front_opening));
echo(str("VERIFY yoke travel = ", yoke_travel));
echo(str("VERIFY servo total swing = ", 2 * servo_half_swing));
echo(str("VERIFY robot envelope = ", robot_size));

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

// ---------------- Part selector ----------------
if (part == "assembly_close") {
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
} else {
    assert(false,str("Unknown part selector: ",part));
}
