/*
  Hamster Robotics Challenge Robot v0.7
  Manufacturing/assembly revision. Capture arcs and travel preserved;
  serviceable joints, removable guide caps, servo clamp and printed link.
  REAL COMPONENT FIT AND BALANCE MUST BE CONFIRMED BEFORE FULL PRINTING.

  Coordinate system
    +Y : robot front / sample entry and release direction
    Z=0: floor

  The sample disk is always assumed to remain on the floor.
  All sample/servo/linkage objects are preview-only (%) in assembly mode.
*/

$fn = 72;

// ---------------------------
// User controls
// ---------------------------
part_to_render = "assembly";
// Print selectors are listed in README; v0.7 gate_slider is ONE rigid part.
// Legacy v0.6 selectors:
// "assembly", "chassis", "lid", "left_motor_mount",
// "right_motor_mount", "electronics_deck", "servo_mount",
// "caster_mount"
// Lower-system and service selectors remain available at the end of the file.

front_gate_state = "UP";       // "UP" or "DOWN"
show_sample = true;
show_servo = true;
show_linkage = true;
show_capture_circle = false;
show_funnel_path = false;
show_capture_test_disks = false;
show_motor_keepouts = false;

show_motors = true;
show_wheels = true;
show_battery = true;
show_controller = true;
show_gate_mechanism = true;
show_lower_capture_system = true;
show_caster = true;
show_gate_keepout = false;
show_lid = true;

cutaway_view = false;
cutaway_side = "RIGHT"; // "RIGHT" or "LEFT"
show_gate_window = true;

// ---------------------------
// Robot and sample envelope
// ---------------------------
robot_width = 100;
robot_length = 100;
robot_height = 100;

sample_diameter = 56;
sample_radius = sample_diameter / 2;
sample_thickness = 5;
sample_preview_clearance = 0.05;

capture_center_x = 0;
capture_center_y = 5;
capture_center = [capture_center_x, capture_center_y];

// ---------------------------
// Curved capture barriers
// ---------------------------
sample_radial_clearance = 2;
barrier_inner_radius = sample_diameter / 2 + sample_radial_clearance; // 30 mm

rear_barrier_height = 10;
rear_barrier_thickness = 2.5;
rear_barrier_bottom_clearance = 0.3;

// Shared defaults retained as aliases for the generic arc generator.
barrier_height = rear_barrier_height;
barrier_thickness = rear_barrier_thickness;
barrier_outer_radius = barrier_inner_radius + barrier_thickness;
barrier_bottom_clearance = rear_barrier_bottom_clearance;
barrier_bottom_chamfer = 0.5;

/*
  95-degree rear arcs centered at 222.5 and 317.5 degrees meet at the rear
  and leave a 170-degree front mouth. This is wider than the sample while
  the two DOWN-state side gaps remain far narrower than the sample.
*/
rear_barrier_arc = 95;
rear_left_center_angle = 222.5;
rear_right_center_angle = 317.5;

front_gate_arc = 105;
front_gate_center_angle = 90;
front_gate_height = 10;
front_gate_thickness = 2.5;
front_gate_inner_radius = barrier_inner_radius;
front_gate_outer_radius = front_gate_inner_radius + front_gate_thickness;
front_gate_bottom_clearance = 0.3;
front_gate_down_bottom_z = front_gate_bottom_clearance;
front_gate_up_bottom_z = 12;
front_gate_bottom_z = front_gate_state == "UP"
    ? front_gate_up_bottom_z
    : front_gate_down_bottom_z;
front_gate_travel = front_gate_up_bottom_z - front_gate_down_bottom_z;

// ---------------------------
// Curved sweeping funnel (fixed walls, never a floor plate)
// ---------------------------
funnel_front_width = 94;       // centerline-to-centerline at the two noses
funnel_exit_width = 62;        // centerline-to-centerline at the pocket end
funnel_rear_width = funnel_exit_width; // legacy-name compatibility
funnel_length = 32;
funnel_wall_height = 9;
funnel_wall_thickness = 2.5;
funnel_height = funnel_wall_height; // legacy-name compatibility
funnel_bottom_clearance = 0.3;
funnel_tip_radius = 3;
funnel_curve_strength = 0.55; // 0 = tighter/earlier turn, 1 = wider/longer bow
bezier_steps = 16;
funnel_transition_steps = 8;
funnel_transition_handle = 0.8;
funnel_capture_offset_target = 15;

funnel_exit_forward_offset = 8;
funnel_exit_y = capture_center_y + funnel_exit_forward_offset; // 13 mm
funnel_front_y = funnel_exit_y + funnel_length;                 // 45 mm
funnel_front_setback = robot_length / 2 - funnel_front_y;
funnel_rear_y = funnel_exit_y; // legacy-name compatibility
funnel_front_center_x = funnel_front_width / 2;             // 47 mm
funnel_exit_center_x = funnel_exit_width / 2;               // 31 mm
funnel_rear_center_x = funnel_exit_center_x;

// Cubic Bezier control placement. Defaults approximate the requested points:
// [47,45] -> [45.5,36] -> [34.4,21.6] -> [31,13].
funnel_control_1_y_ratio = 0.28;
funnel_control_2_y_ratio = 0.27;

// ---------------------------
// Front-gate support, carrier and vertical guides
// ---------------------------
front_support_angles = [45, 135];
front_gate_support_width = 4;
front_gate_support_thickness = 3;
front_support_radial_width = front_gate_support_thickness;
front_support_tangential_width = front_gate_support_width;
front_support_radial_inset = 0.5;
front_support_top_z = 23;
front_support_bottom_overlap = 1.0;

carrier_bottom_z = 20;
carrier_thickness = 3;
carrier_beam_width = 4;
carrier_half_span = 35;
carrier_y = front_gate_outer_radius * sin(45);
carrier_slot_clearance = 0.4;
carrier_slot_pad_size = 10;

guide_axis_x = 35;
guide_axis_y = capture_center[1] + carrier_y;
guide_clearance = 0.4;
guide_rail_size = [4, 6];
guide_rail_bottom_z = 16;
guide_rail_top_z = 61;
guide_sleeve_outer = [8, 10];
guide_sleeve_height = 10;
guide_column_center_x = 47;
guide_column_size = [6, 10];
guide_column_height = 61;
guide_bridge_height = 3;

// ---------------------------
// SG90-class servo and crank-slider linkage
// ---------------------------
servo_length = 23;
servo_height = 22.5;
servo_width = 12.5;
servo_tab_span = 32;
servo_tab_depth = 5;
servo_tab_thickness = 2;
servo_shaft_offset = -5.5;
servo_shaft_offset_y = servo_shaft_offset;
servo_shaft_radius = 2.5;
servo_shaft_length = 4;

servo_crank_radius = 8;
linkage_length = 21;
link_pin_radius = 1.15;
linkage_rod_radius = 1.6;
carrier_lug_size = [5, 8, 13];
servo_axis_x = servo_height / 2;
linkage_plane_x = 22.5;
carrier_link_x = 18;
carrier_link_y = guide_axis_y;
carrier_link_local_z = carrier_bottom_z + carrier_lug_size[2] / 2;
servo_half_swing = asin(front_gate_travel / (2 * servo_crank_radius));
servo_crank_angle = front_gate_state == "UP" ? servo_half_swing : -servo_half_swing;
servo_axis_y = carrier_link_y - servo_crank_radius * cos(servo_half_swing);
servo_axis_z = front_gate_down_bottom_z
             + carrier_link_local_z
             + servo_crank_radius * sin(servo_half_swing)
             + linkage_length;
servo_body_center_x = servo_axis_x - servo_height / 2;
servo_body_center_y = servo_axis_y - servo_shaft_offset_y;
servo_body_center_z = servo_axis_z;

servo_mount_wall = 3;
servo_mount_clearance = 0.5;
servo_mount_base_z = servo_body_center_z - servo_width / 2 - servo_mount_wall;
servo_mount_depth = servo_length + 2 * (servo_mount_wall + servo_mount_clearance);
servo_mount_inner_width = servo_height + 2 * servo_mount_clearance;
servo_mount_outer_width = servo_mount_inner_width + 2 * servo_mount_wall;
servo_mount_back_height = servo_width + 2 * servo_mount_wall;
servo_mount_arm_y = servo_body_center_y + servo_mount_depth / 2
                  - servo_mount_wall;
servo_support_post_size = [6, 8];

// ---------------------------
// Lower chassis and motor keep-out preview
// ---------------------------
side_rail_width = 4;
side_rail_height = 6;
side_rail_center_x = robot_width / 2 - side_rail_width / 2;
side_rail_bottom_z = 0.3;

rear_connector_width = 3;
rear_join_width = 7;
rear_join_depth = 5;

motor_keepout_size = [10, 18, 24];
motor_keepout_center = [40, -22, 14];

// ---------------------------
// v0.6 upper chassis and drive parameters
// ---------------------------
wall_thickness = 3;
corner_radius = 6;
lid_thickness = 3;
upper_shell_bottom_z = rear_barrier_bottom_clearance + rear_barrier_height + 1.2;
upper_shell_top_z = robot_height - lid_thickness;
upper_shell_height = upper_shell_top_z - upper_shell_bottom_z;

front_lower_opening_width = 84;
front_lower_opening_height = 36;
gate_window_width = 50;
gate_window_height = 20;
gate_window_bottom_z = 36;

motor_length = 30;
motor_width = 15;
motor_height = 20;
motor_shaft_diameter = 3;
motor_shaft_length = 12;
motor_clearance = 1.0;
motor_mount_wall = 2.5;
motor_mount_floor_thickness = 1.5;
motor_center_y = -15;

wheel_diameter = 45;
wheel_width = 10;
wheel_clearance = 3;
wheel_z_offset = 0;
wheel_center_z = wheel_diameter / 2 + wheel_z_offset;
wheel_well_radius = wheel_diameter / 2 + wheel_clearance;
wheel_center_x = robot_width / 2 + wheel_clearance + wheel_width / 2;

motor_center_x = robot_width / 2
               - wall_thickness
               - motor_clearance
               - motor_length / 2;
motor_inner_x = motor_center_x - motor_length / 2;
motor_outer_x = motor_center_x + motor_length / 2;
motor_bottom_z = wheel_center_z - motor_height / 2;
motor_top_z = wheel_center_z + motor_height / 2;
motor_mount_base_z = motor_bottom_z - motor_mount_floor_thickness;

caster_diameter = 12;
caster_mount_width = 30;
caster_mount_length = 14;
caster_mount_hole_spacing = 20;
caster_height = 12;
caster_center_y = -40;
caster_plate_thickness = 3;
caster_back_y = -robot_length / 2 + wall_thickness + 0.5;
caster_front_y = -36;
caster_flange_height = 15;
caster_wall_bolt_z = caster_height + 10;

battery_width = 24;
battery_length = 36;
battery_height = 20;
battery_clearance = 2;
battery_center_y = -16;
battery_mount_base_z = max(rear_barrier_bottom_clearance
                          + rear_barrier_height + 0.7,
                          caster_height + caster_plate_thickness + 0.5);
battery_tray_thickness = 2.5;
battery_bottom_z = battery_mount_base_z + battery_tray_thickness;
battery_center_z = battery_bottom_z + battery_height / 2;
battery_strap_width = 8;
battery_rear_bolt_y = -39;
battery_stop_wall = 2;
battery_front_hanger_x = 18;
battery_front_hanger_y = 1;

deck_height = 45;
deck_thickness = 3.5;
deck_width = 86;
deck_length = 52;
deck_center_y = -17;
deck_frame_width = 6;
electronics_hole_spacing = 10;
electronics_hole_diameter = 3.4;
deck_mount_x = 40;

controller_width = 45;
controller_length = 35;
controller_height = 8;
controller_center_y = -18;
controller_standoff_height = 5;

front_gate_keepout_width = 80;
front_gate_keepout_depth = 40;
front_gate_keepout_height = 67;
front_gate_keepout_center_y = 26;
gate_guide_clearance = guide_clearance;

servo_clamp_bolt_spacing = servo_mount_depth; // nominal 30 mm

lower_mount_spacing_x = 88;
lower_mount_front_y = 17;
lower_mount_rear_y = -38;
lower_mount_tab_size = 12;
lower_mount_boss_outer_diameter = 9;
lower_mount_hole_diameter = 3.4;
lower_mount_boss_bottom_z = side_rail_bottom_z + side_rail_height;
lower_mount_boss_height = 13;

boss_outer_diameter = 9;
boss_hole_diameter = 3.4; // M3 nut seats replace unspecified heat-set inserts
lid_hole_diameter = 3.4;
flush_head_diameter = 6.6;
flush_head_depth = 1.6;
lid_boss_spacing_x = 80;
lid_boss_spacing_y = 80;
lid_boss_bottom_z = 65;

rear_panel_width = 54;
rear_panel_height = 28;
rear_panel_bottom_z = 52;
rear_panel_reinforcement = 1.5;

sensor_plate_width = 52;
sensor_plate_depth = 8;
sensor_plate_thickness = 3;
sensor_plate_z = 70;
sensor_hole_spacing = 30;
sensor_hole_diameter = 3.4;

// v0.7 serviceable joints: first print the fit coupon.
nut_af = 5.8;
nut_pocket_height = 2.6;
small_bolt_clearance = 2.8;
small_pilot = 2.0; // finish to ~2.05 mm and tap M2.5x0.45; test on coupon
guide_cap_thickness = 3;
guide_cap_socket_depth = 4;
guide_cap_clearance = 0.3;
guide_cap_top_z = guide_column_height + guide_cap_thickness;
servo_post_top_z = servo_mount_base_z - servo_mount_wall / 2;
servo_clamp_thickness = 3;
link_plate_thickness = 2.5;
link_end_diameter = 6;
battery_hanger_size = 9;
battery_rear_hanger_y = -35;
deck_ledge_thickness = 4.5;
lid_nut_bottom_z = upper_shell_top_z - 6;

// ---------------------------
// Derived geometric checks
// ---------------------------
rear_left_front_angle = rear_left_center_angle - rear_barrier_arc / 2;
rear_left_rear_angle = rear_left_center_angle + rear_barrier_arc / 2;
rear_right_rear_angle = rear_right_center_angle - rear_barrier_arc / 2;
rear_right_front_angle = rear_right_center_angle + rear_barrier_arc / 2 - 360;

front_entry_angle = rear_left_front_angle - rear_right_front_angle;
front_entry_chord = 2 * barrier_inner_radius * sin(front_entry_angle / 2);

right_side_gap_angle = (front_gate_center_angle - front_gate_arc / 2)
                     - rear_right_front_angle;
left_side_gap_angle = rear_left_front_angle
                    - (front_gate_center_angle + front_gate_arc / 2);
max_down_gap_angle = max(right_side_gap_angle, left_side_gap_angle);
max_down_gap_chord = 2 * (barrier_inner_radius + barrier_thickness / 2)
                   * sin(max_down_gap_angle / 2);

rear_gap_angle = max(0, rear_right_rear_angle - rear_left_rear_angle);
rear_gap_chord = 2 * (barrier_inner_radius + barrier_thickness / 2)
               * sin(rear_gap_angle / 2);

up_vertical_clearance = front_gate_up_bottom_z - sample_thickness;
funnel_front_clear_width = funnel_front_width - 2 * funnel_tip_radius;
funnel_exit_clear_width = funnel_exit_width - funnel_wall_thickness;
funnel_front_margin = funnel_front_clear_width - sample_diameter;
funnel_rear_margin = funnel_exit_clear_width - sample_diameter;
funnel_transition_min_center_x = min([
    for (p = funnel_transition_points(1)) p[0] - capture_center_x
]);
funnel_transition_min_clear_width =
    2 * (funnel_transition_min_center_x - funnel_wall_thickness / 2);
funnel_nominal_offset_capacity = funnel_front_clear_width / 2 - sample_radius;
funnel_offset_target_margin = funnel_nominal_offset_capacity
                            - funnel_capture_offset_target;
entry_margin = front_entry_chord - sample_diameter;
retention_margin = sample_diameter - max(max_down_gap_chord, rear_gap_chord);

motor_keepout_inner_x = motor_keepout_center[0] - motor_keepout_size[0] / 2;
motor_front_y = motor_keepout_center[1] + motor_keepout_size[1] / 2;
motor_barrier_plan_clearance = sqrt(
    pow(motor_keepout_inner_x - capture_center_x, 2)
    + pow(motor_front_y - capture_center_y, 2)
) - barrier_outer_radius;
gate_to_guide_plan_clearance = guide_axis_x - guide_rail_size[0] / 2
                             - front_gate_outer_radius
                               * cos(front_gate_center_angle
                                     - front_gate_arc / 2);
sleeve_to_mount_arm_y_clearance = servo_mount_arm_y - 7 / 2
                                - (guide_axis_y
                                   + guide_sleeve_outer[1] / 2);
up_lug_to_mount_clearance = servo_mount_base_z
                          - (front_gate_up_bottom_z
                             + carrier_bottom_z
                             + carrier_lug_size[2]);

// v0.6 upper-system clearances
lower_gate_mechanism_top_z = max([
    front_gate_up_bottom_z + carrier_bottom_z + guide_sleeve_height,
    servo_mount_base_z + servo_mount_back_height + servo_clamp_thickness,
    guide_cap_top_z
]);
motor_to_barrier_z_clearance = motor_bottom_z
                              - (rear_barrier_bottom_clearance
                                 + rear_barrier_height);
battery_to_barrier_z_clearance = battery_mount_base_z
                                - (rear_barrier_bottom_clearance
                                   + rear_barrier_height);
battery_to_motor_x_clearance = motor_inner_x - battery_width / 2;
battery_front_y = battery_center_y + battery_length / 2;
gate_keepout_rear_y = front_gate_keepout_center_y
                    - front_gate_keepout_depth / 2;
battery_to_gate_y_clearance = gate_keepout_rear_y - battery_front_y;
battery_to_deck_z_clearance = deck_height - battery_bottom_z - battery_height;
battery_to_cradle_x_clearance = motor_inner_x - motor_clearance
                              - motor_mount_wall - battery_width / 2;
battery_to_caster_z_clearance = battery_mount_base_z
                              - caster_height - caster_plate_thickness;
deck_front_y = deck_center_y + deck_length / 2;
servo_mount_rear_y = servo_body_center_y - servo_mount_depth / 2;
deck_to_servo_y_clearance = servo_mount_rear_y - deck_front_y;
motor_removal_radius = sqrt(pow(motor_width / 2 + motor_clearance, 2)
                            + pow(motor_height / 2 + motor_clearance, 2));
motor_cradle_half_y = motor_width / 2 + motor_clearance + motor_mount_wall;
battery_front_frame_y = min(battery_front_hanger_y - battery_hanger_size/2,
                           battery_center_y + battery_length / 2
                             + battery_clearance - 5);
battery_rear_frame_front_y = battery_center_y - battery_length / 2
                            - battery_clearance + 5;
upper_body_max_z = upper_shell_top_z + lid_thickness;
wheel_ground_z = wheel_center_z - wheel_diameter / 2;

max_plan_radius = max([
    side_rail_center_x + side_rail_width / 2,
    min(robot_width / 2,
        abs(capture_center_x) + funnel_front_center_x + funnel_tip_radius),
    guide_column_center_x + guide_column_size[0] / 2
]);
max_y_extent = max([
    funnel_front_y + funnel_tip_radius,
    servo_body_center_y + servo_mount_depth / 2,
    servo_mount_arm_y + servo_support_post_size[1] / 2
]);
max_z_extent = max([
    guide_column_height,
    servo_mount_base_z + servo_mount_back_height,
    front_gate_up_bottom_z + carrier_bottom_z + guide_sleeve_height
]);

// Mechanical sanity assertions. A failed requirement stops rendering.
assert(front_gate_state == "UP" || front_gate_state == "DOWN",
       "front_gate_state must be UP or DOWN");
assert(barrier_inner_radius >= sample_radius + sample_radial_clearance,
       "Radial sample clearance is below the requested value");
assert(front_entry_chord > sample_diameter,
       "Fixed rear barriers make the front entry too narrow");
assert(funnel_front_margin > 0 && funnel_rear_margin > 0,
       "Funnel clear width must exceed sample diameter");
assert(funnel_transition_min_clear_width > sample_diameter,
       "Rear funnel transition is too narrow for the sample");
assert(funnel_tip_radius >= funnel_wall_thickness / 2,
       "Funnel tip radius must cover the wall end cleanly");
assert(funnel_front_width >= 90 && funnel_front_width <= 96,
       "Recommended funnel_front_width range is 90 to 96 mm");
assert(funnel_front_center_x - funnel_tip_radius < robot_width / 2,
       "The complete rounded funnel nose lies outside the chassis");
assert(funnel_curve_strength >= 0 && funnel_curve_strength <= 1,
       "funnel_curve_strength must be between 0 and 1");
assert(bezier_steps >= 8,
       "Use at least 8 Bezier segments for a smooth funnel");
assert(funnel_transition_steps >= 4,
       "Use at least 4 transition segments at the rear barrier");
assert(max_down_gap_chord < sample_diameter,
       "A DOWN-state side gap is wide enough for the sample to escape");
assert(rear_gap_chord < sample_diameter,
       "The rear gap is wide enough for the sample to escape");
assert(up_vertical_clearance > 0,
       "UP gate does not clear the top of the floor-level sample");
assert(barrier_bottom_clearance >= 0 && funnel_bottom_clearance >= 0,
       "Fixed geometry penetrates below the floor");
assert(servo_crank_radius * 2 >= front_gate_travel,
       "Crank diameter is smaller than the required gate travel");
assert(max_plan_radius <= robot_width / 2 + 0.01,
       "A component exceeds the 100 mm robot width");
assert(max_y_extent <= robot_length / 2 + 0.01,
       "A component exceeds the 100 mm robot length");
assert(max_z_extent <= robot_height + 0.01,
       "A component exceeds the 100 mm robot height");
assert(servo_mount_base_z > front_gate_up_bottom_z + carrier_bottom_z
                              + carrier_thickness + 2,
       "UP carrier collides with the servo mount");
assert(motor_barrier_plan_clearance > 1,
       "Rear barrier enters the conservative motor keep-out envelope");
assert(gate_to_guide_plan_clearance > 1,
       "Front gate can collide with a vertical guide rail");
assert(sleeve_to_mount_arm_y_clearance > 1,
       "Gate sleeve can collide with a servo-mount support arm");
assert(carrier_link_x - carrier_lug_size[0]/2 - servo_mount_outer_width/2 >= 0.5,
       "UP linkage lug must stay to the side of the servo cradle");
assert(carrier_link_local_z - carrier_bottom_z - carrier_thickness
       - link_end_diameter/2 >= 0.5, "Printed link end hits the carrier beam");
assert(upper_body_max_z <= robot_height + 0.01,
       "Upper body and lid exceed the 100 mm height envelope");
assert(upper_shell_top_z <= robot_height - lid_thickness + 0.01,
       "The lid cannot fit inside the height envelope");
assert(motor_center_y <= -10 && motor_center_y >= -25,
       "Recommended motor_center_y range is -10 to -25 mm");
assert(motor_to_barrier_z_clearance > 1,
       "Drive motor enters the fixed-barrier vertical volume");
assert(battery_to_barrier_z_clearance > 0.5,
       "Battery tray enters the lower capture volume");
assert(battery_to_motor_x_clearance >= 1,
       "Battery overlaps a drive motor");
assert(battery_to_cradle_x_clearance >= 0.4,
       "Battery overlaps a motor cradle");
assert(battery_to_deck_z_clearance >= 3,
       "Battery is too tall for the electronics deck");
assert(battery_to_caster_z_clearance >= 0.4,
       "Battery tray overlaps the caster plate");
assert(battery_front_frame_y - (motor_center_y + motor_cradle_half_y) >= 0.5,
       "Move the battery front hanger/frame away from the motor cradle");
assert(motor_center_y - motor_cradle_half_y - battery_rear_frame_front_y >= 0.5,
       "Move the battery rear frame away from the motor cradle");
assert(battery_to_gate_y_clearance >= 2,
       "Battery enters the front-gate keep-out zone");
assert(deck_to_servo_y_clearance >= 2,
       "Electronics deck blocks servo access or linkage space");
assert(lower_gate_mechanism_top_z < front_gate_keepout_height,
       "Gate/servo mechanism exceeds the reserved upper tunnel");
assert(wheel_clearance >= 2,
       "Wheel-to-body clearance is below 2 mm");
assert(wheel_ground_z >= -0.01,
       "Drive wheel penetrates below the floor");
assert(motor_removal_radius < wheel_well_radius,
       "Motor cannot be removed through the wheel-well opening");
assert(cutaway_side == "RIGHT" || cutaway_side == "LEFT",
       "cutaway_side must be RIGHT or LEFT");

// Console report supports verification without measuring the rendered mesh.
echo(str("v0.7 state: ", front_gate_state));
echo(str("front entry chord = ", front_entry_chord,
         " mm; disk margin = ", entry_margin, " mm"));
echo(str("largest DOWN gap chord = ", max_down_gap_chord,
         " mm; retention margin = ", retention_margin, " mm"));
echo(str("UP vertical clearance = ", up_vertical_clearance, " mm"));
echo(str("front gate travel = ", front_gate_travel,
         " mm; servo half-swing = ", servo_half_swing, " deg"));
echo(str("motor keep-out clearance = ", motor_barrier_plan_clearance,
         " mm; guide clearance = ", gate_to_guide_plan_clearance, " mm"));
echo(str("lug-to-servo SIDE clearance = ",
         carrier_link_x-carrier_lug_size[0]/2-servo_mount_outer_width/2,
         " mm; sleeve-to-arm clearance = ",sleeve_to_mount_arm_y_clearance," mm"));
echo(str("curved funnel clear mouth/exit = ", funnel_front_clear_width,
         " / ", funnel_exit_clear_width, " mm"));
echo(str("minimum rear transition clear width = ",
         funnel_transition_min_clear_width, " mm"));
echo(str("nominal lateral offset capacity = +/-",
         funnel_nominal_offset_capacity,
         " mm; target margin = ", funnel_offset_target_margin, " mm"));
echo(str("upper clearances: motor/barrier = ",
         motor_to_barrier_z_clearance,
         " mm, battery/barrier = ", battery_to_barrier_z_clearance,
         " mm, battery/motor = ", battery_to_motor_x_clearance, " mm"));
echo(str("service clearances: battery/gate = ",
         battery_to_gate_y_clearance,
         " mm, deck/servo = ", deck_to_servo_y_clearance, " mm"));
echo(str("wheel well radius = ", wheel_well_radius,
         " mm; motor removal radius = ", motor_removal_radius, " mm"));
echo(str("battery cradle/caster/deck clearances = ",
         battery_to_cradle_x_clearance, " / ",
         battery_to_caster_z_clearance, " / ",
         battery_to_deck_z_clearance, " mm"));

// ---------------------------
// Reusable primitives
// ---------------------------
function cubic_bezier_point(t, p0, p1, p2, p3) =
      pow(1 - t, 3) * p0
    + 3 * pow(1 - t, 2) * t * p1
    + 3 * (1 - t) * pow(t, 2) * p2
    + pow(t, 3) * p3;

function funnel_front_point(side) =
    [capture_center_x + side * funnel_front_center_x, funnel_front_y];

function funnel_exit_point(side) =
    [capture_center_x + side * funnel_exit_center_x, funnel_exit_y];

function funnel_control_1(side) = [
    capture_center_x
        + side * (funnel_exit_center_x
                  + (funnel_front_center_x - funnel_exit_center_x)
                    * (0.85 + 0.10 * funnel_curve_strength)),
    funnel_front_y - funnel_length * funnel_control_1_y_ratio
];

function funnel_control_2(side) = [
    capture_center_x
        + side * (funnel_exit_center_x
                  + (funnel_front_center_x - funnel_exit_center_x)
                    * (0.10 + 0.20 * funnel_curve_strength)),
    funnel_exit_y + funnel_length * funnel_control_2_y_ratio
];

function funnel_curve_points(side) = [
    for (i = [0 : bezier_steps])
        cubic_bezier_point(
            i / bezier_steps,
            funnel_front_point(side),
            funnel_control_1(side),
            funnel_control_2(side),
            funnel_exit_point(side)
        )
];

function unit_2d(v) = v / norm(v);

function funnel_rear_connection_angle(side) =
    side > 0 ? rear_right_front_angle : rear_left_front_angle;

function funnel_rear_connection_point(side) =
    let(a = funnel_rear_connection_angle(side),
        r = barrier_inner_radius + barrier_thickness / 2)
    [capture_center_x + r * cos(a), capture_center_y + r * sin(a)];

function funnel_rear_tangent(side) =
    let(a = funnel_rear_connection_angle(side))
    [side * sin(a), -side * cos(a)];

function funnel_transition_points(side) =
    let(p0 = funnel_exit_point(side),
        p3 = funnel_rear_connection_point(side),
        incoming = unit_2d(p0 - funnel_control_2(side)),
        outgoing = unit_2d(funnel_rear_tangent(side)),
        p1 = p0 + funnel_transition_handle * incoming,
        p2 = p3 - funnel_transition_handle * outgoing)
    [for (i = [1 : funnel_transition_steps])
        cubic_bezier_point(
            i / funnel_transition_steps,
            p0,
            p1,
            p2,
            p3
        )
    ];

function funnel_full_curve_points(side) =
    concat(funnel_curve_points(side), funnel_transition_points(side));

module capsule_2d(p1, p2, width) {
    hull() {
        translate(p1) circle(d = width);
        translate(p2) circle(d = width);
    }
}

module capsule_beam(p1, p2, width, height) {
    linear_extrude(height = height)
        capsule_2d(p1, p2, width);
}

module rod_between(p1, p2, radius) {
    v = p2 - p1;
    length = norm(v);
    axis = cross([0, 0, 1], v);
    axis_length = norm(axis);
    angle = acos(v[2] / length);
    translate(p1)
        rotate(a = angle,
               v = axis_length < 0.0001 ? [1, 0, 0] : axis)
            cylinder(h = length, r = radius, $fn = 28);
}

// ---------------------------
// Required sample preview
// ---------------------------
module sample_preview() {
    color([0.95, 0.72, 0.10, 0.72])
        translate([capture_center[0], capture_center[1], sample_preview_clearance])
            cylinder(h = sample_thickness, d = sample_diameter);
}

module capture_circle_preview() {
    color([0.2, 0.75, 1.0, 0.35])
        translate([capture_center[0], capture_center[1], 0.02])
            linear_extrude(height = 0.15)
                difference() {
                    circle(r = barrier_inner_radius, $fn = 120);
                    circle(r = barrier_inner_radius - 0.35, $fn = 120);
                }
}

module capture_test_disks_preview() {
    offsets = [-15, -10, 0, 10, 15];
    test_y = funnel_front_y + sample_radius - 1;

    for (i = [0 : len(offsets) - 1])
        color([
            0.18 + 0.13 * i,
            0.72 - 0.08 * i,
            0.95 - 0.10 * i,
            0.16
        ])
            translate([
                capture_center_x + offsets[i],
                test_y,
                sample_preview_clearance
            ])
                cylinder(h = sample_thickness,
                         d = sample_diameter,
                         $fn = 72);
}

// ---------------------------
// Required curved barrier modules
// ---------------------------
module curved_barrier_segment(center_angle,
                              arc_angle,
                              inner_radius = barrier_inner_radius,
                              thickness = barrier_thickness,
                              height = barrier_height,
                              bottom_chamfer = barrier_bottom_chamfer) {
    start_angle = center_angle - arc_angle / 2;
    chamfer = min(bottom_chamfer, min(thickness / 2 - 0.05, height / 2));

    rotate([0, 0, start_angle])
        union() {
            if (chamfer > 0)
                rotate_extrude(angle = arc_angle, convexity = 10)
                    polygon([
                        [inner_radius + chamfer, 0],
                        [inner_radius + thickness - chamfer, 0],
                        [inner_radius + thickness, chamfer],
                        [inner_radius + thickness, height],
                        [inner_radius, height],
                        [inner_radius, chamfer]
                    ]);
            else
                rotate_extrude(angle = arc_angle, convexity = 10)
                    translate([inner_radius, 0, 0])
                        square([thickness, height]);
        }
}

module rear_left_barrier() {
    curved_barrier_segment(
        center_angle = rear_left_center_angle,
        arc_angle = rear_barrier_arc
    );
}

module rear_right_barrier() {
    curved_barrier_segment(
        center_angle = rear_right_center_angle,
        arc_angle = rear_barrier_arc
    );
}

module fixed_rear_barriers() {
    union() {
        rear_left_barrier();
        rear_right_barrier();
    }
}

// ---------------------------
// Required front-gate modules
// ---------------------------
module front_gate_support() {
    for (a = front_support_angles)
        rotate([0, 0, a])
            translate([
                front_gate_outer_radius - front_support_radial_width
                    - front_support_radial_inset,
                -front_support_tangential_width / 2,
                front_gate_height - front_support_bottom_overlap
            ])
                cube([
                    front_support_radial_width,
                    front_support_tangential_width,
                    front_support_top_z - front_gate_height
                    + front_support_bottom_overlap
                ]);
}

module front_gate() {
    union() {
        curved_barrier_segment(
            center_angle = front_gate_center_angle,
            arc_angle = front_gate_arc,
            inner_radius = front_gate_inner_radius,
            thickness = front_gate_thickness,
            height = front_gate_height
        );
        front_gate_support();
    }
}

module carrier_support_slots() {
    for (a = front_support_angles) {
        support_center_radius = front_gate_outer_radius
                              - front_support_radial_width / 2
                              - front_support_radial_inset;
        sx = support_center_radius * cos(a);
        sy = support_center_radius * sin(a);
        translate([
            sx,
            sy,
            carrier_bottom_z + carrier_thickness / 2
        ])
            rotate([0, 0, a])
                cube([
                    front_support_radial_width + 2 * carrier_slot_clearance,
                    front_support_tangential_width + 2 * carrier_slot_clearance,
                    carrier_thickness + 0.2
                ], center = true);
    }
}

module gate_guide_sleeve(x_sign = 1) {
    translate([
        x_sign * guide_axis_x - guide_sleeve_outer[0] / 2,
        carrier_y - guide_sleeve_outer[1] / 2,
        carrier_bottom_z
    ])
        difference() {
            cube([
                guide_sleeve_outer[0],
                guide_sleeve_outer[1],
                guide_sleeve_height
            ]);
            translate([
                (guide_sleeve_outer[0] - guide_rail_size[0]) / 2
                    - guide_clearance,
                (guide_sleeve_outer[1] - guide_rail_size[1]) / 2
                    - guide_clearance,
                -0.1
            ])
                cube([
                    guide_rail_size[0] + 2 * guide_clearance,
                    guide_rail_size[1] + 2 * guide_clearance,
                    guide_sleeve_height + 0.2
                ]);
        }
}

module carrier_link_lug() {
    translate([
        carrier_link_x - carrier_lug_size[0] / 2,
        carrier_y - carrier_lug_size[1] / 2,
        carrier_bottom_z
    ])
        difference() {
            cube(carrier_lug_size);
            translate([
                -0.1,
                carrier_lug_size[1] / 2,
                carrier_lug_size[2] / 2
            ])
                rotate([0, 90, 0])
                    cylinder(
                        h = carrier_lug_size[0] + 0.2,
                        r = link_pin_radius + 0.25,
                        $fn = 28
                    );
        }
}

module front_gate_carrier() {
    difference() {
        union() {
            translate([
                -carrier_half_span,
                carrier_y - carrier_beam_width / 2,
                carrier_bottom_z
            ])
                cube([
                    2 * carrier_half_span,
                    carrier_beam_width,
                    carrier_thickness
                ]);
            for (a = front_support_angles) {
                support_center_radius = front_gate_outer_radius
                                      - front_support_radial_width / 2
                                      - front_support_radial_inset;
                translate([
                    support_center_radius * cos(a),
                    support_center_radius * sin(a),
                    carrier_bottom_z + carrier_thickness / 2
                ])
                    cube([
                        carrier_slot_pad_size,
                        carrier_slot_pad_size,
                        carrier_thickness
                    ], center = true);
            }
            gate_guide_sleeve(-1);
            gate_guide_sleeve(1);
            carrier_link_lug();
        }
        carrier_support_slots();
    }
}

// ---------------------------
// Required fixed gate guides
// ---------------------------
module gate_guide_rail_v06(x_sign = 1) {
    rail_height = guide_rail_top_z - guide_rail_bottom_z;
    translate([
        x_sign * guide_axis_x - guide_rail_size[0] / 2,
        guide_axis_y - guide_rail_size[1] / 2,
        guide_rail_bottom_z
    ])
        cube([guide_rail_size[0], guide_rail_size[1], rail_height]);

    translate([
        x_sign * guide_column_center_x - guide_column_size[0] / 2,
        guide_axis_y - guide_column_size[1] / 2,
        side_rail_bottom_z
    ])
        cube([
            guide_column_size[0],
            guide_column_size[1],
            guide_column_height - side_rail_bottom_z
        ]);

    for (z = [guide_rail_bottom_z, guide_rail_top_z - guide_bridge_height])
        hull() {
            translate([
                x_sign * guide_axis_x,
                guide_axis_y,
                z
            ])
                cube([
                    guide_rail_size[0],
                    guide_rail_size[1],
                    guide_bridge_height
                ], center = true);
            translate([
                x_sign * guide_column_center_x,
                guide_axis_y,
                z
            ])
                cube([
                    guide_column_size[0],
                    guide_column_size[1],
                    guide_bridge_height
                ], center = true);
        }
}

module front_gate_guide_left() {
    gate_guide_rail(-1);
}

module front_gate_guide_right() {
    gate_guide_rail(1);
}

// ---------------------------
// Required funnel wall modules
// ---------------------------
module curved_funnel_wall(side = 1) {
    points = funnel_full_curve_points(side);

    intersection() {
        translate([0, 0, funnel_bottom_clearance])
            linear_extrude(height = funnel_wall_height)
                union() {
                    for (i = [0 : len(points) - 2])
                        hull() {
                            translate(points[i])
                                circle(d = funnel_wall_thickness, $fn = 28);
                            translate(points[i + 1])
                                circle(d = funnel_wall_thickness, $fn = 28);
                        }

                    // A larger circular nose removes the sharp front wall end.
                    translate(points[0])
                        circle(r = funnel_tip_radius, $fn = 40);
                }

        // Only the outward, non-contact side of a 95-96 mm nose is trimmed.
        // The disk-facing semicircle stays round while total width stays 100 mm.
        translate([-robot_width / 2, -robot_length / 2, -0.01])
            cube([
                robot_width,
                robot_length,
                funnel_bottom_clearance + funnel_wall_height + 0.02
            ]);
    }
}

module curved_funnel_left() {
    curved_funnel_wall(-1);
}

module curved_funnel_right() {
    curved_funnel_wall(1);
}

// Legacy module names remain as wrappers so existing assemblies do not break.
module funnel_guide_left() {
    curved_funnel_left();
}

module funnel_guide_right() {
    curved_funnel_right();
}

module funnel_side_connectors() {
    for (s = [-1, 1])
        translate([0, 0, funnel_bottom_clearance])
            capsule_beam(
                [capture_center_x + s * funnel_front_center_x, funnel_front_y],
                [s * side_rail_center_x, funnel_front_y],
                funnel_wall_thickness,
                funnel_height
            );
}

module funnel_path_preview() {
    for (side = [-1, 1]) {
        points = funnel_full_curve_points(side);
        controls = [
            funnel_front_point(side),
            funnel_control_1(side),
            funnel_control_2(side),
            funnel_exit_point(side)
        ];

        color([0.05, 0.85, 1.0, 0.75])
            translate([0, 0, funnel_bottom_clearance
                              + funnel_wall_height + 0.25])
                for (i = [0 : len(points) - 2])
                    capsule_beam(points[i], points[i + 1], 0.65, 0.35);

        color([0.95, 0.25, 0.75, 0.55])
            translate([0, 0, funnel_bottom_clearance
                              + funnel_wall_height + 0.7]) {
                for (i = [0 : len(controls) - 2])
                    capsule_beam(controls[i], controls[i + 1], 0.45, 0.25);
                for (p = controls)
                    translate(p)
                        cylinder(h = 0.55, r = 1.05, $fn = 24);
            }
    }
}

// ---------------------------
// Chassis integration geometry
// ---------------------------
module side_rails() {
    for (s = [-1, 1])
        translate([
            s * side_rail_center_x - side_rail_width / 2,
            -robot_length / 2,
            side_rail_bottom_z
        ])
            cube([side_rail_width, robot_length, side_rail_height]);
}

module rear_barrier_side_connectors() {
    for (s = [-1, 1])
        translate([
            capture_center[0],
            capture_center[1],
            barrier_bottom_clearance
        ])
            capsule_beam(
                [s * (barrier_outer_radius - 0.7), 0],
                [s * side_rail_center_x, 0],
                rear_connector_width,
                barrier_height
            );
}

module rear_join_bridge() {
    translate([
        capture_center[0] - rear_join_width / 2,
        capture_center[1] - barrier_outer_radius - rear_join_depth + 1.0,
        barrier_bottom_clearance
    ])
        cube([rear_join_width, rear_join_depth, barrier_height]);
}

module motor_keepout_preview() {
    color([0.9, 0.2, 0.2, 0.22])
        for (s = [-1, 1])
            translate([
                s * motor_keepout_center[0] - motor_keepout_size[0] / 2,
                motor_keepout_center[1] - motor_keepout_size[1] / 2,
                motor_keepout_center[2] - motor_keepout_size[2] / 2
            ])
                cube(motor_keepout_size);
}

module chassis_frame_without_barriers_v06() {
    union() {
        side_rails();
        front_gate_guide_left();
        front_gate_guide_right();
        for (s = [-1, 1])
            translate([
                s * guide_column_center_x - servo_support_post_size[0] / 2,
                servo_mount_arm_y - servo_support_post_size[1] / 2,
                side_rail_bottom_z
            ])
                cube([
                    servo_support_post_size[0],
                    servo_support_post_size[1],
                    guide_column_height - side_rail_bottom_z
                ]);
    }
}

module lower_chassis_with_fixed_barriers() {
    union() {
        chassis_frame_without_barriers();
        translate([
            capture_center[0],
            capture_center[1],
            barrier_bottom_clearance
        ])
            fixed_rear_barriers();
        rear_barrier_side_connectors();
        rear_join_bridge();
        curved_funnel_left();
        curved_funnel_right();
        funnel_side_connectors();
    }
}

// ---------------------------
// Required SG90 mount and previews
// ---------------------------
module servo_mount_v06() {
    local_axis_z = servo_axis_z - servo_mount_base_z;

    union() {
        // Central U cradle, open at the top for servo installation.
        difference() {
            translate([
                servo_body_center_x - servo_mount_outer_width / 2,
                servo_body_center_y - servo_mount_depth / 2,
                0
            ])
                cube([
                    servo_mount_outer_width,
                    servo_mount_depth,
                    servo_mount_back_height
                ]);

            translate([
                servo_body_center_x - servo_mount_inner_width / 2,
                servo_body_center_y - servo_mount_depth / 2
                    + servo_mount_wall,
                servo_mount_wall
            ])
                cube([
                    servo_mount_inner_width,
                    servo_mount_depth - 2 * servo_mount_wall,
                    servo_mount_back_height + 0.1
                ]);

            // Shaft-side access slot.
            translate([servo_axis_x - 0.1, servo_axis_y, local_axis_z])
                rotate([0, 90, 0])
                    cylinder(
                        h = servo_mount_wall + servo_mount_clearance + 0.2,
                        r = servo_shaft_radius + 2,
                        $fn = 32
                    );
        }

        // Two arms tie the mount to the side guide columns.
        for (s = [-1, 1])
            hull() {
                translate([
                    servo_body_center_x + s * servo_mount_outer_width / 2,
                    servo_mount_arm_y,
                    0
                ])
                    cube([
                        servo_mount_wall,
                        7,
                        servo_mount_wall
                    ], center = true);
                translate([
                    s * guide_column_center_x,
                    servo_mount_arm_y,
                    0
                ])
                    cube([
                        guide_column_size[0],
                        7,
                        servo_mount_wall
                    ], center = true);
            }
    }
}

module servo_preview_v06() {
    color([0.12, 0.25, 0.78, 0.78])
        union() {
            translate([
                servo_body_center_x - servo_height / 2,
                servo_body_center_y - servo_length / 2,
                servo_body_center_z - servo_width / 2
            ])
                cube([servo_height, servo_length, servo_width]);

            translate([
                servo_body_center_x - servo_tab_span / 2,
                servo_body_center_y - servo_tab_depth / 2,
                servo_body_center_z - servo_width / 2 - servo_tab_thickness
            ])
                cube([servo_tab_span, servo_tab_depth, servo_tab_thickness]);

            translate([servo_axis_x, servo_axis_y, servo_axis_z])
                rotate([0, 90, 0])
                    cylinder(h = servo_shaft_length,
                             r = servo_shaft_radius,
                             $fn = 32);
        }
}

module linkage_preview_v06() {
    link_point = [
        carrier_link_x,
        carrier_link_y,
        front_gate_bottom_z + carrier_link_local_z
    ];
    horn_point = [
        linkage_plane_x,
        servo_axis_y + servo_crank_radius * cos(servo_crank_angle),
        servo_axis_z + servo_crank_radius * sin(servo_crank_angle)
    ];

    color([0.95, 0.35, 0.08, 0.88]) {
        rod_between([linkage_plane_x, servo_axis_y, servo_axis_z],
                    horn_point,
                    linkage_rod_radius + 0.55);
        rod_between(horn_point, link_point, linkage_rod_radius);
        translate([linkage_plane_x, servo_axis_y, servo_axis_z])
            sphere(r = link_pin_radius + 0.65, $fn = 24);
        translate(horn_point)
            sphere(r = link_pin_radius + 0.45, $fn = 24);
        translate(link_point)
            sphere(r = link_pin_radius + 0.45, $fn = 24);
    }
}

// ===========================================================
// v0.6 UPPER CHASSIS AND DIFFERENTIAL-DRIVE MODULES
// ===========================================================

module rounded_rectangle_2d(width, length, radius) {
    safe_radius = min(radius, min(width, length) / 2 - 0.01);
    hull()
        for (x = [-width / 2 + safe_radius,
                   width / 2 - safe_radius])
            for (y = [-length / 2 + safe_radius,
                       length / 2 - safe_radius])
                translate([x, y])
                    circle(r = safe_radius, $fn = 36);
}

module wheel_cutout_left() {
    translate([-robot_width / 2, motor_center_y, wheel_center_z])
        rotate([0, 90, 0])
            cylinder(h = 4 * wall_thickness,
                     r = wheel_well_radius,
                     center = true,
                     $fn = 72);
}

module wheel_cutout_right() {
    translate([robot_width / 2, motor_center_y, wheel_center_z])
        rotate([0, 90, 0])
            cylinder(h = 4 * wall_thickness,
                     r = wheel_well_radius,
                     center = true,
                     $fn = 72);
}

module rounded_outer_shell() {
    difference() {
        translate([0, 0, upper_shell_bottom_z])
            linear_extrude(height = upper_shell_height, convexity = 10)
                difference() {
                    rounded_rectangle_2d(
                        robot_width,
                        robot_length,
                        corner_radius
                    );
                    rounded_rectangle_2d(
                        robot_width - 2 * wall_thickness,
                        robot_length - 2 * wall_thickness,
                        max(0.6, corner_radius - wall_thickness)
                    );
                }

        wheel_cutout_left();
        wheel_cutout_right();

        // The lower front stays open above the sweeping funnel.
        translate([
            -front_lower_opening_width / 2,
            robot_length / 2 - wall_thickness - 1,
            -0.1
        ])
            cube([
                front_lower_opening_width,
                2 * wall_thickness + 2,
                front_lower_opening_height + 0.1
            ]);

        if (show_gate_window)
            translate([
                -gate_window_width / 2,
                robot_length / 2 - wall_thickness - 1,
                gate_window_bottom_z
            ])
                cube([
                    gate_window_width,
                    2 * wall_thickness + 2,
                    gate_window_height
                ]);

        // Cut only the selected side wall; internal frames remain visible.
        if (cutaway_view)
            translate([
                cutaway_side == "RIGHT"
                    ? robot_width / 2 - wall_thickness - 0.2
                    : -robot_width / 2 - 1,
                -robot_length / 2 + corner_radius,
                upper_shell_bottom_z + 3
            ])
                cube([
                    wall_thickness + 1.2,
                    robot_length - 2 * corner_radius,
                    upper_shell_height - 8
                ]);
    }
}

module lower_mount_holes(z_bottom = -0.1, hole_height = 25) {
    for (x = [-lower_mount_spacing_x / 2, lower_mount_spacing_x / 2])
        for (y = [lower_mount_front_y, lower_mount_rear_y])
            translate([x, y, z_bottom])
                cylinder(h = hole_height,
                         d = lower_mount_hole_diameter,
                         $fn = 32);
}

module lower_mount_tabs() {
    difference() {
        union()
            for (x = [-lower_mount_spacing_x / 2,
                       lower_mount_spacing_x / 2])
                for (y = [lower_mount_front_y, lower_mount_rear_y])
                    translate([
                        x - lower_mount_tab_size / 2,
                        y - lower_mount_tab_size / 2,
                        side_rail_bottom_z
                    ])
                        cube([
                            lower_mount_tab_size,
                            lower_mount_tab_size,
                            side_rail_height
                        ]);

        lower_mount_holes(-0.1, side_rail_height + 1);
    }
}

module lower_mount_interface_v06() {
    for (x = [-lower_mount_spacing_x / 2, lower_mount_spacing_x / 2])
        for (y = [lower_mount_front_y, lower_mount_rear_y])
            translate([x, y, lower_mount_boss_bottom_z])
                difference() {
                    cylinder(h = lower_mount_boss_height,
                             d = lower_mount_boss_outer_diameter,
                             $fn = 40);
                    translate([0, 0, -0.1])
                        cylinder(h = lower_mount_boss_height + 0.2,
                                 d = lower_mount_hole_diameter,
                                 $fn = 30);
                }
}

module lower_chassis_with_upper_mounts() {
    difference() {
        union() {
            lower_chassis_with_fixed_barriers();
            lower_mount_tabs();
        }
        lower_mount_holes(-0.1, lower_mount_boss_bottom_z + 1);
        // Recess flat-head screws so they stay above the floor.
        for (x = [-lower_mount_spacing_x / 2, lower_mount_spacing_x / 2])
            for (y = [lower_mount_front_y, lower_mount_rear_y])
                translate([x, y, side_rail_bottom_z - 0.01])
                    cylinder(d1 = flush_head_diameter, d2 = lower_mount_hole_diameter,
                             h = flush_head_depth + 0.01, $fn = 32);
    }
}

module motor_mount_right_v06() {
    mount_inner_x = motor_inner_x - motor_clearance - motor_mount_wall;
    mount_outer_x = motor_outer_x + motor_clearance;
    mount_length_x = mount_outer_x - mount_inner_x;
    mount_half_y = motor_width / 2 + motor_clearance + motor_mount_wall;
    side_wall_y = motor_width / 2 + motor_clearance
                + motor_mount_wall / 2;
    wall_top_z = motor_top_z + motor_clearance;
    hanger_bottom_z = wall_top_z;
    hanger_top_z = deck_height - motor_mount_wall;

    difference() {
        union() {
            // Thin floor, two side rails and an inner insertion stopper.
            translate([
                mount_inner_x,
                motor_center_y - mount_half_y,
                motor_mount_base_z
            ])
                cube([
                    mount_length_x,
                    2 * mount_half_y,
                    motor_mount_floor_thickness
                ]);

            for (sy = [-1, 1])
                translate([
                    mount_inner_x,
                    motor_center_y + sy * side_wall_y
                        - motor_mount_wall / 2,
                    motor_mount_base_z
                ])
                    cube([
                        mount_length_x,
                        motor_mount_wall,
                        wall_top_z - motor_mount_base_z
                    ]);

            translate([
                mount_inner_x,
                motor_center_y - mount_half_y,
                motor_bottom_z
            ])
                cube([
                    motor_mount_wall,
                    2 * mount_half_y,
                    motor_height + motor_clearance
                ]);

            // Two hanger ears bolt upward into the removable electronics deck.
            for (sy = [-1, 1]) {
                translate([
                    motor_center_x - 3,
                    motor_center_y + sy * side_wall_y
                        - motor_mount_wall / 2,
                    hanger_bottom_z
                ])
                    cube([
                        6,
                        motor_mount_wall,
                        hanger_top_z - hanger_bottom_z
                    ]);

                translate([
                    motor_center_x - 5,
                    motor_center_y + sy * side_wall_y - 3.5,
                    hanger_top_z
                ])
                    cube([10, 7, motor_mount_wall]);
            }
        }

        // Cable-tie passages through the cradle floor.
        for (x = [motor_center_x - 6, motor_center_x + 6])
            for (sy = [-1, 1])
                translate([
                    x - 2,
                    motor_center_y + sy * (motor_width / 2 + 0.3) - 1.5,
                    motor_mount_base_z - 0.1
                ])
                    cube([4, 3, motor_mount_floor_thickness + 0.2]);

        // Vertical M3 holes in the two deck hanger flanges.
        for (sy = [-1, 1])
            translate([
                motor_center_x,
                motor_center_y + sy * side_wall_y,
                hanger_top_z - 0.1
            ])
                cylinder(h = motor_mount_wall + 0.2,
                         d = electronics_hole_diameter,
                         $fn = 30);
    }
}

module motor_mount_left() {
    mirror([1, 0, 0])
        motor_mount_right();
}

module motor_preview_right() {
    color([0.62, 0.64, 0.68, 0.78]) {
        translate([
            motor_center_x - motor_length / 2,
            motor_center_y - motor_width / 2,
            wheel_center_z - motor_height / 2
        ])
            cube([motor_length, motor_width, motor_height]);

        color([0.75, 0.76, 0.78, 0.85])
            translate([motor_outer_x, motor_center_y, wheel_center_z])
                rotate([0, 90, 0])
                    cylinder(h = motor_shaft_length,
                             d = motor_shaft_diameter,
                             $fn = 28);
    }
}

module motor_preview_left() {
    mirror([1, 0, 0])
        motor_preview_right();
}

module wheel_preview_right() {
    color([0.08, 0.09, 0.10, 0.80])
        translate([wheel_center_x, motor_center_y, wheel_center_z])
            rotate([0, 90, 0])
                cylinder(h = wheel_width,
                         d = wheel_diameter,
                         center = true,
                         $fn = 72);
}

module wheel_preview_left() {
    mirror([1, 0, 0])
        wheel_preview_right();
}

module rear_caster_mount_v06() {
    difference() {
        union() {
            translate([0, (caster_back_y + caster_front_y) / 2, caster_height])
                linear_extrude(height = caster_plate_thickness)
                    rounded_rectangle_2d(
                        caster_mount_width,
                        caster_front_y - caster_back_y,
                        2
                    );
            // Rear upright bolts horizontally into the shell; 0.5 mm fit gap.
            translate([-caster_mount_width / 2, caster_back_y, caster_height])
                cube([caster_mount_width, 3, caster_flange_height]);
            // Separate battery tray rests on these two short spacing pads.
            for (sx = [-1, 1])
                translate([sx * battery_width / 4, battery_rear_bolt_y,
                           caster_height + caster_plate_thickness - 0.1])
                    cylinder(d = 6, h = battery_to_caster_z_clearance + 0.1,
                             $fn = 32);
        }

        for (x = [-caster_mount_hole_spacing / 2,
                   caster_mount_hole_spacing / 2])
            translate([
                x,
                caster_center_y,
                caster_height - 0.1
            ])
                cylinder(h = caster_plate_thickness + 0.2,
                         d = 3.4,
                         $fn = 30);

        caster_wall_bolt_holes();
        for (sx = [-1, 1])
            translate([sx * battery_width / 4, battery_rear_bolt_y,
                       caster_height - 0.1])
                cylinder(d = 3.4,
                         h = battery_mount_base_z - caster_height + 0.2,
                         $fn = 30);
    }
}

module caster_wall_bolt_holes() {
    for (x = [-caster_mount_hole_spacing / 2,
               caster_mount_hole_spacing / 2])
        translate([x, -robot_length / 2 - 0.1, caster_wall_bolt_z])
            rotate([-90, 0, 0])
                cylinder(d = 3.4, h = wall_thickness + 4.2, $fn = 30);
}

module caster_preview() {
    color([0.18, 0.18, 0.20, 0.55])
        translate([0, caster_center_y, caster_diameter / 2])
            sphere(d = caster_diameter, $fn = 48);
}

module battery_mount_v06() {
    tray_half_x = battery_width / 2 + battery_clearance;
    tray_half_y = battery_length / 2 + battery_clearance;
    rail_width = 5;

    difference() {
        union() {
            // Two open rails support the battery without making a solid floor.
            for (sx = [-1, 1])
                translate([
                    sx * (battery_width / 4) - rail_width / 2,
                    battery_center_y - tray_half_y,
                    battery_mount_base_z
                ])
                    cube([
                        rail_width,
                        2 * tray_half_y,
                        battery_tray_thickness
                    ]);

            // Front/rear cross bars are outside the motor overlap zone.
            for (sy = [-1, 1])
                translate([
                    -tray_half_x,
                    battery_center_y + sy * (tray_half_y - rail_width / 2)
                        - rail_width / 2,
                    battery_mount_base_z
                ])
                    cube([
                        2 * tray_half_x,
                        rail_width,
                        battery_tray_thickness
                    ]);

            // Short end stops; the Velcro strap provides side retention.
            for (sy = [-1, 1])
                translate([
                    -battery_width / 2,
                    battery_center_y + sy * tray_half_y
                        - (sy < 0 ? battery_stop_wall : 0)
                        + (sy < 0 ? 0.2 : -0.2),
                    battery_mount_base_z
                ])
                    cube([
                        battery_width,
                        battery_stop_wall,
                        battery_tray_thickness + 5
                    ]);

            // Twin rear tongues bolt onto the removable caster bracket.
            for (sx = [-1, 1])
                translate([
                    sx * battery_width / 4 - 4,
                    battery_rear_bolt_y - 3,
                    battery_mount_base_z
                ])
                    cube([
                        8,
                        battery_center_y - tray_half_y
                            - (battery_rear_bolt_y - 3) + 0.5,
                        battery_tray_thickness
                    ]);

            // Front pair hangs from the deck; the rear pair bolts to caster.
            translate([-battery_front_hanger_x - 3,
                       battery_center_y + tray_half_y - rail_width,
                       battery_mount_base_z])
                cube([2 * battery_front_hanger_x + 6,
                      rail_width, battery_tray_thickness]);
            for (sx = [-1, 1])
                translate([sx * battery_front_hanger_x - 3,
                           battery_front_hanger_y - 3, battery_mount_base_z])
                    cube([6, 6, deck_height - battery_mount_base_z]);
        }

        // Two pairs of slots accept 8 mm Velcro or cable-tie straps.
        for (y = [battery_center_y - battery_length / 4,
                  battery_center_y + battery_length / 4])
            for (sx = [-1, 1])
                translate([
                    sx * battery_width / 4 - 1,
                    y - battery_strap_width / 2,
                    battery_mount_base_z - 0.1
                ])
                    cube([2, battery_strap_width,
                          battery_tray_thickness + 0.2]);

        for (sx = [-1, 1])
            translate([sx * battery_width / 4, battery_rear_bolt_y,
                       battery_mount_base_z - 0.1])
                cylinder(d = 3.4, h = battery_tray_thickness + 0.2, $fn = 30);
        for (sx = [-1, 1])
            translate([sx * battery_front_hanger_x, battery_front_hanger_y,
                       deck_height - 9])
                cylinder(d = 2.7, h = 9.1, $fn = 30); // pilot, tap M3
    }
}

module battery_preview() {
    color([0.22, 0.72, 0.30, 0.55])
        translate([
            -battery_width / 2,
            battery_center_y - battery_length / 2,
            battery_bottom_z
        ])
            cube([battery_width, battery_length, battery_height]);
}

module controller_mount() {
    rail_width = 8;
    rail_length = 40;

    difference() {
        union() {
            for (sx = [-1, 1])
                translate([
                    sx * 20 - rail_width / 2,
                    controller_center_y - rail_length / 2,
                    deck_height
                ])
                    cube([rail_width, rail_length, deck_thickness]);

            for (sy = [-1, 1])
                translate([
                    -24,
                    controller_center_y + sy * (rail_length / 2
                        - rail_width / 2) - rail_width / 2,
                    deck_height
                ])
                    cube([48, rail_width, deck_thickness]);
        }

        for (x = [-2 : 1 : 2])
            for (y = [-1.5 : 1 : 1.5])
                if (!(abs(x) == 2 && (y == -1.5 || y == 1.5)))
                translate([x * electronics_hole_spacing,
                           controller_center_y + y * electronics_hole_spacing,
                           deck_height - 0.1])
                    cylinder(h = deck_thickness + 0.2,
                             d = electronics_hole_diameter,
                             $fn = 28);
    }
}

module electronics_deck_v06() {
    difference() {
        union() {
            translate([0, deck_center_y, deck_height])
                linear_extrude(height = deck_thickness, convexity = 10)
                    difference() {
                        square([deck_width, deck_length], center = true);
                        square([
                            deck_width - 2 * deck_frame_width,
                            deck_length - 2 * deck_frame_width
                        ], center = true);
                    }

            controller_mount();

            // Tie controller rails into the front perimeter (rear cable holes
            // otherwise isolate the central frame from the outer deck).
            for (sx = [-1, 1])
                translate([sx * 20 - 4, 0, deck_height])
                    cube([8, deck_front_y, deck_thickness]);

            // Rails above the motor hanger flanges.
            for (sx = [-1, 1])
                translate([
                    sx * motor_center_x - 4,
                    deck_center_y - deck_length / 2,
                    deck_height
                ])
                    cube([8, deck_length, deck_thickness]);
        }

        // Three large cable-routing openings in the rear deck rail.
        for (x = [-28, 0, 28])
            translate([
                x,
                deck_center_y - deck_length / 2 + deck_frame_width / 2,
                deck_height - 0.1
            ])
                cylinder(h = deck_thickness + 0.2,
                         d = 7,
                         $fn = 32);

        // Holes matching the motor-mount hanger flanges.
        for (sx = [-1, 1])
            for (sy = [-1, 1])
                translate([
                    sx * motor_center_x,
                    motor_center_y
                        + sy * (motor_width / 2 + motor_clearance
                                + motor_mount_wall / 2),
                    deck_height - 0.1
                ])
                    cylinder(h = deck_thickness + 0.2,
                             d = electronics_hole_diameter,
                             $fn = 28);

        // Four holes line up with the shell ledges, with full edge material.
        for (sx = [-1, 1])
            for (y = [deck_center_y - deck_length / 2 + 4,
                      deck_center_y + deck_length / 2 - 4])
                translate([sx * deck_mount_x, y, deck_height - 0.1])
                    cylinder(h = deck_thickness + 0.2,
                             d = electronics_hole_diameter, $fn = 28);
        for (sx = [-1, 1])
            translate([sx * battery_front_hanger_x, battery_front_hanger_y,
                       deck_height - 0.1])
                cylinder(d = 3.4, h = deck_thickness + 0.2, $fn = 30);
    }
}

module controller_preview() {
    color([0.10, 0.48, 0.20, 0.52])
        translate([
            -controller_width / 2,
            controller_center_y - controller_length / 2,
            deck_height + deck_thickness + controller_standoff_height
        ])
            cube([
                controller_width,
                controller_length,
                controller_height
            ]);
}

module front_gate_keepout_preview() {
    color([1.0, 0.18, 0.12, 0.16])
        translate([
            capture_center_x - front_gate_keepout_width / 2,
            front_gate_keepout_center_y - front_gate_keepout_depth / 2,
            0
        ])
            cube([
                front_gate_keepout_width,
                front_gate_keepout_depth,
                front_gate_keepout_height
            ]);
}

module gate_guide_support() {
    support_bottom_z = lower_gate_mechanism_top_z + 4;
    for (sx = [-1, 1])
        hull() {
            translate([
                sx * 45.5 - 4.5,
                guide_axis_y - 5,
                support_bottom_z
            ])
                cube([9, 10, 3]);
            translate([
                sx * 45.5 - 4.5,
                guide_axis_y - 6,
                support_bottom_z + 4
            ])
                cube([9, 12, 3]);
        }
}

module deck_support_ledges_v06() {
    for (sx = [-1, 1])
        for (y = [deck_center_y - deck_length / 2 + 4,
                  deck_center_y + deck_length / 2 - 4])
            difference() {
                translate([
                    sx * 42.75 - 4.75,
                    y - 5,
                    deck_height - 3
                ])
                    cube([9.5, 10, 3]);
                translate([sx * deck_mount_x, y, deck_height - 3.1])
                    cylinder(h = 3.2,
                             d = electronics_hole_diameter,
                             $fn = 28);
            }
}

module lid_screw_bosses_v06() {
    boss_height = upper_shell_top_z - lid_boss_bottom_z;
    for (x = [-lid_boss_spacing_x / 2, lid_boss_spacing_x / 2])
        for (y = [-lid_boss_spacing_y / 2, lid_boss_spacing_y / 2])
            translate([x, y, lid_boss_bottom_z])
                difference() {
                    union() {
                        cylinder(h = boss_height,
                                 d = boss_outer_diameter,
                                 $fn = 40);
                        linear_extrude(height = boss_height)
                            capsule_2d(
                                [0, 0],
                                [sign(x) * (robot_width / 2
                                            - wall_thickness / 2
                                            - abs(x)), 0],
                                3
                            );
                    }
                    translate([0, 0, -0.1])
                        cylinder(h = boss_height + 0.2,
                                 d = boss_hole_diameter,
                                 $fn = 30);
                }
}

module top_lid() {
    difference() {
        linear_extrude(height = lid_thickness)
            rounded_rectangle_2d(
                robot_width,
                robot_length,
                corner_radius
            );

        for (x = [-lid_boss_spacing_x / 2, lid_boss_spacing_x / 2])
            for (y = [-lid_boss_spacing_y / 2, lid_boss_spacing_y / 2])
                union() {
                    translate([x, y, -0.1])
                        cylinder(h = lid_thickness + 0.2,
                                 d = lid_hole_diameter, $fn = 30);
                    translate([x, y, lid_thickness - flush_head_depth])
                        cylinder(h = flush_head_depth + 0.01,
                                 d1 = lid_hole_diameter,
                                 d2 = flush_head_diameter, $fn = 32);
                }

        // Lightweight ventilation slots over the controller area.
        for (x = [-18, -6, 6, 18])
            translate([x - 1.5, controller_center_y - 14, -0.1])
                cube([3, 28, lid_thickness + 0.2]);
    }
}

module rear_panel() {
    // Solid reinforcement only; future switch/USB holes can be added here.
    translate([
        -rear_panel_width / 2,
        -robot_length / 2 + wall_thickness - 0.5,
        rear_panel_bottom_z
    ])
        cube([
            rear_panel_width,
            rear_panel_reinforcement,
            rear_panel_height
        ]);
}

module sensor_mount_area() {
    difference() {
        translate([
            -sensor_plate_width / 2,
            robot_length / 2 - wall_thickness - sensor_plate_depth,
            sensor_plate_z
        ])
            cube([
                sensor_plate_width,
                sensor_plate_depth + 0.5,
                sensor_plate_thickness
            ]);

        for (x = [-sensor_hole_spacing / 2, sensor_hole_spacing / 2])
            translate([
                x,
                robot_length / 2 - wall_thickness - sensor_plate_depth / 2,
                sensor_plate_z - 0.1
            ])
                cylinder(h = sensor_plate_thickness + 0.2,
                         d = sensor_hole_diameter,
                         $fn = 30);
    }
}

module main_upper_chassis() {
    difference() {
        union() {
            rounded_outer_shell();
            lower_mount_interface();
            gate_guide_support();
            deck_support_ledges();
            lid_screw_bosses();
            rear_panel();
            sensor_mount_area();
        }
        lower_post_clearance_pockets();
        gate_support_arm_slots();
        caster_wall_bolt_holes();
        // Rear bolt heads also stay within the nominal body length.
        for (x = [-caster_mount_hole_spacing / 2,
                   caster_mount_hole_spacing / 2])
            translate([x, -robot_length / 2 - 0.01, caster_wall_bolt_z])
                rotate([-90, 0, 0])
                    cylinder(d1 = flush_head_diameter, d2 = 3.4,
                             h = flush_head_depth + 0.01, $fn = 32);
    }
}

module lower_post_clearance_pockets_v06() {
    // Preserve the four tall v0.5 posts. The upper shell slides over them.
    // These open-bottom pockets also free the original servo support arms.
    for (sx = [-1, 1]) {
        translate([sx * guide_column_center_x - guide_column_size[0] / 2
                      - gate_guide_clearance,
                   guide_axis_y - guide_column_size[1] / 2 - gate_guide_clearance,
                   -0.1])
            cube([guide_column_size[0] + 2 * gate_guide_clearance,
                  guide_column_size[1] + 2 * gate_guide_clearance,
                  guide_column_height + gate_guide_clearance + 0.1]);
        translate([sx * guide_column_center_x - servo_support_post_size[0] / 2
                      - gate_guide_clearance,
                   servo_mount_arm_y - servo_support_post_size[1] / 2
                       - gate_guide_clearance,
                   -0.1])
            cube([servo_support_post_size[0] + 2 * gate_guide_clearance,
                  servo_support_post_size[1] + 2 * gate_guide_clearance,
                  guide_column_height + gate_guide_clearance + 0.1]);
    }
}

module gate_support_arm_slots() {
    // Explicit left/right vertical arm tunnels, matched to the existing arms.
    for (a = front_support_angles) {
        r = front_gate_outer_radius - front_support_radial_width / 2
            - front_support_radial_inset;
        translate([capture_center_x + r * cos(a),
                   capture_center_y + r * sin(a), -0.1])
            rotate([0, 0, a])
                linear_extrude(height = front_gate_up_bottom_z
                                        + front_support_top_z + 0.2)
                    square([front_support_radial_width + 2 * gate_guide_clearance,
                            front_support_tangential_width
                                + 2 * gate_guide_clearance], center = true);
    }
}

module upper_assembly() {
    color([0.34, 0.39, 0.46, 1.0])
        main_upper_chassis();
    color([0.42, 0.47, 0.54, 1.0]) {
        motor_mount_left();
        motor_mount_right();
        battery_mount();
        electronics_deck();
        rear_caster_mount();
    }
    if (show_lid)
        color([0.56, 0.61, 0.68, 1.0])
            translate([0, 0, upper_shell_top_z])
                top_lid();

    if (show_motors) {
        %motor_preview_left();
        %motor_preview_right();
    }
    if (show_wheels) {
        %wheel_preview_left();
        %wheel_preview_right();
    }
    if (show_battery)
        %battery_preview();
    if (show_controller)
        %controller_preview();
    if (show_caster)
        %caster_preview();
    if (show_gate_keepout)
        %front_gate_keepout_preview();
}

module lower_capture_system_for_upper() {
    color([0.42,0.47,0.54]) { installed_guide_caps(); installed_servo_clamp(); }
    color([0.9,0.45,0.12]) printed_link_pose();
    color([0.68, 0.72, 0.78, 1.0])
        lower_chassis_with_upper_mounts();
    color([0.15, 0.72, 0.35, 1.0])
        moving_front_gate_system();
    color([0.38, 0.42, 0.48, 1.0])
        translate([0, 0, servo_mount_base_z])
            servo_mount();

    if (show_sample)
        %sample_preview();
    if (show_servo && show_gate_mechanism)
        %servo_preview();
    if (show_linkage && show_gate_mechanism)
        %linkage_preview();
    if (show_funnel_path)
        %funnel_path_preview();
    if (show_capture_test_disks)
        %capture_test_disks_preview();
}

module final_robot_assembly() {
    if (show_lower_capture_system)
        lower_capture_system_for_upper();
    upper_assembly();
}

// ---------------------------
// Required capture-system modules
// ---------------------------
module moving_front_gate_system_v06() {
    translate([
        capture_center[0],
        capture_center[1],
        front_gate_bottom_z
    ]) {
        front_gate();
        front_gate_carrier();
    }
}

module capture_system() {
    union() {
        installed_guide_caps(); installed_servo_clamp(); printed_link_pose();
        lower_chassis_with_fixed_barriers();
        moving_front_gate_system();
        translate([0, 0, servo_mount_base_z])
            servo_mount();
    }
}

module capture_system_preview() {
    color([0.68, 0.72, 0.78, 1.0])
        lower_chassis_with_fixed_barriers();

    color([0.15, 0.72, 0.35, 1.0])
        moving_front_gate_system();

    color([0.38, 0.42, 0.48, 1.0])
        translate([0, 0, servo_mount_base_z])
            servo_mount();

    if (show_sample)
        %sample_preview();
    if (show_servo)
        %servo_preview();
    if (show_linkage)
        %linkage_preview();
    if (show_capture_circle)
        %capture_circle_preview();
    if (show_funnel_path)
        %funnel_path_preview();
    if (show_capture_test_disks)
        %capture_test_disks_preview();

    if (show_motor_keepouts)
        %motor_keepout_preview();
}

// ---------------------------
// Part selector
// ---------------------------
// v0.7 manufacturing joints. All coordinates remain in the assembly frame.
module hex_nut_cut(height = nut_pocket_height, af = nut_af, angle = 30) {
    rotate([0,0,angle]) cylinder(h=height, d=af/cos(30), $fn=6);
}
module nut_entry_y(x, y, z, direction=1) {
    translate([x,y,z]) {
        hex_nut_cut(angle=0);
        translate([-3.45, direction>0 ? 0 : -5.1, 0])
            cube([6.9,5.1,nut_pocket_height]);
    }
}
module gate_guide_rail(x_sign=1) {
    difference() {
        union() {
            translate([x_sign*guide_axis_x-guide_rail_size[0]/2,
                       guide_axis_y-guide_rail_size[1]/2,guide_rail_bottom_z])
                cube([guide_rail_size[0],guide_rail_size[1],
                      guide_rail_top_z-guide_rail_bottom_z]);
            translate([x_sign*guide_column_center_x-guide_column_size[0]/2,
                       guide_axis_y-guide_column_size[1]/2,side_rail_bottom_z])
                cube([guide_column_size[0],guide_column_size[1],
                      guide_column_height-side_rail_bottom_z]);
            hull() for(x=[guide_axis_x,guide_column_center_x])
                translate([x_sign*x,guide_axis_y,guide_rail_bottom_z])
                    cube([guide_rail_size[0],guide_rail_size[1],
                          guide_bridge_height],center=true);
        }
        translate([x_sign*guide_column_center_x,guide_axis_y,guide_column_height-8])
            cylinder(d=small_pilot,h=8.1,$fn=28);
    }
}
module guide_cap() {
    span=guide_column_center_x-guide_axis_x;
    difference() {
        union() {
            hull() for(x=[0,span]) translate([x,0,guide_cap_thickness/2])
                cube([6,8,guide_cap_thickness],center=true);
            translate([-4,-5,-guide_cap_socket_depth])
                cube([8,10,guide_cap_socket_depth+guide_cap_thickness]);
        }
        translate([-guide_rail_size[0]/2-guide_cap_clearance,
                   -guide_rail_size[1]/2-guide_cap_clearance,
                   -guide_cap_socket_depth-0.1])
            cube([guide_rail_size[0]+2*guide_cap_clearance,
                  guide_rail_size[1]+2*guide_cap_clearance,
                  guide_cap_socket_depth+0.11]);
        translate([span,0,-0.1]) cylinder(d=small_bolt_clearance,
                                             h=guide_cap_thickness+0.2,$fn=28);
    }
}
module installed_guide_caps() {
    translate([guide_axis_x,guide_axis_y,guide_column_height]) guide_cap();
    translate([-guide_axis_x,guide_axis_y,guide_column_height])
        mirror([1,0,0]) guide_cap();
}
module chassis_frame_without_barriers() {
    union() {
        side_rails(); front_gate_guide_left(); front_gate_guide_right();
        for(s=[-1,1]) difference() {
            translate([s*guide_column_center_x-servo_support_post_size[0]/2,
                       servo_mount_arm_y-servo_support_post_size[1]/2,
                       side_rail_bottom_z])
                cube([servo_support_post_size[0],servo_support_post_size[1],
                      servo_post_top_z-side_rail_bottom_z]);
            translate([s*guide_column_center_x,servo_mount_arm_y,servo_post_top_z-8])
                cylinder(d=small_pilot,h=8.1,$fn=28);
        }
    }
}
module gate_slider() {
    difference() {
        union() {
            front_gate(); front_gate_carrier();
            for(a=front_support_angles) {
                r=front_gate_outer_radius-front_support_radial_width/2-front_support_radial_inset;
                translate([r*cos(a),r*sin(a),carrier_bottom_z]) rotate([0,0,a])
                    translate([-3.5,-4,0]) cube([7,8,carrier_thickness]);
            }
        }
        // The carrier beam crosses the sleeves. Recut the guide passages AFTER
        // joining everything so that the beam cannot refill the guide bores.
        for(s=[-1,1]) translate([s*guide_axis_x-guide_rail_size[0]/2-guide_clearance,
             carrier_y-guide_rail_size[1]/2-guide_clearance,carrier_bottom_z-0.1])
            cube([guide_rail_size[0]+2*guide_clearance,
                  guide_rail_size[1]+2*guide_clearance,guide_sleeve_height+0.2]);
    }
}
module moving_front_gate_system() {
    translate([capture_center_x,capture_center_y,front_gate_bottom_z]) gate_slider();
}
module servo_mount() {
    difference() {
        union() {
            servo_mount_v06();
            for(s=[-1,1]) translate([servo_body_center_x,
                servo_body_center_y+s*servo_clamp_bolt_spacing/2,0])
                    cylinder(d=6,h=servo_mount_back_height,$fn=36);
        }
        for(s=[-1,1]) {
            translate([s*guide_column_center_x,servo_mount_arm_y,-servo_mount_wall/2-0.1])
                cylinder(d=small_bolt_clearance,h=servo_mount_wall+0.2,$fn=28);
            translate([servo_body_center_x,servo_body_center_y+s*servo_clamp_bolt_spacing/2,
                       servo_mount_back_height-8])
                cylinder(d=small_pilot,h=8.1,$fn=28);
        }
        // Clear the horn and its rear pin head, not just the servo shaft.
        translate([servo_mount_inner_width/2-0.05,servo_axis_y,
                   servo_axis_z-servo_mount_base_z]) rotate([0,90,0])
            cylinder(r=servo_crank_radius+3,h=servo_mount_wall+0.2,$fn=64);
    }
}
module servo_clamp() {
    // Local print part: bridge at z=0..3, contact pad projects downward.
    pad_drop=servo_mount_back_height-servo_mount_wall-servo_width-0.4;
    difference() {
        union() {
            translate([servo_body_center_x,servo_body_center_y,0])
                linear_extrude(height=servo_clamp_thickness)
                    capsule_2d([0,-servo_clamp_bolt_spacing/2],[0,servo_clamp_bolt_spacing/2],6);
            translate([servo_body_center_x-3,servo_body_center_y-servo_length/4,-pad_drop])
                cube([6,servo_length/2,pad_drop+0.1]);
        }
        for(s=[-1,1]) translate([servo_body_center_x,
            servo_body_center_y+s*servo_clamp_bolt_spacing/2,-0.1])
                cylinder(d=small_bolt_clearance,h=servo_clamp_thickness+0.2,$fn=28);
    }
}
module installed_servo_clamp() {
    translate([0,0,servo_mount_base_z+servo_mount_back_height]) servo_clamp();
}
module servo_preview() {
    // Body/shaft only. Real mounting ears and connector need measurement.
    color([0.12,0.25,0.78,0.78]) {
        translate([servo_body_center_x-servo_height/2,servo_body_center_y-servo_length/2,
                   servo_body_center_z-servo_width/2])
            cube([servo_height,servo_length,servo_width]);
        translate([servo_axis_x,servo_axis_y,servo_axis_z]) rotate([0,90,0])
            cylinder(h=servo_shaft_length,r=servo_shaft_radius,$fn=32);
    }
}
function link_horn_point(angle=servo_crank_angle) =
    [servo_axis_y+servo_crank_radius*cos(angle),servo_axis_z+servo_crank_radius*sin(angle)];
function gate_z_at_angle(angle) = let(h=link_horn_point(angle),
    dy=h[0]-carrier_link_y)
    h[1]-sqrt(linkage_length*linkage_length-dy*dy)-carrier_link_local_z;
module connecting_link() {
    difference() {
        linear_extrude(height=link_plate_thickness)
            capsule_2d([0,0],[0,linkage_length],link_end_diameter);
        for(y=[0,linkage_length]) translate([0,y,-0.1])
            cylinder(d=small_bolt_clearance,h=link_plate_thickness+0.2,$fn=32);
    }
}
module printed_link_pose(angle=servo_crank_angle, gate_z=front_gate_bottom_z) {
    p=[carrier_link_y,gate_z+carrier_link_local_z];
    h=link_horn_point(angle);
    dy=(h[0]-p[0])/linkage_length; dz=(h[1]-p[1])/linkage_length;
    multmatrix([[0,0,1,linkage_plane_x-link_plate_thickness/2],
                [dz,dy,0,p[0]],[-dy,dz,0,p[1]],[0,0,0,1]]) connecting_link();
}
module linkage_preview() {
    // Purchased horn on the servo spline; the connecting link is printable.
    h=link_horn_point();
    color([0.95,0.35,0.08,0.8])
        translate([servo_axis_x+servo_shaft_length,servo_axis_y,servo_axis_z])
            rotate([0,90,0]) rotate([0,0,servo_crank_angle+90])
                linear_extrude(height=2)
                    capsule_2d([0,0],[servo_crank_radius,0],4);
    color([0.7,0.7,0.7,0.7])
        translate([servo_axis_x+servo_shaft_length+2,h[0],h[1]]) rotate([0,90,0])
            cylinder(d=4.5,h=4,$fn=24); // purchased spacer stack, nominal
}
module lower_mount_interface() {
    difference() {
        lower_mount_interface_v06();
        for(x=[-lower_mount_spacing_x/2,lower_mount_spacing_x/2])
            for(y=[lower_mount_front_y,lower_mount_rear_y])
                translate([x,y,lower_mount_boss_bottom_z+lower_mount_boss_height-nut_pocket_height])
                    hex_nut_cut(height=nut_pocket_height+0.1);
    }
}
module motor_mount_right() {
    wall_y=motor_width/2+motor_clearance+motor_mount_wall/2;
    difference() {
        union() {
            motor_mount_right_v06();
            for(s=[-1,1]) translate([motor_center_x-5,motor_center_y+s*wall_y-4.5,
                                    deck_height-7]) cube([10,9,7]);
        }
        for(s=[-1,1]) {
            translate([motor_center_x,motor_center_y+s*wall_y,deck_height-9])
                cylinder(d=3.4,h=9.1,$fn=28);
            nut_entry_y(motor_center_x,motor_center_y+s*wall_y,deck_height-6.5,s);
        }
    }
}
module battery_mount() {
    hx=battery_width/2+battery_clearance;
    hy=battery_length/2+battery_clearance;
    difference() {
        union() {
            for(s=[-1,1]) translate([s*battery_width/4-2.5,battery_center_y-hy,battery_mount_base_z])
                cube([5,2*hy,battery_tray_thickness]);
            for(s=[-1,1]) {
                translate([-battery_front_hanger_x-battery_hanger_size/2,
                           battery_center_y+s*(hy-2.5)-2.5,battery_mount_base_z])
                    cube([2*battery_front_hanger_x+battery_hanger_size,5,battery_tray_thickness]);
                translate([-battery_width/2,battery_center_y+s*hy
                    -(s<0 ? battery_stop_wall : 0)+(s<0 ? 0.2 : -0.2),battery_mount_base_z])
                    cube([battery_width,battery_stop_wall,battery_tray_thickness+5]);
            }
            for(sx=[-1,1]) for(y=[battery_front_hanger_y,battery_rear_hanger_y])
                translate([sx*battery_front_hanger_x-battery_hanger_size/2,
                           y-battery_hanger_size/2,battery_mount_base_z])
                    cube([battery_hanger_size,battery_hanger_size,deck_height-battery_mount_base_z]);
        }
        for(y=[battery_center_y-battery_length/4,battery_center_y+battery_length/4])
            for(s=[-1,1]) translate([s*battery_width/4-1,y-battery_strap_width/2,battery_mount_base_z-0.1])
                cube([2,battery_strap_width,battery_tray_thickness+0.2]);
        for(sx=[-1,1]) for(y=[battery_front_hanger_y,battery_rear_hanger_y]) {
            translate([sx*battery_front_hanger_x,y,deck_height-9]) cylinder(d=3.4,h=9.1,$fn=28);
            nut_entry_y(sx*battery_front_hanger_x,y,deck_height-7,
                        y==battery_front_hanger_y ? 1 : -1);
        }
    }
}
module electronics_deck() {
    difference() {
        electronics_deck_v06();
        for(sx=[-1,1]) translate([sx*battery_front_hanger_x,battery_rear_hanger_y,deck_height-0.1])
            cylinder(d=3.4,h=deck_thickness+0.2,$fn=28);
    }
}
module deck_support_ledges() {
    for(s=[-1,1]) for(y=[deck_center_y-deck_length/2+4,deck_center_y+deck_length/2-4])
        difference() {
            translate([s*41.5-6,y-5,deck_height-deck_ledge_thickness])
                cube([12,10,deck_ledge_thickness]);
            translate([s*deck_mount_x,y,deck_height-deck_ledge_thickness-0.1]) {
                cylinder(d=3.4,h=deck_ledge_thickness+0.2,$fn=28);
                hex_nut_cut(height=nut_pocket_height+0.1);
            }
        }
}
module lid_screw_bosses() {
    difference() {
        lid_screw_bosses_v06();
        for(x=[-lid_boss_spacing_x/2,lid_boss_spacing_x/2])
            for(y=[-lid_boss_spacing_y/2,lid_boss_spacing_y/2]) translate([x,y,lid_nut_bottom_z]) {
                hex_nut_cut();
                translate([x>0 ? -5 : 0,-3.45,0]) cube([5,6.9,nut_pocket_height]);
            }
    }
}
module rear_caster_mount() {
    difference() {
        union() {
            translate([0,(caster_back_y+caster_front_y)/2,caster_height])
                linear_extrude(height=caster_plate_thickness)
                    rounded_rectangle_2d(caster_mount_width,caster_front_y-caster_back_y,2);
            translate([-caster_mount_width/2,caster_back_y,caster_height])
                cube([caster_mount_width,3,caster_flange_height]);
            for(x=[-caster_mount_hole_spacing/2,caster_mount_hole_spacing/2])
                translate([x-4.5,caster_back_y,caster_wall_bolt_z-4]) cube([9,5,9]);
        }
        for(x=[-caster_mount_hole_spacing/2,caster_mount_hole_spacing/2])
            translate([x,caster_center_y,caster_height-0.1])
                cylinder(d=3.4,h=caster_plate_thickness+0.2,$fn=28);
        caster_wall_bolt_holes();
        for(x=[-caster_mount_hole_spacing/2,caster_mount_hole_spacing/2])
            translate([x,caster_back_y+5-nut_pocket_height,caster_wall_bolt_z])
            rotate([-90,0,0]) hex_nut_cut(height=nut_pocket_height+0.1,angle=0);
    }
}
module lower_post_clearance_pockets() {
    for(s=[-1,1]) {
        translate([s*guide_column_center_x-guide_column_size[0]/2-gate_guide_clearance,
                   guide_axis_y-guide_column_size[1]/2-gate_guide_clearance,-0.1])
            cube([guide_column_size[0]+2*gate_guide_clearance,
                  guide_column_size[1]+2*gate_guide_clearance,guide_cap_top_z+3.1]);
        translate([s*guide_column_center_x-servo_support_post_size[0]/2-gate_guide_clearance,
                   servo_mount_arm_y-servo_support_post_size[1]/2-gate_guide_clearance,-0.1])
            cube([servo_support_post_size[0]+2*gate_guide_clearance,
                  servo_support_post_size[1]+2*gate_guide_clearance,
                  servo_mount_base_z+servo_mount_wall/2+3.1]);
    }
}
module fit_coupon() {
    difference() {
        cube([65,40,6]);
        for(i=[0:2]) {
            translate([10+20*i,8,-0.1]) cylinder(d=3.2+0.2*i,h=6.2,$fn=36);
            translate([10+20*i,20,3.4]) hex_nut_cut(height=2.7,af=5.6+0.2*i);
            c=0.3+0.1*i;
            translate([10+20*i-(4+2*c)/2,32-(6+2*c)/2,-0.1]) cube([4+2*c,6+2*c,6.2]);
        }
        translate([60,8,-0.1]) cylinder(d=small_pilot,h=6.2,$fn=28);
    }
}
module guide_test_key() {
    union() {
        cube([12,12,2]);
        translate([4,3,1.9]) cube([4,6,12.1]);
    }
}

if (part_to_render == "assembly") {
    final_robot_assembly();
} else if (part_to_render == "chassis") {
    // Lid rim on the print bed. Local supports are still needed for ledges.
    translate([0, 0, upper_shell_top_z])
        rotate([180, 0, 0]) main_upper_chassis();
} else if (part_to_render == "lid") {
    top_lid();
} else if (part_to_render == "left_motor_mount") {
    translate([motor_center_x, -motor_center_y, -motor_mount_base_z])
        motor_mount_left();
} else if (part_to_render == "right_motor_mount") {
    translate([-motor_center_x, -motor_center_y, -motor_mount_base_z])
        motor_mount_right();
} else if (part_to_render == "electronics_deck") {
    translate([0, -deck_center_y, -deck_height])
        electronics_deck();
} else if (part_to_render == "caster_mount") {
    translate([0, -caster_center_y, -caster_height])
        rear_caster_mount();
} else if (part_to_render == "battery_mount") {
    translate([0, -battery_center_y, -battery_mount_base_z])
        battery_mount();
} else if (part_to_render == "upper_assembly") {
    upper_assembly();
} else if (part_to_render == "fixed_rear_barriers") {
    fixed_rear_barriers();
} else if (part_to_render == "rear_left_barrier") {
    rear_left_barrier();
} else if (part_to_render == "rear_right_barrier") {
    rear_right_barrier();
} else if (part_to_render == "front_gate") {
    gate_slider();
} else if (part_to_render == "front_gate_carrier") {
    gate_slider(); // compatibility alias: v0.7 prints the joined slider only
} else if (part_to_render == "gate_slider") {
    gate_slider();
} else if (part_to_render == "guide_cap") {
    translate([0,0,guide_cap_thickness]) rotate([180,0,0]) guide_cap();
} else if (part_to_render == "servo_clamp") {
    translate([0,servo_body_center_y,servo_clamp_thickness])
        rotate([180,0,0]) servo_clamp();
} else if (part_to_render == "connecting_link") {
    translate([0,-linkage_length/2,0]) connecting_link();
} else if (part_to_render == "fit_coupon") {
    fit_coupon();
} else if (part_to_render == "guide_test_key") {
    guide_test_key();
} else if (part_to_render == "servo_mount") {
    translate([0, 0, servo_mount_wall / 2]) servo_mount();
} else if (part_to_render == "curved_funnels") {
    translate([0, 0, -funnel_bottom_clearance]) {
        curved_funnel_left();
        curved_funnel_right();
    }
} else if (part_to_render == "curved_funnel_left") {
    translate([0, 0, -funnel_bottom_clearance])
        curved_funnel_left();
} else if (part_to_render == "curved_funnel_right") {
    translate([0, 0, -funnel_bottom_clearance])
        curved_funnel_right();
} else if (part_to_render == "capture_system") {
    capture_system();
} else if (part_to_render == "lower_chassis_with_upper_mounts") {
    translate([0, 0, -side_rail_bottom_z]) lower_chassis_with_upper_mounts();
} else if (part_to_render == "lower_chassis_with_fixed_barriers") {
    lower_chassis_with_fixed_barriers();
} else if (part_to_render == "none") {
    // Library/geometry validation mode; emit no top-level geometry.
} else {
    assert(false, str("Unknown part_to_render: ", part_to_render));
}
