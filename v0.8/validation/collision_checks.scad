include <../hamster_print_ready_v0.8.scad>

check = "gate_endpoint";

module fixed_prints() {
    lower_chassis_with_upper_mounts();
    installed_guide_caps();
    translate([0,0,servo_mount_base_z]) servo_mount();
    installed_servo_clamp();
    main_upper_chassis();
    motor_mount_left();
    motor_mount_right();
    electronics_deck();
    battery_mount();
    rear_caster_mount();
}

if (check == "gate_endpoint")
    intersection() { moving_front_gate_system(); fixed_prints(); }

if (check == "offset_disk_to_skids")
    intersection() {
        front_stability_skids();
        for (x=[-funnel_capture_offset_target,
                 funnel_capture_offset_target])
            translate([x,front_skid_center_y,0])
                cylinder(d=sample_diameter,h=sample_thickness,$fn=72);
    }

if (check == "wheels_to_skids")
    intersection() {
        front_stability_skids();
        union() { wheel_preview_left(); wheel_preview_right(); }
    }
