include <../hamster_print_ready_v0.9.scad>

check = "gate_endpoint";

module fixed_prints_v09() {
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

module installed_hardware_references_v09() {
    motor_preview_left();
    motor_preview_right();
    battery_preview();
}

if (check == "gate_endpoint")
    intersection() { moving_front_gate_system(); fixed_prints_v09(); }

if (check == "offset_disk_to_skids")
    intersection() {
        front_stability_skids();
        for (x=[-funnel_capture_offset_target,
                 funnel_capture_offset_target])
            translate([x,front_skid_center_y,0])
                cylinder(d=sample_diameter,h=sample_thickness,$fn=72);
    }

if (check == "wheels_to_prints")
    intersection() {
        union() { wheel_preview_left(); wheel_preview_right(); }
        union() {
            lower_chassis_with_upper_mounts();
            motor_mount_left();
            motor_mount_right();
            main_upper_chassis();
        }
    }

if (check == "hardware_to_prints")
    intersection() {
        installed_hardware_references_v09();
        union() {
            lower_chassis_with_upper_mounts();
            motor_mount_left();
            motor_mount_right();
            battery_mount();
            electronics_deck();
            main_upper_chassis();
            moving_front_gate_system();
            rear_caster_mount();
        }
    }

if (check == "motors_to_prints")
    intersection() {
        union() { motor_preview_left(); motor_preview_right(); }
        union() {
            lower_chassis_with_upper_mounts();
            motor_mount_left();
            motor_mount_right();
            electronics_deck();
            main_upper_chassis();
            moving_front_gate_system();
            rear_caster_mount();
        }
    }

if (check == "battery_to_prints")
    intersection() {
        battery_preview();
        union() {
            lower_chassis_with_upper_mounts();
            motor_mount_left();
            motor_mount_right();
            battery_mount();
            electronics_deck();
            main_upper_chassis();
            moving_front_gate_system();
            rear_caster_mount();
        }
    }

if (check == "battery_to_lower")
    intersection() { battery_preview(); lower_chassis_with_upper_mounts(); }
if (check == "battery_to_motor_mounts")
    intersection() { battery_preview(); union() { motor_mount_left(); motor_mount_right(); } }
if (check == "battery_to_tray")
    intersection() { battery_preview(); battery_mount(); }
if (check == "battery_to_deck")
    intersection() { battery_preview(); electronics_deck(); }
if (check == "battery_to_shell")
    intersection() { battery_preview(); main_upper_chassis(); }
if (check == "battery_to_gate")
    intersection() { battery_preview(); moving_front_gate_system(); }
if (check == "battery_to_caster")
    intersection() { battery_preview(); rear_caster_mount(); }

if (check == "battery_to_motors")
    intersection() {
        battery_preview();
        union() { motor_preview_left(); motor_preview_right(); }
    }
