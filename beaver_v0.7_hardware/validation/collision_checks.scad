include <../beaver_robot_v0.7_hardware.scad>

check = "housing_pose";
gate_test_angle = 0;
front_test_state = "CLOSE";

if (check == "housing_pose")
    intersection() {
        upper_housing_raw();
        housing_clearance_targets(front_test_state,gate_test_angle);
    }

if (check == "wheel_to_prints")
    intersection() {
        wheel_reference_v04(1);
        union() {
            integrated_chassis_raw();
            battery_tray_assembly(false);
            drive_motor_mount_right(false);
            drive_motor_mount_left(false);
        }
    }

if (check == "patient_to_jaws")
    intersection() {
        patient_reference(1);
        union() {
            jaw_right_assembly(jaw_close_angle);
            jaw_left_assembly(jaw_close_angle);
            jaw_right_assembly(jaw_open_angle);
            jaw_left_assembly(jaw_open_angle);
        }
    }

if (check == "patient_to_skids")
    intersection() {
        patient_reference(1);
        stability_skids();
    }

if (check == "boxes_to_magazine")
    intersection() {
        loaded_boxes(2,1);
        cargo_magazine_assembly(false);
    }

if (check == "motor_to_prints")
    intersection() {
        motor_reference_v07(1);
        union() {
            integrated_chassis_raw();
            battery_tray_assembly(false);
            drive_motor_mount_right(false);
            drive_motor_mount_left(false);
            cargo_magazine_assembly(false);
            upper_housing_raw();
        }
    }

if (check == "motor_to_chassis")
    intersection() { motor_reference_v07(1); integrated_chassis_raw(); }
if (check == "motor_to_mounts")
    intersection() {
        motor_reference_v07(1);
        union() { drive_motor_mount_right(false); drive_motor_mount_left(false); }
    }
if (check == "motor_to_battery_tray")
    intersection() { motor_reference_v07(1); battery_tray_assembly(false); }
if (check == "motor_to_cargo")
    intersection() { motor_reference_v07(1); cargo_magazine_assembly(false); }
if (check == "motor_to_housing")
    intersection() { motor_reference_v07(1); upper_housing_raw(); }

if (check == "battery_to_prints")
    intersection() {
        battery_reference_v07(1);
        union() {
            integrated_chassis_raw();
            battery_tray_assembly(false);
            drive_motor_mount_right(false);
            drive_motor_mount_left(false);
            cargo_magazine_assembly(false);
            rear_dump_gate_assembly(0);
            upper_housing_raw();
        }
    }

if (check == "battery_to_chassis")
    intersection() { battery_reference_v07(1); integrated_chassis_raw(); }
if (check == "battery_to_tray")
    intersection() { battery_reference_v07(1); battery_tray_assembly(false); }
if (check == "battery_to_motor_mounts")
    intersection() {
        battery_reference_v07(1);
        union() { drive_motor_mount_right(false); drive_motor_mount_left(false); }
    }
if (check == "battery_to_cargo")
    intersection() { battery_reference_v07(1); cargo_magazine_assembly(false); }
if (check == "battery_to_gate")
    intersection() { battery_reference_v07(1); rear_dump_gate_assembly(0); }
if (check == "battery_to_housing")
    intersection() { battery_reference_v07(1); upper_housing_raw(); }

if (check == "battery_to_motors")
    intersection() {
        battery_reference_v07(1);
        motor_reference_v07(1);
    }
