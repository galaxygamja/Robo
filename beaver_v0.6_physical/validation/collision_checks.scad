include <../beaver_robot_v0.6_physical.scad>

check = "housing_pose";
gate_test_angle = 0;
front_test_state = "CLOSE";

if (check == "housing_pose")
    intersection() {
        upper_housing_raw();
        housing_clearance_targets(front_test_state,gate_test_angle);
    }

if (check == "wheel_to_chassis")
    intersection() {
        wheel_reference_v04(1);
        integrated_chassis_raw();
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
