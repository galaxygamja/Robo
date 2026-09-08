include <../hamster_print_ready_v0.9.scad>

sample_count = ceil(2*servo_half_swing/0.5);
for (i=[0:sample_count]) {
    a = -servo_half_swing + 2*servo_half_swing*i/sample_count;
    z = gate_z_at_angle(a);
    h = link_horn_point(a);
    d = sqrt(pow(h[0]-carrier_link_y,2)
             +pow(h[1]-z-carrier_link_local_z,2));
    assert(abs(d-linkage_length)<0.00001,
           "Rod length changed during motion");
    assert(z>=front_gate_down_bottom_z-0.00001
           && z<=front_gate_up_bottom_z+0.00001,
           "Slider exits intended travel");
    if (i>0)
        assert(z>=gate_z_at_angle(a-2*servo_half_swing/sample_count),
               "Non-monotonic gate motion");
}

max_dy = servo_crank_radius*(1-cos(servo_half_swing));
derivative_lower_bound = servo_crank_radius*cos(servo_half_swing)
    -max_dy*servo_crank_radius*sin(servo_half_swing)
      /sqrt(linkage_length*linkage_length-max_dy*max_dy);
assert(derivative_lower_bound>0,
       "Cannot bound continuous travel monotonically");
assert(front_gate_down_bottom_z+carrier_bottom_z
       > guide_rail_bottom_z+guide_bridge_height/2,
       "Sleeve reaches bottom bridge");
assert(front_gate_up_bottom_z+carrier_bottom_z+guide_sleeve_height
       < guide_column_height-guide_cap_socket_depth,
       "Sleeve reaches top cap");

echo(str("KINEMATICS V0.9 PASS: samples=",sample_count+1,
         ", travel=",gate_z_at_angle(servo_half_swing)
                      -gate_z_at_angle(-servo_half_swing),
         ", continuous monotonic bound=",derivative_lower_bound));
