std_msgs/Header header

bool lanes_first_with_last_connected

pacsim/Landmark[] left_lane
pacsim/Landmark[] right_lane
pacsim/Landmark[] unknown

pacsim/Landmark[] time_keeping_gates

# path is represented by pairs of left and right boundary point
# lists contain index in left_lane and right_lane respectively
int32[] path_left_point_indices
int32[] path_right_points_indices