geometry_msgs/PoseStamped pre_grasp
geometry_msgs/PoseStamped grasp
geometry_msgs/PoseStamped retreat

# Quality score of the grasp candidate.
float64 score

bool is_full_antipodal # Return whether the grasp is antipodal.
bool is_half_antipodal # Return whether the grasp is indeterminate.
float64 grasp_width #the width of the object contained in the grasp