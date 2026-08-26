std_msgs/Header header

# Frame semantics:
# - header.frame_id: global/map frame of pose
# - child_frame_id: matched body frame, usually base_link
string child_frame_id

geometry_msgs/PoseWithCovariance pose

bool success
bool converged
string map_id
string reject_reason

# Registration quality. fitness_score follows the matcher implementation
# convention, where smaller is usually better.
float64 fitness_score
float64 inlier_ratio
float64 inlier_rmse
uint32 inlier_count
uint32 source_points
uint32 target_points
float64 elapsed_ms

# Initial guess consistency, used by pose_fusion/localization_manager gates.
float64 initial_translation_error
float64 initial_yaw_error
