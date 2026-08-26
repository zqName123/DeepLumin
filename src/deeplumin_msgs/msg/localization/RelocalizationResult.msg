std_msgs/Header header

# Frame semantics:
# - header.frame_id: global/map frame of pose
# - child_frame_id: recovered body frame, usually base_link
string child_frame_id

geometry_msgs/PoseWithCovariance pose

bool success

# Set by localization_manager when this result has passed validation and can
# be consumed by pose_fusion as a reset event.
bool accepted

string map_id
string method
string reject_reason

# Descriptor and fine-registration quality.
float64 score
float64 descriptor_score
float64 fitness_score
float64 inlier_ratio
float64 inlier_rmse
uint32 inlier_count

# Candidate/debug metadata.
int32 candidate_index
int32 keyframe_id
float64 yaw_diff
float64 translation_diff
float64 elapsed_ms
