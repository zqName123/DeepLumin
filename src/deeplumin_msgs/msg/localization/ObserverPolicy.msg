std_msgs/Header header

# Scene labels are strings so new deployment scenes can be added without
# changing the message ABI.
string scene_type
string reason

bool use_dr
bool use_slam
bool use_global_matcher
bool use_relocalization
bool use_gnss
bool use_gnss_position
bool use_gnss_heading

float64 dr_weight
float64 slam_weight
float64 global_matcher_weight
float64 relocalization_weight
float64 gnss_position_weight
float64 gnss_heading_weight

# Manager-side safety controls.
bool allow_fusion_reset
bool allow_dr_reset
bool allow_global_matcher_trigger
bool allow_relocalization_trigger
