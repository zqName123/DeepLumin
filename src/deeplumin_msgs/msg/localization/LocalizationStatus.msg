std_msgs/Header header
uint8 level
uint8 LEVEL_NORMAL = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_DEGRADED = 2
uint8 LEVEL_FAILURE = 3
string failure_reason
float32 quality_score
bool is_gnss_available
bool is_slam_available
bool is_dr_available
string current_mode
