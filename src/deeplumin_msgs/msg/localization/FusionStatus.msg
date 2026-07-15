std_msgs/Header header

uint8 LEVEL_NORMAL = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_DEGRADED = 2
uint8 LEVEL_FAILURE = 3

uint8 level
string state
bool initialized
bool output_valid
string failure_reason

string current_map_id
string active_scene

float64 last_dr_age
float64 last_slam_age
float64 last_global_matcher_age
float64 last_relocalization_age
float64 last_gnss_age
float64 last_can_age
float64 last_imu_age
float64 last_lidar_age

float64 position_std
float64 velocity_std
float64 roll_std
float64 pitch_std
float64 yaw_std

string[] used_sources
string[] rejected_sources
string[] reject_reasons

bool is_dr_available
bool is_slam_available
bool is_global_matcher_available
bool is_relocalization_available
bool is_gnss_available
bool is_can_available
bool is_imu_available
bool is_lidar_available
