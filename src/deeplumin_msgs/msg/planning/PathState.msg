Header header

uint8 state
uint8 NORMAL = 0
uint8 OBSTACLE_DETECTED = 1
uint8 PATH_BLOCKED = 2
uint8 REPLANNING = 3
uint8 GOAL_REACHED = 4

nav_msgs/Path path
float32 progress
float32 estimated_arrival_time
