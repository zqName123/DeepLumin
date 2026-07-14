cmake_minimum_required(VERSION 3.0.2)
project(deeplumin_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg/perception
  FILES
  Obstacle.msg
  ObstacleArray.msg
  FilteredPointCloud.msg
)

add_message_files(
  DIRECTORY msg/localization
  FILES
  SlamPose.msg
  FusedOdometry.msg
  LocalizationStatus.msg
  SlamHealth.msg
  Gpchc.msg
  CanReceiveInfo.msg
)

add_message_files(
  DIRECTORY msg/planning
  FILES
  PathState.msg
  SpeedProfile.msg
)

add_message_files(
  DIRECTORY msg/control
  FILES
  ControlCommand.msg
  ControlStatus.msg
)

add_message_files(
  DIRECTORY msg/system
  FILES
  SystemStatus.msg
  VehicleStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs sensor_msgs message_runtime
)
