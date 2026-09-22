cmake_minimum_required(VERSION 3.0.2)
project(deeplumin_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
  rosgraph_msgs
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
  LocalizationStatus.msg
  GlobalMatchResult.msg
  RelocalizationResult.msg
  FusionStatus.msg
  MappingEnvironmentItem.msg
  MappingEnvironmentStatus.msg
  MappingStatus.msg
  MappingRecordStatus.msg
  LocPoint.msg
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
  Gpchc.msg
  CanReceiveInfo.msg
  DiagnosticStatus.msg
  DiagnosticStatusArray.msg
  HardwareStatus.msg
  NodeStatus.msg
)

add_service_files(
  DIRECTORY srv/localization
  FILES
  CheckMappingEnvironment.srv
  StartMapping.srv
  StopMapping.srv
  StartBagRecording.srv
  StopBagRecording.srv
  GetMappingStatus.srv
  StartMappingRuntime.srv
  StopMappingRuntime.srv
  SaveMappingRuntime.srv
  GetMappingRuntimeStatus.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  rosgraph_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs sensor_msgs rosgraph_msgs message_runtime
)
