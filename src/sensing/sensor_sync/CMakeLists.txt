cmake_minimum_required(VERSION 3.0.2)
project(sensor_sync)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_filters
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_sync_node src/sensor_sync_node.cpp)
add_dependencies(sensor_sync_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_sync_node ${catkin_LIBRARIES})

install(TARGETS sensor_sync_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
