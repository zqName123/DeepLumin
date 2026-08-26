cmake_minimum_required(VERSION 3.0.2)
project(localization_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  deeplumin_msgs
)

find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED COMPONENTS common io)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs pcl_ros pcl_conversions deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(localization_manager_node
  src/localization_manager_node.cpp
  src/core/manager_core.cpp
  src/core/map_height_provider.cpp
  src/ros/ros_adapter.cpp
)
add_dependencies(localization_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_manager_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS localization_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
