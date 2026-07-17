cmake_minimum_required(VERSION 3.0.2)
project(pose_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  deeplumin_msgs
)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_fusion_core pose_fusion_ros
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(pose_fusion_core
  src/core/types.cpp
  src/core/quality_gate.cpp
  src/core/fusion_state_machine.cpp
  src/core/pose_fusion_core.cpp
)
add_dependencies(pose_fusion_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_fusion_core ${catkin_LIBRARIES})

add_library(pose_fusion_ros
  src/ros/param_loader.cpp
  src/ros/ros_adapter.cpp
  src/ros/tf_manager.cpp
)
add_dependencies(pose_fusion_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_fusion_ros pose_fusion_core ${catkin_LIBRARIES})

add_executable(pose_fusion_node src/pose_fusion_node.cpp)
add_dependencies(pose_fusion_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_fusion_node pose_fusion_core pose_fusion_ros ${catkin_LIBRARIES})

add_executable(pose_fusion_path_compare_node src/pose_fusion_path_compare_node.cpp)
add_dependencies(pose_fusion_path_compare_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_fusion_path_compare_node ${catkin_LIBRARIES})

install(TARGETS pose_fusion_core pose_fusion_ros pose_fusion_node pose_fusion_path_compare_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
