cmake_minimum_required(VERSION 3.0.2)
project(slam_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  std_srvs
  deeplumin_msgs
  slam_odometry
)

find_package(PCL REQUIRED COMPONENTS common io filters)
set(GTSAM_DIR "/home/hhy/2004ros/DeepLumin/gtsam_install/lib/cmake/GTSAM" CACHE PATH "Path to GTSAMConfig.cmake" FORCE)
find_package(GTSAM REQUIRED CONFIG PATHS "${GTSAM_DIR}" NO_DEFAULT_PATH)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs tf2_ros pcl_ros pcl_conversions std_srvs deeplumin_msgs slam_odometry
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
)

add_library(slam_mapping_core
  src/core/mapping_runtime.cpp
  src/core/keyframe_manager.cpp
  src/core/map_saver.cpp
  src/core/loop_closure_backend.cpp
  src/core/loop_closure_fastlio2_backend.cpp
  src/ros/param_loader.cpp
  src/ros/ros_converters.cpp
  src/ros/visualization_publisher.cpp
  src/ros/ros_adapter.cpp
)
add_dependencies(slam_mapping_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_mapping_core ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

add_executable(slam_mapping_node src/slam_mapping_node.cpp)
add_dependencies(slam_mapping_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_mapping_node slam_mapping_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS slam_mapping_core slam_mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
