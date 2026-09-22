cmake_minimum_required(VERSION 3.0.2)
project(slam_mapping_v2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
  cv_bridge
  deeplumin_msgs
  deeplumin_health_checker
  syslog_ros_wrap
)
find_package(PCL REQUIRED COMPONENTS common io features filters search)
find_package(OpenCV REQUIRED)
set(GTSAM_DIR "/home/hhy/2004ros/DeepLumin_zq/gtsam_install/lib/cmake/GTSAM" CACHE PATH "Path to GTSAMConfig.cmake" FORCE)
find_package(GTSAM REQUIRED CONFIG PATHS "${GTSAM_DIR}" NO_DEFAULT_PATH)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs tf2_ros pcl_ros pcl_conversions std_srvs cv_bridge deeplumin_msgs
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ivox3d
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DBoW2
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ORBextractor
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
)

# ===== 第三方库 =====

# DBoW2（词袋模型库）
add_subdirectory(thirdparty/DBoW2)

# ORBextractor（ORB 特征提取器）
add_subdirectory(thirdparty/ORBextractor)

# ===== v2 自有核心：订阅驱动多线程建图 =====
add_library(slam_mapping_v2_core
  src/core/mapping_runtime.cpp
  src/core/keyframe_manager.cpp
  src/core/gnss_optimizer.cpp
  src/core/loop_detector_kdtree_gicp.cpp
  src/core/pose_graph_optimizer.cpp
  src/core/global_map.cpp
  src/core/map_saver.cpp
  src/core/types_impl.cpp
  # ===== 视觉回环模块 =====
  src/core/orb_feature_provider.cpp
  src/core/visual_feature_buffer.cpp
  src/core/visual_loop_detector.cpp
  src/core/loop_detector_factory.cpp
  # ===== ROS 适配层 =====
  src/ros/converters.cpp
  src/ros/config_loader.cpp
  src/ros/publishers.cpp
)
add_dependencies(slam_mapping_v2_core ${catkin_EXPORTED_TARGETS})
target_compile_options(slam_mapping_v2_core PRIVATE -O2)
target_link_libraries(slam_mapping_v2_core
  gtsam
  glog
  DBoW2
  ORBextractor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(slam_mapping_v2_node src/slam_mapping_v2_node.cc)
add_dependencies(slam_mapping_v2_node ${catkin_EXPORTED_TARGETS})
target_compile_options(slam_mapping_v2_node PRIVATE -O2)
target_link_libraries(slam_mapping_v2_node
  slam_mapping_v2_core
  gtsam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS slam_mapping_v2_core slam_mapping_v2_node DBoW2 ORBextractor
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
