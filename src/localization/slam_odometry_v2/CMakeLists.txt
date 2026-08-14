cmake_minimum_required(VERSION 3.0.2)
project(slam_odometry_v2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 与 faster_lio_ws 一致：-std=c++17（非 gnu++17）


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  pcl_ros
  pcl_conversions
  eigen_conversions
  message_generation
  deeplumin_msgs
)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)

# faster-lio 依赖的 Livox 消息定义（与 faster_lio_ws thirdparty 一致）
add_subdirectory(thirdparty/livox_ros_driver)

add_message_files(FILES Pose6D.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES slam_odometry_v2_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf pcl_ros pcl_conversions eigen_conversions deeplumin_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

# =============================================================================
# faster-lio 纯里程计核心（源码全部来自 faster_lio_ws，逐行一致）
# 优化编译配置与 faster_lio_ws 一致：-O3 -pthread -fexceptions，
# 不使用 fast-math / 不改浮点融合，保证数值行为与原始版本相同。
# =============================================================================
add_library(slam_odometry_v2_core
  src/faster_lio/laser_odometry.cc
  src/faster_lio/pointcloud_preprocess.cc
  src/faster_lio/options.cc
  src/faster_lio/utils.cc
  src/ros/slam_odometry_v2_config.cpp
)
add_dependencies(slam_odometry_v2_core
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)
target_include_directories(slam_odometry_v2_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/faster_lio)
target_compile_definitions(slam_odometry_v2_core PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
target_compile_options(slam_odometry_v2_core PRIVATE
  -O3
  -pthread
  -fexceptions
  -UNDEBUG
  -fno-fast-math
  -ffp-contract=off
)
target_link_libraries(slam_odometry_v2_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  glog
  yaml-cpp
  TBB::tbb
)

add_executable(slam_odometry_v2_node src/slam_odometry_v2_node.cc)
add_dependencies(slam_odometry_v2_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)
target_compile_options(slam_odometry_v2_node PRIVATE -O3)
target_compile_definitions(slam_odometry_v2_node PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
target_link_libraries(slam_odometry_v2_node
  slam_odometry_v2_core
  TBB::tbb
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS slam_odometry_v2_core slam_odometry_v2_node
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
