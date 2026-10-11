cmake_minimum_required(VERSION 3.0.2)
project(dr_odometry_v2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  pcl_conversions
  deeplumin_msgs
  syslog_ros_wrap
)

find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED COMPONENTS common io filters)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# 核心库：纯 C++（零 ROS）：ESKF / 时间序事件队列 / 定位观测跳变判断
# ---------------------------------------------------------------------------
add_library(dr_odometry_v2_core
  src/core/eskf.cpp
  src/core/event_queue.cpp
  src/core/pose_gate.cpp
)
target_compile_features(dr_odometry_v2_core PUBLIC cxx_std_17)
target_link_libraries(dr_odometry_v2_core ${EIGEN3_LIBRARIES})

# ---------------------------------------------------------------------------
# ROS 适配层：配置加载、消息转换
# ---------------------------------------------------------------------------
add_library(dr_odometry_v2_ros
  src/ros/config.cpp
  src/ros/converters.cpp
  src/ros/map_viz.cpp
)
add_dependencies(dr_odometry_v2_ros ${catkin_EXPORTED_TARGETS})
target_compile_features(dr_odometry_v2_ros PUBLIC cxx_std_17)
target_link_libraries(dr_odometry_v2_ros
  dr_odometry_v2_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ---------------------------------------------------------------------------
# 节点：薄编排
# ---------------------------------------------------------------------------
add_executable(dr_odometry_v2_node src/dr_odometry_v2_node.cpp)
add_dependencies(dr_odometry_v2_node ${catkin_EXPORTED_TARGETS})
target_compile_features(dr_odometry_v2_node PUBLIC cxx_std_17)
target_link_libraries(dr_odometry_v2_node
  dr_odometry_v2_core
  dr_odometry_v2_ros
  ${catkin_LIBRARIES}
)

install(TARGETS dr_odometry_v2_core dr_odometry_v2_ros dr_odometry_v2_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ---------------------------------------------------------------------------
# 单元测试（目标名带 v2 前缀：catkin 单构建树内目标名全局唯一）
# ---------------------------------------------------------------------------
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})

  catkin_add_gtest(test_v2_eskf test/test_v2_eskf.cpp)
  target_link_libraries(test_v2_eskf dr_odometry_v2_core ${GTEST_LIBRARIES} pthread)

  catkin_add_gtest(test_v2_event_queue test/test_v2_event_queue.cpp)
  target_link_libraries(test_v2_event_queue dr_odometry_v2_core ${GTEST_LIBRARIES} pthread)

  catkin_add_gtest(test_v2_pose_gate test/test_v2_pose_gate.cpp)
  target_link_libraries(test_v2_pose_gate dr_odometry_v2_core ${GTEST_LIBRARIES} pthread)
endif()
