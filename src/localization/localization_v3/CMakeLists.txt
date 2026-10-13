cmake_minimum_required(VERSION 3.0.2)
project(localization_v3)

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
  tf2_ros
  pcl_ros
  pcl_conversions
  deeplumin_msgs
  deeplumin_health_checker
  syslog_ros_wrap
)

find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED COMPONENTS common io filters)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros pcl_ros pcl_conversions deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# 核心库：纯 C++ 逻辑（不依赖 ROS 消息），便于单元测试
#   fusion_core      位姿写入策略（人工/验证/jump guard/恢复）
#   jump_guard       TRACKING 跳变防护
#   failure_detector 快速失效信号
#   state_machine    五态状态机
# ---------------------------------------------------------------------------
add_library(localization_v3_core
  src/core/fusion_core.cpp
  src/core/jump_guard.cpp
  src/core/failure_detector.cpp
  src/core/state_machine.cpp
  src/core/local_odom_source.cpp
)
target_compile_features(localization_v3_core PUBLIC cxx_std_17)
target_link_libraries(localization_v3_core ${EIGEN3_LIBRARIES})

# ---------------------------------------------------------------------------
# ROS 适配层：参数加载、状态组装、重定位客户端
# ---------------------------------------------------------------------------
add_library(localization_v3_ros
  src/ros/localization_v3_config.cpp
  src/ros/status_publisher.cpp
  src/ros/relocalization_ros_adapter.cpp
)
add_dependencies(localization_v3_ros ${catkin_EXPORTED_TARGETS})
target_compile_features(localization_v3_ros PUBLIC cxx_std_17)
target_link_libraries(localization_v3_ros
  localization_v3_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ---------------------------------------------------------------------------
# 节点：薄编排层
# ---------------------------------------------------------------------------
add_executable(localization_v3_node src/ros/localization_v3_node.cpp)
add_dependencies(localization_v3_node ${catkin_EXPORTED_TARGETS})
target_compile_features(localization_v3_node PUBLIC cxx_std_17)
target_link_libraries(localization_v3_node
  localization_v3_core
  localization_v3_ros
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ---------------------------------------------------------------------------
# 局部里程计源桥（SLAM 主 / DR 备）——最小侵入接入，独立节点便于回退
# ---------------------------------------------------------------------------
add_executable(source_bridge_node src/ros/source_bridge_node.cpp)
add_dependencies(source_bridge_node ${catkin_EXPORTED_TARGETS})
target_compile_features(source_bridge_node PUBLIC cxx_std_17)
target_link_libraries(source_bridge_node
  localization_v3_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS localization_v3_core localization_v3_ros localization_v3_node source_bridge_node
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
# 单元测试（core 零 ROS 依赖）
# 目标名带 v3 前缀：catkin_make 全工作空间单构建树，目标名全局唯一
# （localization_v2 已占用 test_state_machine 等名字）
# ---------------------------------------------------------------------------
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)

  include_directories(${GTEST_INCLUDE_DIRS})

  catkin_add_gtest(test_v3_state_machine test/test_state_machine.cpp)
  target_link_libraries(test_v3_state_machine localization_v3_core ${GTEST_LIBRARIES} pthread)

  catkin_add_gtest(test_v3_fusion_core test/test_fusion_core.cpp)
  target_link_libraries(test_v3_fusion_core localization_v3_core ${GTEST_LIBRARIES} pthread)

  catkin_add_gtest(test_v3_jump_guard test/test_jump_guard.cpp)
  target_link_libraries(test_v3_jump_guard localization_v3_core ${GTEST_LIBRARIES} pthread)

  catkin_add_gtest(test_v3_failure_detector test/test_failure_detector.cpp)
  target_link_libraries(test_v3_failure_detector localization_v3_core ${GTEST_LIBRARIES} pthread)

  catkin_add_gtest(test_v3_local_odom_source test/test_local_odom_source.cpp)
  target_link_libraries(test_v3_local_odom_source localization_v3_core ${GTEST_LIBRARIES} pthread)
endif()
