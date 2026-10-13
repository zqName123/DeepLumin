cmake_minimum_required(VERSION 3.10)
project(local_odom_gateway)

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
  pcl_ros
  pcl_conversions
  localization_common
  syslog_ros_wrap
)
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED COMPONENTS common io filters)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_odom_gateway_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs pcl_ros pcl_conversions localization_common
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ── core：纯 C++，零 ROS 依赖（仲裁/连续性/锚+增量） ──────────────
add_library(local_odom_gateway_core
  src/core/motion_provider.cpp
  src/core/arbiter.cpp
)
target_compile_features(local_odom_gateway_core PUBLIC cxx_std_17)
target_link_libraries(local_odom_gateway_core ${EIGEN3_LIBRARIES})

# ── ros：薄适配层（订阅/发布/时间基），内嵌与独立前端共用 ─────────
# S4 阶段接入（adapters / embedded / standalone）；当前先落 core + 测试
# add_library(local_odom_gateway_ros src/ros/adapters.cpp)
# target_link_libraries(local_odom_gateway_ros local_odom_gateway_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_executable(local_odom_gateway_node src/ros/standalone_node.cpp)
# target_link_libraries(local_odom_gateway_node local_odom_gateway_ros)

install(TARGETS local_odom_gateway_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# ── 单元测试（core 零 ROS） ─────────────────────────────────────
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)

  catkin_add_gtest(test_gw_motion_provider test/test_motion_provider.cpp)
  target_link_libraries(test_gw_motion_provider local_odom_gateway_core ${GTEST_LIBRARIES} pthread)

  catkin_add_gtest(test_gw_arbiter test/test_arbiter.cpp)
  target_link_libraries(test_gw_arbiter local_odom_gateway_core ${GTEST_LIBRARIES} pthread)

  catkin_add_gtest(test_gw_contracts test/test_contracts.cpp)
  target_link_libraries(test_gw_contracts local_odom_gateway_core ${GTEST_LIBRARIES} pthread)
endif()
