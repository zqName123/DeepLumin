cmake_minimum_required(VERSION 3.0.2)
project(localization_v2)

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
# 核心库：纯 C++ 逻辑（不依赖 ROS 消息），便于单元测试与后续复用
# ---------------------------------------------------------------------------
add_library(localization_v2_core
  src/core/fusion_core.cpp
  src/core/initialization.cpp
)
target_compile_features(localization_v2_core PUBLIC cxx_std_17)
target_link_libraries(localization_v2_core ${EIGEN3_LIBRARIES})

# ---------------------------------------------------------------------------
# ROS 适配层：话题/参数加载、重定位客户端实现
# ---------------------------------------------------------------------------
add_library(localization_v2_ros
  src/ros/localization_v2_config.cpp
  src/ros/relocalization_ros_adapter.cpp
)
add_dependencies(localization_v2_ros ${catkin_EXPORTED_TARGETS})
target_compile_features(localization_v2_ros PUBLIC cxx_std_17)
target_link_libraries(localization_v2_ros
  localization_v2_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------
add_executable(localization_v2_node src/ros/localization_v2_node.cpp)
add_dependencies(localization_v2_node ${catkin_EXPORTED_TARGETS})
target_compile_features(localization_v2_node PUBLIC cxx_std_17)
target_link_libraries(localization_v2_node
  localization_v2_core
  localization_v2_ros
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS localization_v2_core localization_v2_ros localization_v2_node
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
