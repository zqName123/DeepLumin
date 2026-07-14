# =============================================================================
# dr_odometry / CMakeLists.txt
#
# 产出：
#   - 库 dr_odometry_core：ESKF 核心 + ROS 适配（供节点 link）
#   - 可执行文件 dr_odometry_node：正式节点
#   - 可执行文件 dr_odometry_debug_node：带观测消融开关的调试节点
#
# 依赖：roscpp、常用 msgs、tf、deeplumin_msgs、Eigen3（版本由 deeplumin_cmake 约束）
# =============================================================================
cmake_minimum_required(VERSION 3.0.2)
project(dr_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  deeplumin_msgs
)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dr_odometry_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ---------- 核心库：算法 + 适配，不含 main ----------
add_library(dr_odometry_core
  src/core/dr_eskf.cpp
  src/ros/ros_adapter.cpp
)
add_dependencies(dr_odometry_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(dr_odometry_core ${catkin_LIBRARIES})
target_include_directories(dr_odometry_core PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# ---------- 正式 / 调试节点 ----------
add_executable(dr_odometry_node src/dr_odometry_node.cpp)
add_dependencies(dr_odometry_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dr_odometry_node dr_odometry_core ${catkin_LIBRARIES})

add_executable(dr_odometry_debug_node src/dr_odometry_debug_node.cpp)
add_dependencies(dr_odometry_debug_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dr_odometry_debug_node dr_odometry_core ${catkin_LIBRARIES})

install(TARGETS dr_odometry_core dr_odometry_node dr_odometry_debug_node
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
