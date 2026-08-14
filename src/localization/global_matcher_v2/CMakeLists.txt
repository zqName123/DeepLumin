cmake_minimum_required(VERSION 3.0.2)
project(global_matcher_v2)

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
  pcl_ros
  pcl_conversions
  deeplumin_msgs
  small_gicp
  fast_gicp
)
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED COMPONENTS common io filters)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)
find_package(OpenMP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs pcl_ros pcl_conversions deeplumin_msgs small_gicp fast_gicp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# 核心库：地图管理 + 匹配器（纯算法，不依赖 ROS 消息）
# ---------------------------------------------------------------------------
add_library(global_matcher_v2_core
  src/core/map_manager.cpp
  src/core/matcher.cpp
)
target_compile_features(global_matcher_v2_core PUBLIC cxx_std_17)
target_link_libraries(global_matcher_v2_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OpenMP::OpenMP_CXX
)

# ---------------------------------------------------------------------------
# ROS 适配层：配置加载
# ---------------------------------------------------------------------------
add_library(global_matcher_v2_ros
  src/ros/global_matcher_v2_config.cpp
)
add_dependencies(global_matcher_v2_ros ${catkin_EXPORTED_TARGETS})
target_compile_features(global_matcher_v2_ros PUBLIC cxx_std_17)
target_link_libraries(global_matcher_v2_ros
  global_matcher_v2_core
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------
add_executable(global_matcher_v2_node src/ros/global_matcher_v2_node.cpp)
add_dependencies(global_matcher_v2_node ${catkin_EXPORTED_TARGETS})
target_compile_features(global_matcher_v2_node PUBLIC cxx_std_17)
target_link_libraries(global_matcher_v2_node
  global_matcher_v2_core
  global_matcher_v2_ros
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(bench_voxel_build src/tools/bench_voxel_build.cpp)
add_dependencies(bench_voxel_build ${catkin_EXPORTED_TARGETS})
target_compile_features(bench_voxel_build PUBLIC cxx_std_17)
target_link_libraries(bench_voxel_build ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

install(TARGETS global_matcher_v2_core global_matcher_v2_ros global_matcher_v2_node bench_voxel_build
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
