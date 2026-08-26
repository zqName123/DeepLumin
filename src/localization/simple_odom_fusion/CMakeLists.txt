cmake_minimum_required(VERSION 3.0.2)
project(simple_odom_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  deeplumin_msgs
)

find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED COMPONENTS common filters)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs tf2_ros pcl_ros pcl_conversions deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(simple_odom_fusion_core
  src/core/simple_fusion_core.cpp
)
target_compile_features(simple_odom_fusion_core PUBLIC cxx_std_17)
target_link_libraries(simple_odom_fusion_core ${catkin_LIBRARIES})

add_executable(simple_odom_fusion_node src/ros/simple_odom_fusion_node.cpp)
add_dependencies(simple_odom_fusion_node ${catkin_EXPORTED_TARGETS})
target_compile_features(simple_odom_fusion_node PUBLIC cxx_std_17)
target_link_libraries(simple_odom_fusion_node simple_odom_fusion_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ysw_style_fusion_node src/ros/ysw_style_fusion_node.cpp)
add_dependencies(ysw_style_fusion_node ${catkin_EXPORTED_TARGETS})
target_compile_features(ysw_style_fusion_node PUBLIC cxx_std_17)
target_link_libraries(ysw_style_fusion_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS simple_odom_fusion_core simple_odom_fusion_node ysw_style_fusion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
