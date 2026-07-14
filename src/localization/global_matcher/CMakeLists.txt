cmake_minimum_required(VERSION 3.0.2)
project(global_matcher)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
  deeplumin_msgs
)
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs pcl_ros deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(global_matcher_node src/global_matcher_node.cpp)
add_dependencies(global_matcher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_matcher_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS global_matcher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
