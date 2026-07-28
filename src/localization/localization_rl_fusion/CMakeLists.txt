cmake_minimum_required(VERSION 3.0.2)
project(localization_rl_fusion)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED COMPONENTS common io filters)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_map_publisher src/pcd_map_publisher.cpp)
add_dependencies(pcd_map_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_map_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS pcd_map_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
