cmake_minimum_required(VERSION 3.0.2)
project(lidar_driver)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(lidar_driver_node src/lidar_driver_node.cpp)
add_dependencies(lidar_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_driver_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS lidar_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
