cmake_minimum_required(VERSION 3.0.2)
project(vehicle_model)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs gazebo_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_model_node src/vehicle_model_node.cpp)
add_dependencies(vehicle_model_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_model_node ${catkin_LIBRARIES})

install(TARGETS vehicle_model_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
