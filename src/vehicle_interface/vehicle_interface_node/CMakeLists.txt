cmake_minimum_required(VERSION 3.0.2)
project(vehicle_interface_node)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  deeplumin_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_interface_node_node src/vehicle_interface_node_node.cpp)
add_dependencies(vehicle_interface_node_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_interface_node_node ${catkin_LIBRARIES})

install(TARGETS vehicle_interface_node_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
