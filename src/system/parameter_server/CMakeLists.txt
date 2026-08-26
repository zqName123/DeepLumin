cmake_minimum_required(VERSION 3.0.2)
project(parameter_server)

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

add_executable(parameter_server_node src/parameter_server_node.cpp)
add_dependencies(parameter_server_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(parameter_server_node ${catkin_LIBRARIES})

install(TARGETS parameter_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
