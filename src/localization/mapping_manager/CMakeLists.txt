cmake_minimum_required(VERSION 3.0.2)
project(mapping_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  deeplumin_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mapping_manager_node
  src/mapping_manager_node.cpp
  src/core/manager_core.cpp
  src/ros/ros_adapter.cpp
)
add_dependencies(mapping_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapping_manager_node ${catkin_LIBRARIES})

install(TARGETS mapping_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
