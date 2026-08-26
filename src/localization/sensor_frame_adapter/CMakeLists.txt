cmake_minimum_required(VERSION 3.0.2)
project(sensor_frame_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(deeplumin_cmake REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(sensor_frame_adapter_node src/sensor_frame_adapter_node.cpp)
add_dependencies(sensor_frame_adapter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_frame_adapter_node ${catkin_LIBRARIES})

install(TARGETS sensor_frame_adapter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
