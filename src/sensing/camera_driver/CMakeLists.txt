cmake_minimum_required(VERSION 3.0.2)
project(camera_driver)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV ${DEEPLUMIN_OPENCV_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_driver_node src/camera_driver_node.cpp)
add_dependencies(camera_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_driver_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS camera_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
