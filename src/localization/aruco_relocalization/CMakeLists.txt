cmake_minimum_required(VERSION 3.0.2)
project(aruco_relocalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  deeplumin_msgs
)

find_package(OpenCV REQUIRED COMPONENTS core aruco calib3d imgproc)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_reloc_core
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs std_srvs tf2 tf2_ros deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Core library
add_library(aruco_reloc_core
  src/detector/marker_detector.cpp
  src/map/marker_map_loader.cpp
  src/solver/relocalization_solver.cpp
  src/validator/relocalization_validator.cpp
  src/ros/param_loader.cpp
)
add_dependencies(aruco_reloc_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_reloc_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
target_include_directories(aruco_reloc_core PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_options(aruco_reloc_core PRIVATE -O3 -DNDEBUG)

# Online relocalization node
add_executable(aruco_reloc_online_node src/ros/online_node.cpp)
add_dependencies(aruco_reloc_online_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_reloc_online_node aruco_reloc_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_compile_options(aruco_reloc_online_node PRIVATE -O3 -DNDEBUG)

# Offline mapping node
add_executable(aruco_reloc_offline_mapping_node src/ros/offline_mapping_node.cpp src/mapping/offline_mapper.cpp)
add_dependencies(aruco_reloc_offline_mapping_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_reloc_offline_mapping_node aruco_reloc_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_compile_options(aruco_reloc_offline_mapping_node PRIVATE -O3 -DNDEBUG)

install(TARGETS aruco_reloc_core aruco_reloc_online_node aruco_reloc_offline_mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
