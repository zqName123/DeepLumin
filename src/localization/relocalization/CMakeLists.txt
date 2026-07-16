cmake_minimum_required(VERSION 3.0.2)
project(relocalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  deeplumin_msgs
)
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED COMPONENTS common io filters registration kdtree)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES relocalization_core
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros rosbag roscpp sensor_msgs std_msgs std_srvs visualization_msgs deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_library(relocalization_core
  src/core/common.cpp
  src/core/keyframe_database.cpp
  src/core/relocalization_core.cpp
  src/descriptor/scan_context_descriptor.cpp
  src/matcher/gicp_global_matcher.cpp
  src/ros/eval_visualizer.cpp
  src/ros/param_loader.cpp
)
add_dependencies(relocalization_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(relocalization_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_include_directories(relocalization_core PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_options(relocalization_core PRIVATE
  -O3
  -DNDEBUG
)


add_executable(global_relocalization_node src/ros/global_relocalization_node.cpp)
add_dependencies(global_relocalization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_relocalization_node relocalization_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

target_compile_options(global_relocalization_node PRIVATE
  -O3
  -DNDEBUG
)


add_executable(bag_eval_node src/ros/bag_eval_node.cpp)
add_dependencies(bag_eval_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(bag_eval_node relocalization_core ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${rosbag_LIBRARIES})

add_executable(relocalization_node src/relocalization_node.cpp)
add_dependencies(relocalization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(relocalization_node ${catkin_LIBRARIES})

install(TARGETS relocalization_core global_relocalization_node bag_eval_node relocalization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
