cmake_minimum_required(VERSION 3.0.2)
project(slam_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "" CACHE STRING "Build type" FORCE)
endif()


add_compile_options(-ffp-contract=off) # 浮点融合关闭，保证精度

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  tf
  deeplumin_msgs
)
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED COMPONENTS common io filters)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)
find_package(TBB REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES slam_odometry_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs pcl_ros pcl_conversions tf deeplumin_msgs
)

set(FASTER_LIO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/faster_lio)
set(FASTER_LIO_INCLUDE ${FASTER_LIO_ROOT}/include)

include_directories(
  include
  ${FASTER_LIO_INCLUDE}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(slam_odometry_core
  src/slam/faster_lio_impl.cpp
  src/slam/laser_mapping_core.cpp
  src/factory/factory_registry.cpp
  src/ros/pointcloud_preprocess.cpp
  src/ros/ros_converters.cpp
  src/ros/sensor_topics.cpp
  ${FASTER_LIO_ROOT}/src/options.cc
  ${FASTER_LIO_ROOT}/src/utils.cc
)
add_dependencies(slam_odometry_core ${catkin_EXPORTED_TARGETS})
target_compile_options(slam_odometry_core PRIVATE 
  -pthread
  $<$<CONFIG:Release>:-O2>
  $<$<CONFIG:Release>:-fno-fast-math>
  $<$<CONFIG:Release>:-ffp-contract=off>
)
target_link_libraries(slam_odometry_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  TBB::tbb
  glog
)
target_include_directories(slam_odometry_core PUBLIC
  include
  ${FASTER_LIO_INCLUDE}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(slam_odometry_node src/slam_odometry_node.cpp)
add_dependencies(slam_odometry_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_odometry_node slam_odometry_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS slam_odometry_core slam_odometry_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
