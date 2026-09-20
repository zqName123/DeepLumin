cmake_minimum_required(VERSION 3.0)
project(lio_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    tf
    pcl_conversions
    pcl_ros
    deeplumin_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(YAML_CPP_LIB "/usr/lib/x86_64-linux-gnu/libyaml-cpp.so")
set(GLOG_LIB "/usr/lib/x86_64-linux-gnu/libglog.so")
set(TBB_LIB "/usr/lib/x86_64-linux-gnu/libtbb.so")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lio_odometry_core
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf deeplumin_msgs pcl_conversions
    DEPENDS PCL Eigen3 yaml-cpp
)

include_directories(
    include
    thirdparty
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../slam_odometry_v2/include
)

add_library(lio_odometry_core
    src/core/eskf.cpp
    src/core/imu_processing.cpp
    src/core/pointcloud_preprocess.cpp
    src/core/laser_odometry_core.cpp
    src/core/correction_handler.cpp
    src/core/pcl_instantiate.cpp
)

target_link_libraries(lio_odometry_core
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIB}
    ${GLOG_LIB}
    ${TBB_LIB}
)

add_executable(lio_odometry_node
    src/ros/lio_odometry_node.cpp
    src/ros/ros_adapter.cpp
    src/ros/config.cpp
)

target_link_libraries(lio_odometry_node
    lio_odometry_core
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIB}
    ${GLOG_LIB}
    ${TBB_LIB}
)