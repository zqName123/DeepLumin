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
    deeplumin_health_checker
    syslog_ros_wrap
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)

# glog 系统未提供 cmake config，改用 find_library/find_path 探测
find_path(GLOG_INCLUDE_DIR glog/logging.h)
find_library(GLOG_LIBRARY NAMES glog)

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
    ${GLOG_INCLUDE_DIR}
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
    yaml-cpp
    ${GLOG_LIBRARY}
    TBB::tbb
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
    yaml-cpp
    ${GLOG_LIBRARY}
    TBB::tbb
)
if(CATKIN_ENABLE_TESTING)
    add_executable(lio_odometry_regression EXCLUDE_FROM_ALL tests/regression.cpp)
    target_link_libraries(lio_odometry_regression lio_odometry_core ${PCL_LIBRARIES} yaml-cpp ${GLOG_LIBRARY} TBB::tbb)
    add_test(NAME lio_odometry_regression
        COMMAND lio_odometry_regression ${CMAKE_CURRENT_SOURCE_DIR}/config/lio_odometry.yaml)
endif()