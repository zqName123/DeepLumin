cmake_minimum_required(VERSION 3.0)
project(lio_odometry_voxelmap)

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
    syslog_ros_wrap
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

set(YAML_CPP_LIB "/usr/lib/x86_64-linux-gnu/libyaml-cpp.so")
set(GLOG_LIB "/usr/lib/x86_64-linux-gnu/libglog.so")
set(TBB_LIB "/usr/lib/x86_64-linux-gnu/libtbb.so")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lio_odometry_voxelmap_core
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

add_library(lio_odometry_voxelmap_core
    src/core/eskf.cpp
    src/core/imu_processing.cpp
    src/core/pointcloud_preprocess.cpp
    src/core/laser_odometry_core.cpp
    src/core/correction_handler.cpp
    src/core/pcl_instantiate.cpp
    src/core/voxelmap/voxel_map_fastlivo.cpp
)

target_link_libraries(lio_odometry_voxelmap_core
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIB}
    ${GLOG_LIB}
    ${TBB_LIB}
    OpenMP::OpenMP_CXX
)

add_executable(lio_odometry_voxelmap_node
    src/ros/lio_odometry_node.cpp
    src/ros/ros_adapter.cpp
    src/ros/config.cpp
)

target_link_libraries(lio_odometry_voxelmap_node
    lio_odometry_voxelmap_core
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIB}
    ${GLOG_LIB}
    ${TBB_LIB}
    OpenMP::OpenMP_CXX
)

if(CATKIN_ENABLE_TESTING)
    add_executable(lio_voxelmap_optimization_regression EXCLUDE_FROM_ALL tests/optimization_regression.cpp)
    target_link_libraries(lio_voxelmap_optimization_regression lio_odometry_voxelmap_core ${PCL_LIBRARIES} ${YAML_CPP_LIB} OpenMP::OpenMP_CXX)
    add_test(NAME lio_voxelmap_optimization_regression
        COMMAND lio_voxelmap_optimization_regression ${CMAKE_CURRENT_SOURCE_DIR}/config/lio_odometry.yaml)
    set_tests_properties(lio_voxelmap_optimization_regression PROPERTIES ENVIRONMENT "OMP_NUM_THREADS=2")
endif()
