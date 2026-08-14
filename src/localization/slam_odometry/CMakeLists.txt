cmake_minimum_required(VERSION 3.0.2)
project(slam_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 与 slam_odometry_v2/faster_lio_ws 一致：-std=c++17



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
  faster_lio
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
# 旧 FasterLIOImpl 编译时引入 vendored thirdparty/faster_lio 头文件（imu_processing/ivox 等），
# 会向 core.so 导出同名 faster_lio::* 符号，与 libfaster_lio.so 的 faster_lio_ws 符号冲突，
# 运行时被符号抢占导致 fastlio2 的 ImuProcess 用了 vendored 版本（垃圾 IMU 状态）。
# 将这两个源文件的符号设为 hidden，仅保留 libfaster_lio.so 的 faster_lio 符号。
set_source_files_properties(
  src/slam/faster_lio_impl.cpp
  src/slam/laser_mapping_core.cpp
  ${FASTER_LIO_ROOT}/src/options.cc
  ${FASTER_LIO_ROOT}/src/utils.cc
  PROPERTIES COMPILE_OPTIONS "-fvisibility=hidden"
)
# target_compile_options(slam_odometry_core PRIVATE 
#   -pthread
#   $<$<CONFIG:Release>:-O2>
#   $<$<CONFIG:Release>:-fno-fast-math>
#   $<$<CONFIG:Release>:-ffp-contract=off>
# )
target_compile_options(slam_odometry_core PRIVATE
  -pthread
  -O3
  -fno-fast-math
  -ffp-contract=off
  -UNDEBUG
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

# fastlio2 适配层：单独编译以隔离 faster_lio_ws 头文件与 vendored thirdparty/faster_lio 的同名头文件
add_library(fastlio2_adapter SHARED src/slam/fastlio2_ws_impl.cpp)
add_dependencies(fastlio2_adapter ${catkin_EXPORTED_TARGETS})
# 必须 BEFORE：把 faster_lio_ws 的 include 放在 vendored thirdparty/faster_lio 之前，
# 否则同名头文件（imu_processing.hpp/ivox3d/so3_math.h 等）会解析到 vendored 版本，
# 造成混合头文件 ODR 冲突（IMU 状态垃圾值）。
target_include_directories(fastlio2_adapter BEFORE PRIVATE
  ${faster_lio_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_definitions(fastlio2_adapter PRIVATE ROOT_DIR="${faster_lio_SOURCE_DIR}/")
target_compile_options(fastlio2_adapter PRIVATE
  -pthread
  -O3
  -fno-fast-math
  -ffp-contract=off
  -UNDEBUG
)
target_link_libraries(fastlio2_adapter
  ${faster_lio_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  TBB::tbb
  glog
  yaml-cpp
  ${PYTHON_LIBRARIES}
)

add_executable(slam_odometry_node src/slam_odometry_node.cpp)
add_dependencies(slam_odometry_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_odometry_node slam_odometry_core fastlio2_adapter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

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
