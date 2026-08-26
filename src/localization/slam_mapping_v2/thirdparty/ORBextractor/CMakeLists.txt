# ORBextractor - 子目录构建版本（从 ORB_SLAM3 移植，命名空间已改为 slam_mapping_v2）
cmake_minimum_required(VERSION 3.0.2)

add_library(ORBextractor STATIC ORBextractor.cc)
target_include_directories(ORBextractor PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ORBextractor ${OpenCV_LIBRARIES})
target_compile_options(ORBextractor PRIVATE -Wall -O2)
