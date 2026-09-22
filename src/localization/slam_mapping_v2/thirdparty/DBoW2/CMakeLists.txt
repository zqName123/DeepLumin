# DBoW2 - 子目录构建版本（从 ORB_SLAM3 移植）
cmake_minimum_required(VERSION 3.0.2)

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h
  DBoW2/FClass.h
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h
  DBoW2/TemplatedVocabulary.h)

set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)

set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

add_library(DBoW2 STATIC ${SRCS_DBOW2} ${SRCS_DUTILS})
target_include_directories(DBoW2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(DBoW2 ${OpenCV_LIBRARIES})
target_compile_options(DBoW2 PRIVATE -Wall -O2)
