cmake_minimum_required(VERSION 3.0.2)
project(deeplumin_cmake)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS deeplumin_dependencies.cmake
)

install(DIRECTORY cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
