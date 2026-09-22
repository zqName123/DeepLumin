cmake_minimum_required(VERSION 3.5)
project(syslog_ros_wrap)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS syslog/include
  LIBRARIES syslog
)

set(SYSLOG_SKIP_PROJECT ON)
add_subdirectory(syslog)