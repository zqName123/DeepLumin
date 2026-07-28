cmake_minimum_required(VERSION 3.0.2)
project(mapping_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  deeplumin_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs deeplumin_msgs
)

catkin_install_python(PROGRAMS
  scripts/mapping_gui_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
