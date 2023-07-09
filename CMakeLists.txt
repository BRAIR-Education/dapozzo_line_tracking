cmake_minimum_required(VERSION 3.0.2)
project(dapozzo_line_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
)

catkin_install_python(PROGRAMS
  nodes/camera_node.py
  nodes/manual_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
