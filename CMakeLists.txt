cmake_minimum_required(VERSION 3.0.2)
project(dapozzo_line_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  sensor_msgs
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/line_tracking/nodes/planner_node.py
  src/line_tracking/nodes/control_node.py
  src/line_tracking/nodes/manual_control_node.py
  src/line_tracking/planning_strategies/centerline_strategy.py
  src/line_tracking/planning_strategies/centroid_strategy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()