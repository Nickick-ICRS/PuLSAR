cmake_minimum_required(VERSION 2.8.3)
project(pulsar_server)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    nav_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs nav_msgs
)

install(PROGRAMS
  nodes/single_robot_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
