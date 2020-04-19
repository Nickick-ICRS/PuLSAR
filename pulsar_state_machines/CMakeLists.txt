cmake_minimum_required(VERSION 2.8.3)
project(pulsar_state_machines)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy geometry_msgs nav_msgs sensor_msgs
)
