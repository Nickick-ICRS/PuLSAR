cmake_minimum_required(VERSION 2.8.3)
project(pulsar_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS gazebo_ros
)
