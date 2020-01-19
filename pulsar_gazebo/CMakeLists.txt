cmake_minimum_required(VERSION 2.8.3)
project(pulsar_gazebo)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    gazebo_plugins
    roscpp
    nav_msgs
    geometry_msgs
    pulsar_lib
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED)

catkin_package(
    CATKIN_DEPENDS gazebo_ros gazebo_plugins roscpp nav_msgs geometry_msgs pulsar_lib
    DEPENDS Boost gazebo
)

include_directories(
    plugins 
    ${catkin_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIR} 
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 11)

add_library(pulsar_gazebo_plugin SHARED
    plugins/pulsar_gazebo_plugin.cpp
)
target_link_libraries(pulsar_gazebo_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)
