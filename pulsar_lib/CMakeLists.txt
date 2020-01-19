cmake_minimum_required(VERSION 2.8.3)
project(pulsar_lib)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pulsar_kalman_filter pulsar_odometry
    DEPENDS Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(pulsar_kalman_filter
    src/kalman_filter/kalman_filter.cpp
)

add_dependencies(pulsar_kalman_filter
    ${pulsar_kalman_filter_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pulsar_kalman_filter
    ${catkin_LIBRARIES}
    Eigen3::Eigen
)

add_library(pulsar_odometry
    src/odometry/odometry.cpp
)

add_dependencies(pulsar_odometry
    ${pulsar_odometry_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pulsar_odometry
    ${catkin_LIBRARIES}
)

add_library(pulsar_motor_control
    src/motor_control/motor_control.cpp
)

add_dependencies(pulsar_motor_control
    ${pulsar_motor_control_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pulsar_motor_control
    ${catkin_LIBRARIES}
)

install(TARGETS pulsar_kalman_filter pulsar_odometry pulsar_motor_control
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
