cmake_minimum_required(VERSION 2.8.3)
project(pulsar_localisation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs sensor_msgs pcl_ros pcl_conversions tf2_ros
)

include_directories(
  include/pulsar_localisation
  ${catkin_INCLUDE_DIRS}
)

add_library(pulsar_funcs
    src/pulsar_localisation/maths/useful_functions.cpp
)

add_dependencies(pulsar_funcs
    ${pulsar_funcs_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pulsar_funcs
    ${catkin_LIBRARIES}
)

add_library(pulsar_cloud_generator
    src/pulsar_localisation/cloud_generator/cloud_generator.cpp
)

add_dependencies(pulsar_cloud_generator
    ${pulsar_cloud_generator_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pulsar_cloud_generator
    ${catkin_LIBRARIES}
)

add_library(pulsar_map_manager
    src/pulsar_localisation/map_manager/map_manager.cpp
)

add_dependencies(pulsar_map_manager
    ${pulsar_map_manager_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pulsar_map_manager
    pulsar_funcs
    ${catkin_libraries}
)

add_library(pulsar_sensor_models
    src/pulsar_localisation/sensor_models/range_cloud_sensor_model.cpp
)

add_dependencies(pulsar_sensor_models
    ${pulsar_sensor_models_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pulsar_sensor_models
    pulsar_cloud_generator
    pulsar_map_manager
    pulsar_funcs
    ${catkin_libraries}
)

add_library(pulsar_pose_estimators
    src/pulsar_localisation/pose_estimators/swarm_pose_estimator.cpp
    src/pulsar_localisation/pose_estimators/single_robot_pose_estimator.cpp
)

add_dependencies(pulsar_pose_estimators
    ${pulsar_pose_estimators_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pulsar_pose_estimators
    pulsar_cloud_generator
    pulsar_map_manager
    pulsar_sensor_models
    pulsar_funcs
    ${catkin_LIBRARIES}
)

add_executable(pulsar_localisation_node
    src/pulsar_localisation/localisation_node.cpp
)

add_dependencies(pulsar_localisation_node
    ${pulsar_localisation_node_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pulsar_localisation_node
    pulsar_cloud_generator
    pulsar_map_manager
    pulsar_pose_estimators
    pulsar_sensor_models
    ${catkin_LIBRARIES}
)

set_target_properties(pulsar_localisation_node
    PROPERTIES OUTPUT_NAME localisation_node
    PREFIX ""
)
