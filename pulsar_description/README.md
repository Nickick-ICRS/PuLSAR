# pulsar_description

This package contains URDF (Universal Robot Description Format) files for robots used to test localisation algorithms as part of the PuLSAR project.

## Meshes

The three .stl files here are the files needed for visualisation of the robot in RVIZ or Gazebo. For full CAD files see the pulsar_cad package.

## URDF

The file `pulsar.urdf.xacro` contains URDF and SDFin Xacro format as a top level description file. It includes each of the other files to make thefull robot description.

This also sets the values for the robot plugin in Gazebo (see pulsar_gazebo for what values do).

### Chassis

This Xacro file creates the chassis and castor wheels of the robot. It also sets up gazebo friction parameters for these  components.

### Imu

This Xacro file creates the IMU sensor plugin (gazebo_ros_imu_sensor) and sets the relevant parameters.

### Laser Range Finder

This Xacro file creates the laser range finder sensors and models forsimulation. The update rate and cone angle parameters can be changed easily, and the gazebo_ros_range plugin is used.

### Wheel

This Xacro file creates the wheels and motors for the simulation. Friction values and joint limits are set too.
