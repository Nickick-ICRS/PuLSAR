# PuLSAR - Probabilistic Localisation of SwArm Robots

ME4 Master's Project of Nick Hafner

Supervised by Dr Ravi Vaidyanathan

Imperial College London Department of Mechanical Engineering

## Overview of repository

This repository contains the code to localise a swarm of PuLSARs. The code is research code and no liability can be taken for any losses owing to the direct or indirect usage of this code.

The repository also contains CAD files for the Kicad PCB and Solidworks models.

## Example Usage

First follow the installation instructions below.

### Running Gazebo
(In a new terminal)
To launch a Gazebo-simulated swarm of 20 robots with the Group Motion state machine: `$ roslaunch pulsar_gazebo multiple_robots.launch num_robots:=20 state_machine_type:=group_target`.
To launch a Gazebo-simulated swarm of 10 robots with no state machine running: `$ roslaunch pulsar_gazebo multiple_robots.launch num_robots:=10 run_state_machine:=false`

### Running Localisation Algorithms
(With Gazebo running, in a new terminal)
To launch PSOSwarmPoseEstimator with a swarm of 20 robots: `$ roslaunch pulsar_localisation localisation.launch num_robots:=20 localiser:=pso robot_model:=scan_matching`.
To launch AverageMCLSwarmPoseEstimator with a swarm of 10 robots and the odometry robot model: `$ roslaunch pulsar_localisation localisation.launch num_robots:=10 localiser:=average_swarm_mcl robot_model:=odometry`.
To launch MCLSwarmPoseEstimator with a swarm of 5 robots and the scan matching robot model, debugging using Valgrind//KCacheGrind: `$roslaunch pulsar_localisation localisation.launch num_robots:=5 localiser:=mcl robot_model:=scan_matching valgrind:=true`.

etc...

### Changing Algorithm Parameters

Edit pulsar_localisation's `localisation.launch` file. See the package README or file comments for details on what parameters do.

## Requirements

### Building

This repository used [Catkin](wiki.ros.org/catkin) to build the sub-components. This is installed along with ROS - this set of packages has been written to run with [ROS Melodic](http://wiki.ros.org/melodic/Installation) on **Ubuntu 18.04** with **Bash**. Please start by installing ROS Melodic before attempting to install any of these packages.

Create the following directory layout:
```
catkin_ws
  -> src
```

Run

`cd catkin_ws/src && catkin_init_workspace && cd .. && catkin_make` 

to generate the initial workspace. After this, clone this repository into the src folder:

`cd src && git clone <this package url>`

Install the required dependencies:

`cd PuLSAR && ./install_dependencies.sh`

Finally build the full set of packages:

`cd ../.. && catkin_make`

## Package Layout

### pulsar_cad

This package contains the CAD files needed to manufacture the PuLSAR robots. Solidworks and Kicad were used for mechanical and electrical design respectively. Kicad is free, however Solidworks will require a license to view files.

### pulsar_description

This package contains the stl meshes and urdf files required for the robot description in the ROS ecosystem. URDF has been split up into multiple files, with `urdf/pulsar.urdf.xacro` the top level URDF file.

### pulsar_gazebo

This package contains files relevant to simulating PuLSAR in the Gazebo robot simulator.

#### Launch

The launch folder contains the roslaunch files necessary to launch the simulation environment. `one_robot.launch` launches a single robot, while `multiple_robots.launch` will spawn multiple robots.

##### One Robot

This launch file spawns one pulsar robot with accompanying nodes (not localisation) in the Gazebo simulator, optionally starting the simulator. It is supposed to be called recursively from `multiple_robots.launch`, but exposes the following parameters if a user wishes to use it:

- robot_ns - The name(space) of the robot. String. Default is 'pulsar_0'.
- launch_gazebo - Whether to launch gazebo or not. Boolean. Default is true.
- start_x - The starting x position of the robot. Decimal. Default is 0.
- start_y - The starting y position of the robot. Decimal. Default is 0.
- start_yaw - The starting yaw position of the robot in radians. Decimal. Default is 0.
- start_z - The starting z height of the robot. Useful for preventing robots from clipping through the floor. Decimal. Default is 0.02.
- run_state_machine - Should the launch file spawn a state machine for the robot. Boolean. Default is true.
- state_machine_type - Which state machine should be spawned (see pulsar_state_machines). String. Default is "default". Options are:
    - "default" - Random movement state machine.
    - "group_target" - State machine which attempts to group with other robots and move towards a target point.
- map_file - Which map file to use. String. Default is "\<location of pulsar_gazebo>/maps/map.yaml".

##### Multiple Robots

This launch file spawns a swarm of pulsar robots in the Gazebo simulator, and starts the simulator. The robots are spawned in random (valid) positions. It exposes the following parameters:

- num_robots - The numberof robots to spawn. Integer. Default is 5.
- run_state_machine - Should the launch file spawn a state machine for the robot. Boolean. Default is true.
- state_machine_type - Which state machine should be spawned (see pulsar_state_machines). String. Default is "default". Options are:
    - "default" - Random movement state machine.
    - "group_target" - State machine which attempts to group with other robots and move towards a target point.
- map_file - Which map file to use. String. Default is "\<location of pulsar_gazebo>/maps/map.yaml".

### pulsar_hardware

Contains code to be compiled for the ESP-32 on the robots. Note that due to the impacts of COVID-19 on the university and project, robots could not be tested. As such this package is unfinished and has been included for completion.

### pulsar_lib

Contains custom libraries used both in simulation and in hardware such as EKF, PI controllers and Odometry.

### pulsar_localisation

Contains the developed localisation algorithms and surrounding infrastructure. It also contains the localisation launch file.

#### Launch

The launch file will run one of the localisation algorithms developed as part of this project. This should only be launched once all the robots have successfully been loaded into Gazebo. It has the following command line parameters:
- num_robots - The number of robots to localise. Should match the multiple robots launchfile parameter of the same name. Integer. Default is 1.
- valgrind - Allows the node to be launched under Valgrind for intensive debugging (e.g. function call counts). Takes precedence over gdb. Boolean. Default is false.
- gdb - Allows the node to be launched in gdb for debugging with breakpoints etc. Boolean. Default is false.
- localiser - Which localisation algorithm to use. String. Default is "mcl". Options are:
    - mcl - Runs the MCLSwarmPoseEstimator algorithm.
    - average_swarm_mcl - Runs the AverageMCLSwarmPoseEstimator algorithm. This algorithm was found to perform best.
    - pso - Runs the PSOSwarmPoseEstimator algorithm.
- robot_model - Picks which robot model to use. Note that PSO only supports scan_matching. String. Default is "odometry". Options are:
    - odometry - The odometry motion model (variant of Thrun et al.'s (2010) **odometry_motion_model**). Found to perform best.
    - scan_matching - The scan matching model.

For finer details on this launch file see the pulsar_localisation README.

### pulsar_server

Contains code to connect to robots via TCP, effectively the bridge between ROS and individual robots. Unfinished due to COVID-19, but included for completion.

### pulsar_state_machines

Contains robot state machines for swarm movement. Two state machines exist:
- Random Movement - move randomly
- Group Movement - form a group and move to a target

#### Node

The `state_machine` node runs a state machine for a given robot. It takes the following parameters:
- state_machine - Which state machine to use. String. Default is "default". Options are:
    - default - The default, random movement state machine.
    - group_target - The group motion state machine.
- robot_ns - The robot name(space). String. Default is "pulsar_0".
- update_freq - The state machine update frequency in Hz. Decimal. Default is 10.
- delay - Delay the start of state machine logic in seconds. Decimal. Default is 0.
