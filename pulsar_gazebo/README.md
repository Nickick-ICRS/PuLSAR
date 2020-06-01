# pulsar_gazebo

This package contains files relevant to simulating PuLSAR in the Gazebo robot simulator.

## Launch

The launch folder contains the roslaunch files necessary to launch the simulation environment. `one_robot.launch` launches a single robot, while `multiple_robots.launch` will spawn multiple robots.

### One Robot

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
- map_file - Which map file to use. String. Default is "<location of pulsar_gazebo>/maps/map.yaml".

### Multiple Robots

This launch file spawns a swarm of pulsar robots in the Gazebo simulator, and starts the simulator. The robots are spawned in random (valid) positions. It exposes the following parameters:

- num_robots - The numberof robots to spawn. Integer. Default is 5.
- run_state_machine - Should the launch file spawn a state machine for the robot. Boolean. Default is true.
- state_machine_type - Which state machine should be spawned (see pulsar_state_machines). String. Default is "default". Options are:
    - "default" - Random movement state machine.
    - "group_target" - State machine which attempts to group with other robots and move towards a target point.
- map_file - Which map file to use. String. Default is "<location of pulsar_gazebo>/maps/map.yaml".

## Maps

This folder contains the map file for the testing arena. This is called `map.yaml`, and the occupancy grid image is `map.png`.

## Plugins

This folder contains the gazebo robot plugin. The plugin is used in the robot URDF to connect the robot to ROS. It has the following sdf parameters:

- namespace - The robot name(space). String. Default is ''.
- odomTopic - The topic on which to publish nav_msgs/Odometry information. String. Default is 'odometry/filtered'.
- cmdVelTopic - The topic on which to listen for geometry_msgs/Twist information. String. Default is 'cmd_vel'.
- imuTopic - The topic on which to listen for sensor_msgs/Imu information. String. Default is 'imu/data'.
- Q_x, Q_y, Q_th, Q_dot and Q_th_dot - The EKF process noise Q. Decimals. Defaults are 0.
- motorKp - Motor P control constant. Decimal. Default is 1.
- motorKi - Motor I control constant. Decimal. Default is 1.
- cpr - Counts per revolution of the simulated encoders. Integer. Default is 4000.
- wheelRadius - The radius of the simulated wheels in meters. It is important that this matches the URDF model! Decimal. Default is 0.015.
- wheelSeparation - The distance between the simulated wheel centers in meters. It is important that this matches the URDF model! Decimal. Default is 0.015.
- updateRate - The rate at which odometry calculations should be updated in Hz. Decimal. Default is 10.
- leftWheelJointName - The name of the left wheel joint. It is important that this matches the URDF model! String. Default is 'left_wheel_joint'.
- rightWheelJointName - The name of the left wheel joint. It is important that this matches the URDF model! String. Default is 'right_wheel_joint'.
- motorMaxTorque - The maxiumum torque able to be applied per motor, including gear ratio in Nm. Decimal. Default is 1.
- odommUniformNoise - The uniform percentage noise to be added to odometry measurements. Must be less than 1 (100%). Decimal. Default is 0.
- robotTfPrefix - The robot TF prefix. String. Default is "".

### Publishers and Subscribers

The plugin creates the following publishers:

- "<namespace>/<odomTopic>" - Odometry measurements (nav_msgs/Odometry).

The plugin creates the following subscribers:
- "<namespace>/<cmdVelTopic>" - Velocity commands (geometry_msgs/Twist).
- "<namespace>/<imuTopic>" - Imu data (sensor_msgs/Imu).

## Scripts

This package has two scripts. The first `spawn_pulsar_robot.py` spawns a pulsar robot in gazeboby procedurally calling the one robot launch file. It is not intended to be called by humans and is called bythe multiple robot launch file.

The second script `pid_graph.py` is helpful for tuning PI values of the motors. With a single running robot (called PuLSAR) in gazebo, it plots the current speed at which the wheels are turning in radians.

## Worlds

This contains the world used for simulation and testing. It is a simple 2x2 meter square with a few obstacles in it.
