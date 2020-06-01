# pulsar_lib

This package contains hardware-abstracted classes which are used both in simulation and on hardware. These libraries are used in pulsar_hardware and pulsar_gazebo.

## KalmanFilter

This class uses the Eigen vector mathematics library to run a custom EKF for the robots in this project. See header file for documentation.

## MotorControl

This class runs a PI controller for an arbitrary revolute joint. It requiresa joint implementation which receives the percentage power output. This may be either the simulated joint torque, or real PWM duty cycle percentage. See header file for documentation.

## Odometry

This class calculates odometry from encoder input, and calculates required wheel velocities for requested command velocities. It needs to be regularly updated via the update function.

The class expects to have one of the count functions called whenever a new (real or simulated) encoder pulse arrives. This requires knowledge of the direction (e.g. via quadrature encoding).

See header file for documentation.
