# PuLSAR - Probabilistic Localisation of SwArm Robots

ME4 Master's Project of Nick Hafner

Supervised by Dr Ravi Vaidyanathan

Imperial College London Department of Mechanical Engineering

## Overview of repository

This repository contains the code to localise a swarm of PuLSARs. The code is currently in development, and cannot be relied upon in any way, shape or form.

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

`cd catkin_ws/src && git clone <this package url>`

Install the required dependencies:

`cd PuLSAR && ./install_dependencies.sh`

Finally build the full set of packages:

`cd ../.. && catkin_make`
