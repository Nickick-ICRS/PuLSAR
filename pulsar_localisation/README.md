# pulsar_localisation

This contains all of the localisation code developed as part of this project, as well as the localisation launch file.

## Launch

To launch this you first need Gazebo running with robots spawned, or a set of real robots with ROS communication in place. The launch file `localisation.launch` has the following parameters:
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

Fine tuning of algorithm parameters can be done my modifying the launch file. The parameters outlined are as follows:
- map_topic - The topic on which the map is published. String.
- a1 - Robot odometry noise parameter (noise in rotation due to rotation). See Probablistic Robotics (2010). Decimal.
- a2 - Robot odometry noise parameter (noise in rotation due to translation). See Probablistic Robotics (2010). Decimal.
- a3 - Robot odometry noise parameter (noise in translation due to translation). See Probablistic Robotics (2010). Decimal.
- a4 - Robot odometry noise parameter (noise in translation due to rotation). See Probablistic Robotics (2010). Decimal.
- aslow - Slow exponential MCL parameter. See Probabilistic Robotics (2010). Decimal
- afast - Fast exponential MCL parameter. See Probabilistic Robotics (2010). Decimal
- cycle_sensor_readings - How many range sensor readings should be kept in the filter. Must be a multiple of the number of range sensors on a robot (3). 3 Was found the be the best value. Integer.
- time_resolution - Resolution for time keeping in the range sensor model. Decimal.
- robotX - Name of the Xth type of robot within the swarm. Only one type is proposed within the project. Robots will be expected to be in namespaces \<value>n where n is the number of the robot starting at 0. String.
- robotX_quantity - Number of robots in the swarm of type X. Defaults to the value of the parameter \<num_robots>. Integer.
- robotX_odom_topic - Odometry topic for robots of type X. String.
- robotX_base_link_frame - Name of the base link frame for robots of type X.
- robotX_range_topicY - Name of the Yth range finder topic for robots of type X. Theoretically any number of range sensors is supported. String.
- zhit - Intrinsic range sensor parameter. Should be learnt via the learn_range_sensor_intrinsic_parameters script (see below). Decimal.
- zshort - Intrinsic range sensor parameter. Should be learnt via the learn_range_sensor_intrinsic_parameters script (see below). Decimal.
- zmax - Intrinsic range sensor parameter. Should be learnt via the learn_range_sensor_intrinsic_parameters script (see below). Decimal.
- zrand - Intrinsic range sensor parameter. Should be learnt via the learn_range_sensor_intrinsic_parameters script (see below). Decimal.
- sigmahit - Intrinsic range sensor parameter. Should be learnt via the learn_range_sensor_intrinsic_parameters script (see below). Decimal.
- lamshort - Intrinsic range sensor parameter. Should be learnt via the learn_range_sensor_intrinsic_parameters script (see below). Decimal.
- ztime - Parameter to adjust how unreliable old sensor data is considered to be. Must be positive. Large means more unreliable. Decimal.
- lamcont - Parameter representing the importance of pose continuity when scan matching. Larger means more important. Decimal.
- num_particles - The number of particles per robot for AverageMCLSwarmPoseEstimator, or the total number of particles for MCLSwarmPoseEstimator. Integer.
- dbscan_min_points - Minimum number of points for DBSCAN to not ignore as an outlier. Integer.
- dbscan_epsilon - Epsilon value for DBSCAN. Decimal.
- pso_robot_particles - Number of particles per robot for PSOSwarmPoseEstimator. Integer.
- pso_omega - Omega value for the PSO algorithm, a good value is between 0.4 and 0.8. Decimal.
- pso_phi_p - Controls how much the algorithm responds to per particle maxima. Decimal.
- pso_phi_g - Controls how much the algorithm responds to global maxima. Decimal.
- pso_phi_r - Controls how much random velocity is added every time step. Don't make this very big. Decimal.
- pso_max_iterations - Controls the maximum number of iterations per PSO update cycle.
- min_trans_update - Minimum distance the robot must move (in meters) before performing an odometry model update. Helps prevent excessive drift. Decimal.
- update_rate - **Maximum** update rate to run algorithms at in Hz. Decimal.
- robot_localiser - Which robot localisation algorithm to use. Equals the \<localiser> parameter. String.
- robot_model - Which robot model to use. Equals the \<robot_model> parameter. String.

### Publishers and Subscribers

The different localisation algorithms all have similar publisher/subscriber layouts. These are as follows:

Publishers:
- "swarm_pose_estimate" - The swarm pose estimate. geometry_msgs/PoseWithCovarianceStamped.

Subscribers:
- "map" - The map data. nav_msgs/OccupancyGrid.
- "\<robot_name_x>/\<range_finder_topic_y>" - The range finder messages from robots. sensor_msgs/Range.
- "\<robot_name_x>/\<odometry_topic>" - The odometry mmessages from robots. nav_msgs/Odometry.

## Scripts

A couple of useful scripts exist within this package. `plot_robot_movements` and `plot_swarm_movements` plot the motion of an individual robot or swarm respectively, plotting raw odometry, ekf, algorithm and perfect localisation.

The `perfect_swarm_localisation` script simply reports the exact positions of every robot. This is useful for testing state machines.

`learn_range_sensor_intrinsic_parameters` is used to learn the range sensor intrinsic parameters. It waits for a single robot to gather 10,000 data points before performing fitting  and outputting the best intrinsics. It's best to run the random movement state machine on the robot and go to sleep for the night...

## Code Structure

In the include and src folders, there are many C++ files. These are (with the exception of the main node cpp file) laid out in 6 different sub folders.

### localisation_node.cpp

This is the main node which is compiled to make an executable, pulling in everything else. To add a new algorithm, you'll need to make sure it fits in here (and inherits from the correct pose_estimator class), and modify the CMakeLists.txt. The parameters for the node etc. are explained in the launch file section above.

### CloudGenerator

This contains the CloudGenerator class, which receives and stores the range finder data. This is then accessible by any class with a shared pointer to the CloudGenerator instance. See the header file for detailed documentation.

### MapManager

This contains the MapManager class, which receives and stores the map data. It also stores a map for each robot which contains the estimated poses of all other robots as occupied. This may then be accessed from any class with a shared pointer to the MapManager instance. See the header file for detailed docuentation.

### Maths

#### DBSCAN

This contains a slightly modified version of [James Yoo's DBSCAN implementation](https://github.com/james-yoo/DBSCAN).

#### Kmeans

This contains a simple implementation of the popular K-means clustering algorithm. See the header file for detailed documentation.

#### UsefulFunctions

This contains several useful mathematical functions such as sampling from normal distributions or calculating covariances. See the header file for detailed documentation.

### PoseEstimators

There are six classes within this folder, 2 abstract classes, 3 full algorithms and 1 middle-ground class. The algorithm classes are the ones which should be used for localisation.

#### SingleRobotPoseEstimator

This is an abstract class used to represent a pose estimator for a single robot. Currently it is only inherited once. See the header file for detailed documentation.

#### MCLSingleRobotPoseEstimator

Inheriting from SingleRobotPoseEstimator, this class runs an MCL localisation algorithm for a single robot. It is used by the AverageMCLSwarmPoseEstimator algorithm. See the header file for detailed documentation.

#### SwarmPoseEstimator

This is an abstract class used to represent a swarm pose estimator algorithm. All swarm pose estimation algorithms should inherit from this class. See the header file for detailed documentation.

#### AverageMCLSwarmPoseEstimator

This class performs an MCL filter for each robot in the swarm, before averaging the result and using that as the estimate for the swarm pose. This had the best results of all tested algorithms. Both robot models are supported. See the header file for detailed documentation.

#### MCLSwarmPoseEstimator

This class performs a single MCL filter for the whole swarm, before clustering to attempt to find all the robots. It generally performed quite poorly and has issues with losing track of the robots. Both robot models are supported. See the header file for detailed documentation.

#### PSOSwarmPoseEstimator

This class performs Particle Swarm Optimisation to attempt to solve for the optimal position of each robot. It had varying degrees of success, but generally was the worst performing algorithm tested. The cpp file has a boolean variable to adjust whether the robot poses are stored globally or relative to the swarm pose estimate. Globally performed significantly better. See the header file for detailed documentation.

### RobotModels

There are 3 RobotModels which contain motion and sensor models. The first model `BaseRobotModel` is an abstract class which all other RobotModels should inherit from. It defines the key interfaces for all models. See the header file for detailed documentation.

#### OdometryRobotModel

This model is a minor variant of Thrun et al.'s (2010) **odometry_motion_model** and **beam_range_finder_model**. See the header file for detailed documentation.

#### ScanMatchingRobotModel

This model uses the motion model from OdometryRobotModel, but adds in a scan matching step to attempt to adjust poses towards valid map measurements. It tends to perform worse than OdometryRobotModel however. It also wieghs poses based on the least squares error between measurements and map tiles. See the header file for detailed documentation.

### SensorModels

This contains the RangeCloudSensorModel, which models a set of infrared range finder sensors. This can then be used by robot models as needed. See the header file for detailed documentation.
