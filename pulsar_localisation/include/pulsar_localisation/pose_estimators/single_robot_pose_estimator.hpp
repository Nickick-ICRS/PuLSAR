#ifndef __SINGLE_ROBOT_POSE_ESTIMATOR__
#define __SINGLE_ROBOT_POSE_ESTIMATOR__

#include <memory>
#include <cmath>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "cloud_generator/cloud_generator.hpp"

/**
 * @brief Estimates the pose of a single robot.
 *
 * Class to estimate the pose of a single robot within a swarm of robots.
 * Uses laser scan and odometry information along with knowledge of the map
 * environment to estimate the pose of the robot.
 */
class SingleRobotPoseEstimator {
public:
    /**
     * Constructor for the SingleRobotPoseEstimator class.
     *
     * @param name The name of the robot (as used by cloud_gen)
     *
     * @param cloud_gen Pointer to an initialised cloud generator class
     *                  instance.
     *
     * @param initial_pose An initial estimate of the robot's pose relative
     *                     to the map frame.
     *
     * @param odom_topic The topic on which odometry measurements from the
     *                   robot are published.
     */
    SingleRobotPoseEstimator(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        std::string map_frame, geometry_msgs::Pose initial_pose,
        std::string odom_topic);
    ~SingleRobotPoseEstimator();
    
    /**
     * @brief Update robot pose estimate.
     *
     * Update estimate of the pose of this robot. Does *not* publish
     * anything to TF.
     */
    void update_estimate();

    /**
     * Get the pose estimate.
     */
    geometry_msgs::PoseWithCovarianceStamped& get_pose_estimate() {
        return pose_estimate_;
    };
private:
    /**
     * Odometry callback from the robot.
     *
     * @param msg The odometry message.
     */
    void odom_cb(const nav_msgs::OdometryConstPtr& msg);

    /**
     * @brief Runs an iteration of the augmented MCL algorithm.
     *
     * Runs one iteration of the augmented Monte Carlo Localisation
     * algorithm, finding a cloud of points all of which may be the pose
     * of this robot. Robust against local minima due to additions of
     * random additional poses. See Probabilistic Robotics by Thrun et al.
     *
     * @param Xt_1 The previous set of pose estimates.
     *
     * @param ut The most recent odometry measurements.
     *
     * @return The new (calculated) set of pose estimates.
     */
    std::vector<geometry_msgs::Pose> augmented_MCL(
        const std::vector<geometry_msgs::Pose>& Xt_1,
        const nav_msgs::Odometry& ut);

    /**
     * @brief Generates a random valid pose for the robot.
     * 
     * Uses knowledge of the map to generate a random valid (i.e. not in a 
     * wall) pose that the robot may be in. Does not take into consideration
     * the poses of other robots in the swarm.
     *
     * @return A valid pose for the robot.
     */
    geometry_msgs::Pose gen_random_valid_pose();

    /**
     * @brief Calculate the probability of a occuring.
     *
     * Calculate the probability of a in the normal distribution with
     * variance b2. See Probabilistic Robotics by Thrun et al.
     * 
     * @param a The event that the probability of occuring is being
     *          calculated for.
     * 
     * @param b2 The variance of the normal distribution.
     *
     * @return p(a) in the normal distribution defined by b2.
     */
    double prob_normal_distribution(double a, double b2);

    /**
     * @brief Samples a value from a normal distribution.
     *
     * Samples a value from the normal distribution defined by b2.
     *
     * @param b2 The variance of the normal distribution.
     *
     * @return A random number from the distribution.
     */
    double sample_normal_distribution(double b2);

    /**
     * @brief Samples from p(xt | ut, xt-1).
     *
     * Sample from p(xt | ut, xt-1) given odometry information. See 
     * Probabilistic Robotics by Thrun et al.
     *
     * @param ut The most recent odometry measurement.
     *
     * @param xt_1 The most recent pose estimate.
     *
     * @return The current (new) pose estimate.
     */
    geometry_msgs::Pose sample_motion_model_odometry(
        const nav_msgs::Odometry& ut,
        const geometry_msgs::Pose& xt_1);

    std::string name_;
    const std::shared_ptr<CloudGenerator> cloud_gen_;
    geometry_msgs::PoseWithCovarianceStamped pose_estimate_;

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;

    std::random_device rd_;
    std::mt19937_64 gen_;

    // Robot noise parameters
    static double a1_, a2_, a3_, a4_;
    // Exponential parameters for Augmented MCL, 0 <= aslow < afast
    static double aslow_, afast_;

    // Saves us a lot of constructor parameters by having the above params
    // be static and making this class able to access them.
    friend class LocalisationNode;
};

#endif // __SINGLE_ROBOT_POSE_ESTIMATOR__
