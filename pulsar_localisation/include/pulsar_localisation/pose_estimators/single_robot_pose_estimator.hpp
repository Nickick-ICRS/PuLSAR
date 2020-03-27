#ifndef __SINGLE_ROBOT_POSE_ESTIMATOR__
#define __SINGLE_ROBOT_POSE_ESTIMATOR__

#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
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
    geometry_msgs::PoseStamped& get_pose_estimate() {
        return pose_estimate_;
    };
private:
    void odom_cb(const nav_msgs::OdometryConstPtr& msg);
    std::string name_;
    const std::shared_ptr<CloudGenerator> cloud_gen_;
    geometry_msgs::PoseStamped pose_estimate_;

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
};

#endif // __SINGLE_ROBOT_POSE_ESTIMATOR__
