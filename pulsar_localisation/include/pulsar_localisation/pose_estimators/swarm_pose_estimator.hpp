#ifndef __SWARM_POSE_ESTIMATOR_HPP__
#define __SWARM_POSE_ESTIMATOR_HPP__

#include <map>
#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "pose_estimators/single_robot_pose_estimator.hpp"
#include "cloud_generator/cloud_generator.hpp"

/**
 * @brief Estimates the pose of the whole swarm.
 *
 * Class to estimate the pose of an entire swarm of robots. Estimates of the
 * pose of individual robots are combined with data points fromt the swarm
 * as a whole to estimate the pose of the whole swarm. From this, errors can
 * be found in pose estimates of swarm members and these may be iteratively
 * updated to improve pose estimates.
 */
class SwarmPoseEstimator {
public:
    /**
     * Constructor for the SwarmPoseEstimator class.
     * 
     * @param cloud_gen Pointer to an initialised cloud generator class 
     *                  instance.
     * 
     * @param robot_names Names of the different robots in the swarm.
     * 
     * @param initial_robot_poses Map of starting poses of the robots in 
     *                            the swarm keyed by name.
     *
     * @param robot_odom_topics Map of the topic names of robot odometry 
     *                          measurements keyed by name.
     *
     * @param map_frame The map frame of the whole swarm.
     */
    SwarmPoseEstimator(
        const std::shared_ptr<CloudGenerator>& cloud_gen,
        std::string map_frame, std::vector<std::string> robot_names, 
        std::map<std::string, geometry_msgs::Pose> initial_robot_poses,
        std::map<std::string, std::string> robot_odom_topics);
    ~SwarmPoseEstimator();

    /**
     * @brief Update robot pose estimates and swarm pose estimate.
     *
     * Update estimates of the pose of each robot within the swarm. Does
     * *not* publish anything to TF. Also updates the pose estimate of
     * the entire swarm.
     */
    void update_estimate();

    /**
     * Get the pose estimates of each robot in the swarm.
     */
    std::map<std::string, geometry_msgs::PoseStamped>& get_pose_estimates()
    {
        return robot_pose_estimates_;
    }

    /**
     * Get the pose estimate of the swarm.
     */
    geometry_msgs::PoseStamped& get_pose_estimate() {
        return swarm_pose_estimate_;
    };
private:
    geometry_msgs::PoseStamped swarm_pose_estimate_;

    std::map<std::string, geometry_msgs::PoseStamped> robot_pose_estimates_;

    std::map<std::string, std::shared_ptr<SingleRobotPoseEstimator>>
        robot_pose_estimators_;

    std::string map_frame_;
    std::shared_ptr<CloudGenerator> cloud_gen_;
};

#endif // __SWARM_POSE_ESTIMATOR_HPP__
