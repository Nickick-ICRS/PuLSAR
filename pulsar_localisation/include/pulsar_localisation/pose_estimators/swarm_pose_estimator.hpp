#ifndef __SWARM_POSE_ESTIMATOR_HPP__
#define __SWARM_POSE_ESTIMATOR_HPP__

#include <map>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "pose_estimators/single_robot_pose_estimator.hpp"
#include "cloud_generator/cloud_generator.hpp"

/**
 * @brief Estimates the pose of the whole swarm.
 *
 * Abstract class to represent a pose estimator for the whole swarm.
 */
class SwarmPoseEstimator {
public:
    /**
     * Constructor for the SwarmPoseEstimator class.
     * 
     * @param cloud_gen Pointer to an initialised cloud generator class 
     *                  instance.
     *
     * @param map_man Pointer to an initialised map manager class instance.
     * 
     * @param robot_names Names of the different robots in the swarm.
     * 
     * @param initial_robot_poses Map of starting poses of the robots in 
     *                            the swarm keyed by name.
     *
     * @param robot_odom_topics Map of the topic names of robot odometry 
     *                          measurements keyed by name.
     * 
     * @param robot_base_links Map of the base_link frames of the robots,
     *                         keyed by name.
     *
     * @param robot_radii Map of the robot radii, keyed by name.
     *
     * @param map_frame The map frame of the whole swarm.
     *
     * @param M The number of particles to include in the single robot
     *          pose estimator particle filters.
     *
     * @param min_trans_update Minimum distance for robots to move before
     *                         performing a filter update.
     */
    SwarmPoseEstimator(
        const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame, 
        std::vector<std::string>& robot_names, 
        std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
        std::map<std::string, std::string>& robot_odom_topics,
        std::map<std::string, std::string>& robot_base_links,
        std::map<std::string, float>& robot_radii, unsigned int M,
        double min_trans_update);
    virtual ~SwarmPoseEstimator();

    /**
     * @brief Update robot pose estimates and swarm pose estimate.
     */
    virtual void update_estimate() = 0;

    /**
     * Get the pose estimates of each robot in the swarm.
     *
     * @return Map of the pose estimates, keyed by robot name.
     */
    virtual std::map<std::string, geometry_msgs::PoseWithCovarianceStamped>& get_pose_estimates()
    {
        return robot_pose_estimates_;
    }

    /**
     * Get the pose estimate of the swarm.
     *
     * @return The current pose estimate.
     */
    virtual geometry_msgs::PoseWithCovarianceStamped& get_pose_estimate() {
        return swarm_pose_estimate_;
    };

    /**
     * Publish the swarm pose estimate.
     */
    virtual void publish_pose_estimate();
protected:
    /**
     * @brief Calculate pose estimate covariance.
     *
     * After the individual pose estimators have been updated, this function
     * is called to calculate the average pose and accompanying covariance.
     */
    virtual void update_estimate_covariance();

    geometry_msgs::PoseWithCovarianceStamped swarm_pose_estimate_;

    std::map<std::string, geometry_msgs::PoseWithCovarianceStamped>
        robot_pose_estimates_;

    std::string map_frame_;
    std::shared_ptr<CloudGenerator> cloud_gen_;

    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;

    // For debugging
    friend class LocalisationNode;
};

#endif // __SWARM_POSE_ESTIMATOR_HPP__
