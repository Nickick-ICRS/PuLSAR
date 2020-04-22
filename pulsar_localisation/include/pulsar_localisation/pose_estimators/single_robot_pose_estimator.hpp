#ifndef __SINGLE_ROBOT_POSE_ESTIMATOR_HPP__
#define __SINGLE_ROBOT_POSE_ESTIMATOR_HPP__

#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "robot_models/robot_model.hpp"

/**
 * Abstract class to represent a single robot pose estimator.
 */
class SingleRobotPoseEstimator :public RobotModel {
public:
    /**
     * Constructs a SingleRobotPoseEstimator instance.
     */
    SingleRobotPoseEstimator(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::string odom_topic, std::string base_link_frame, float radius,
        double min_trans_update, geometry_msgs::Pose initial_pose);
    virtual ~SingleRobotPoseEstimator();

    /**
     * @brief Update robot pose estimate.
     *
     * Update estimate of the pose of this robot.
     */
    virtual void update_estimate() = 0;

    /**
     * Get the pose estimate.
     */
    geometry_msgs::PoseWithCovarianceStamped& get_pose_estimate() {
        return pose_estimate_;
    }

protected:
    geometry_msgs::PoseWithCovarianceStamped pose_estimate_;

    /**
     * @brief Calculates the transform from the odom frame to the map frame.
     *
     * As we are estimating from the base_link frame to the map frame,
     * using odometry information, our pose estimate is map->base_link.
     * However, we need to publish map->odom, which is what this function
     * calculates.
     *
     * @param odom The most recent odometry message that the filter was 
                  updated with.
     * 
     * @return The transform from the map frame to the frame of the odometry
     *         message (odom).
     */
    geometry_msgs::TransformStamped calculate_transform(
        const nav_msgs::Odometry& odom);
};

#endif // __SINGLE_ROBOT_POSE_ESTIMATOR_HPP__
