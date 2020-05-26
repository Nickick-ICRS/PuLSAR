#ifndef __SINGLE_ROBOT_POSE_ESTIMATOR_HPP__
#define __SINGLE_ROBOT_POSE_ESTIMATOR_HPP__

#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "robot_models/odometry_robot_model.hpp"
#include "robot_models/scan_matching_robot_model.hpp"

#include "maths/useful_functions.hpp"

#include <ros/ros.h>

/**
 * Abstract class to represent a single robot pose estimator.
 */

template<class T>
class SingleRobotPoseEstimator :public OdometryRobotModel {
public:
    SingleRobotPoseEstimator(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::string odom_topic, std::string base_link_frame, float radius,
        double min_trans_update, geometry_msgs::Pose initial_pose)

        :OdometryRobotModel(
            name, cloud_gen, map_man, map_frame, odom_topic, 
            base_link_frame, radius, min_trans_update)
    {
        throw("Unrecognised class type!");
    }

    virtual ~SingleRobotPoseEstimator() {};

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
};


template<>
class SingleRobotPoseEstimator <OdometryRobotModel>
    :public OdometryRobotModel {
public:
    /**
     * Constructs a SingleRobotPoseEstimator instance.
     */
    SingleRobotPoseEstimator(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::string odom_topic, std::string base_link_frame, float radius,
        double min_trans_update, geometry_msgs::Pose initial_pose)

        :OdometryRobotModel(
            name, cloud_gen, map_man, map_frame, odom_topic, 
            base_link_frame, radius, min_trans_update)
    {
        pose_estimate_.pose.pose = initial_pose;
        pose_estimate_.header.frame_id = map_frame;
        pose_estimate_.header.stamp = ros::Time::now();

        pose_estimate_.pose.covariance[0]  = 1e-6;
        pose_estimate_.pose.covariance[7]  = 1e-6;
        pose_estimate_.pose.covariance[35] = 1e-6;

        map_man_->set_robot_pose(name, initial_pose, radius);
    };

    virtual ~SingleRobotPoseEstimator() {};

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
};


template<>
class SingleRobotPoseEstimator <ScanMatchingRobotModel> 
    :public ScanMatchingRobotModel {
public:
    /**
     * Constructs a SingleRobotPoseEstimator instance.
     */
    SingleRobotPoseEstimator(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::string odom_topic, std::string base_link_frame, float radius,
        double min_trans_update, geometry_msgs::Pose initial_pose)

        :ScanMatchingRobotModel(
            name, cloud_gen, map_man, map_frame, odom_topic, 
            base_link_frame, radius, min_trans_update)
    {
        pose_estimate_.pose.pose = initial_pose;
        pose_estimate_.header.frame_id = map_frame;
        pose_estimate_.header.stamp = ros::Time::now();

        pose_estimate_.pose.covariance[0]  = 1e-6;
        pose_estimate_.pose.covariance[7]  = 1e-6;
        pose_estimate_.pose.covariance[35] = 1e-6;

        map_man_->set_robot_pose(name, initial_pose, radius);
    };

    virtual ~SingleRobotPoseEstimator() {};

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
};

#endif // __SINGLE_ROBOT_POSE_ESTIMATOR_HPP__
