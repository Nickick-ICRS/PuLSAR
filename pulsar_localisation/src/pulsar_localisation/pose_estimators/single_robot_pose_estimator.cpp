#include "pose_estimators/single_robot_pose_estimator.hpp"

#include "maths/useful_functions.hpp"

#include <ros/ros.h>

SingleRobotPoseEstimator::SingleRobotPoseEstimator(
    std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::string odom_topic, std::string base_link_frame, float radius,
    double min_trans_update, geometry_msgs::Pose initial_pose)

    :RobotModel(name, cloud_gen, map_man, map_frame, odom_topic, 
    base_link_frame, radius, min_trans_update)
{
    pose_estimate_.pose.pose = initial_pose;
    pose_estimate_.header.frame_id = map_frame;
    pose_estimate_.header.stamp = ros::Time::now();

    pose_estimate_.pose.covariance[0]  = 1e-6;
    pose_estimate_.pose.covariance[7]  = 1e-6;
    pose_estimate_.pose.covariance[35] = 1e-6;

    map_man_->set_robot_pose(name, initial_pose, radius);
}

SingleRobotPoseEstimator::~SingleRobotPoseEstimator() {
    // dtor
}

geometry_msgs::TransformStamped 
    SingleRobotPoseEstimator::calculate_transform(
        const nav_msgs::Odometry& odom)
{
    double mRo = quat_to_yaw(pose_estimate_.pose.pose.orientation)
               - quat_to_yaw(odom.pose.pose.orientation);

    // Create the message and return
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom.header.frame_id;
    double oTbx = odom.pose.pose.position.x * cos(mRo)
                - odom.pose.pose.position.y * sin(mRo);
    double oTby = odom.pose.pose.position.y * cos(mRo)
                + odom.pose.pose.position.x * sin(mRo);

    tf.transform.translation.x =
        pose_estimate_.pose.pose.position.x - oTbx;
    tf.transform.translation.y =
        pose_estimate_.pose.pose.position.y - oTby;
    tf.transform.rotation = yaw_to_quat(mRo);
    return tf;
}
