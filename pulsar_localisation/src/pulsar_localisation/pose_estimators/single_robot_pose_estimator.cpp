#include "pose_estimators/single_robot_pose_estimator.hpp"

SingleRobotPoseEstimator::SingleRobotPoseEstimator(
    std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
    std::string map_frame, geometry_msgs::Pose initial_pose,
    std::string odom_topic)
    
    :name_(name), cloud_gen_(cloud_gen)
{
    pose_estimate_.pose = initial_pose;
    pose_estimate_.header.frame_id = map_frame;
    pose_estimate_.header.stamp = ros::Time::now();

    odom_sub_ = nh_.subscribe(
        odom_topic, 1, &SingleRobotPoseEstimator::odom_cb, this);
}

SingleRobotPoseEstimator::~SingleRobotPoseEstimator() {
    // dtor
}

void SingleRobotPoseEstimator::update_estimate() {
    
}

void SingleRobotPoseEstimator::odom_cb(
    const nav_msgs::OdometryConstPtr& msg)
{

}
