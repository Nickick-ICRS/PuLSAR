#include "pose_estimators/swarm_pose_estimator.hpp"

#include "maths/useful_functions.hpp"

#include <cmath>

SwarmPoseEstimator::SwarmPoseEstimator(
    const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame)

    :cloud_gen_(cloud_gen), map_frame_(map_frame)
{
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "swarm_pose_estimate", 1);

    swarm_pose_estimate_.header.frame_id = map_frame;
}

SwarmPoseEstimator::~SwarmPoseEstimator() {
    // dtor
}

void SwarmPoseEstimator::update_estimate_covariance() {
    // For calculating avg pose
    geometry_msgs::PoseWithCovariance avg_pose;
    auto& pos = avg_pose.pose.position;
    double avg_yaw;
    for(const auto& pair : robot_pose_estimates_) {
        const auto& pose = robot_pose_estimates_[pair.first].pose.pose;
        // Store new pose estimate
        // Calculate average
        pos.x += pose.position.x;
        pos.y += pose.position.y;
        pos.z += pose.position.z;
        avg_yaw += quat_to_yaw(pose.orientation);
    }
    double n = robot_pose_estimates_.size();
    avg_pose.pose.position.x /= n;
    avg_pose.pose.position.y /= n;
    avg_pose.pose.position.z /= n;

    avg_yaw /= n;
    while(avg_yaw > M_PI)   avg_yaw -= 2*M_PI;
    while(avg_yaw <= -M_PI) avg_yaw += 2*M_PI;

    avg_pose.pose.orientation = yaw_to_quat(avg_yaw);

    // Helper lambda for calculating the difference between two angles
    static auto ang_diff = [](double a, double b) {
        double diff = a-b;
        if(diff > M_PI/2)   diff -= M_PI;
        if(diff <= -M_PI/2) diff += M_PI;
        return diff;
    };

    // Calculate covariance matrix
    auto& cov_mat = avg_pose.covariance;

    for(const auto& pair : robot_pose_estimates_) {
        const auto& p = pair.second.pose.pose;
        double yaw_diff = ang_diff(quat_to_yaw(p.orientation), avg_yaw);
        cov_mat[0] += pow(p.position.x - pos.x, 2) / n;
        cov_mat[1] += (p.position.x - pos.x) * (p.position.y - pos.y) / n;
        cov_mat[5] += (p.position.x - pos.x) * yaw_diff / n;
        cov_mat[7] += pow(p.position.y - pos.y, 2) / n;
        cov_mat[11] += (p.position.y - pos.y) * yaw_diff / n;
        cov_mat[35] += pow(yaw_diff, 2) / n;
    }

    // Symmetry
    cov_mat[6]  = cov_mat[1];  // y-x , x-y
    cov_mat[30] = cov_mat[5];  // th-x, x-th
    cov_mat[31] = cov_mat[11]; // th-y, y-th

    swarm_pose_estimate_.pose = avg_pose;
    swarm_pose_estimate_.header.stamp = ros::Time::now();
}

void SwarmPoseEstimator::publish_pose_estimate() {
    pose_pub_.publish(swarm_pose_estimate_);
}
