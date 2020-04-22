#include "pose_estimators/average_mcl_swarm_pose_estimator.hpp"

#include "maths/useful_functions.hpp"

#include <cmath>

AverageMCLSwarmPoseEstimator::AverageMCLSwarmPoseEstimator(
    const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::vector<std::string>& robot_names, 
    std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
    std::map<std::string, std::string>& robot_odom_topics,
    std::map<std::string, std::string>& robot_base_links,
    std::map<std::string, float>& robot_radii, unsigned int M,
    double min_trans_update)

    :SwarmPoseEstimator(
        cloud_gen, map_man, map_frame)
{
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "swarm_pose_estimate", 1);
    for(const std::string& name : robot_names) {
        robot_pose_estimators_[name].reset(new MCLSingleRobotPoseEstimator(
            name, cloud_gen, map_man, map_frame, initial_robot_poses[name],
            robot_odom_topics[name], robot_base_links[name], 
            robot_radii[name], M, min_trans_update));
        robot_pose_estimates_[name] = 
            robot_pose_estimators_[name]->get_pose_estimate();
        worker_threads_[name] = std::thread();
    }

    swarm_pose_estimate_.header.frame_id = map_frame;
}

AverageMCLSwarmPoseEstimator::~AverageMCLSwarmPoseEstimator() {
    // dtor
}

void AverageMCLSwarmPoseEstimator::update_estimate() {
    // Run all the update functions in threads
    for(auto& pair : worker_threads_) {
        pair.second = std::thread(
            &MCLSingleRobotPoseEstimator::update_estimate,
            robot_pose_estimators_[pair.first]);
    }
    for(auto& pair : worker_threads_) {
        pair.second.join();
        robot_pose_estimates_[pair.first] = 
            robot_pose_estimators_[pair.first]->get_pose_estimate();
    }
    update_estimate_covariance();
}
