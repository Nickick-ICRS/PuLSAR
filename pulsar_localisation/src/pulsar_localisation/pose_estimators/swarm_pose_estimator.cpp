#include "pose_estimators/swarm_pose_estimator.hpp"

SwarmPoseEstimator::SwarmPoseEstimator(
    const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::vector<std::string>& robot_names, 
    std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
    std::map<std::string, std::string>& robot_odom_topics,
    std::map<std::string, std::string>& robot_base_links,
    std::map<std::string, float>& robot_radii) 

    :cloud_gen_(cloud_gen), map_frame_(map_frame)
{
    for(const std::string& name : robot_names) {
        robot_pose_estimators_[name].reset(new SingleRobotPoseEstimator(
            name, cloud_gen, map_man, map_frame, initial_robot_poses[name],
            robot_odom_topics[name], robot_base_links[name], 
            robot_radii[name]));
        robot_pose_estimates_[name] = 
            robot_pose_estimators_[name]->get_pose_estimate();
    }
}

SwarmPoseEstimator::~SwarmPoseEstimator() {
    // dtor
}

void SwarmPoseEstimator::update_estimate() {

}
