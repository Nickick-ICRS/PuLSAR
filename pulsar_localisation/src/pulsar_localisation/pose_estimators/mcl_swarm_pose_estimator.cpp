#include "pose_estimators/mcl_swarm_pose_estimator.hpp"

MCLSwarmPoseEstimator::MCLSwarmPoseEstimator(
    const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::vector<std::string>& robot_names, 
    std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
    std::map<std::string, std::string>& robot_odom_topics,
    std::map<std::string, std::string>& robot_base_links,
    std::map<std::string, float>& robot_radii, unsigned int M)

    :SwarmPoseEstimator(cloud_gen, map_man, map_frame)
{

}

MCLSwarmPoseEstimator::~MCLSwarmPoseEstimator() {
    // dtor
}

void MCLSwarmPoseEstimator::update_estimate() {

}
