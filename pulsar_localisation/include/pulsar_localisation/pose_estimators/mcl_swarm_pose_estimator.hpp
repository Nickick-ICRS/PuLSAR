#ifndef __MCL_SWARM_POSE_ESTIMATOR_HPP__
#define __MCL_SWARM_POSE_ESTIMATOR_HPP__

#include <map>
#include <memory>

#include <geometry_msgs/Pose.h>

#include "pose_estimators/swarm_pose_estimator.hpp"
#include "cloud_generator/cloud_generator.hpp"

/**
 * @ brief Estimates the pose of the whole swarm.
 *
 * Class to estimate the pose of an entire swarm of robots. Runs a modified
 * MCL filter, where pose estimates are tested among all robots in order
 * to attempt to reduce the total number of points required in the filter.
 */
class MCLSwarmPoseEstimator :public(SwarmPoseEstimator) {
public:
    /**
     * Constructor for the MCLSwarmPoseEstimator class.
     *
     * @param cloud_gen Pointer to an initialised cloud generator class
     *                  instance.
     * 
     * @param map_man Pointer to an initialised map manager class instance.
     *
     * @param map_frame The map frame of the whole swarm.
     *
     * @param robot_names Names of the different robots in the swarm.
     *
     * @param initial_robot_poses Map of the startinng poses of the robots
     *                            in the swarm keyed by name.
     *
     * @param robot_odom_topics Map of the topic names of robot odometry
     *                          measurements keyed by name.
     *
     * @param robot_base_links Map of the robot base_link frames, keyed by
     *                         name.
     *
     * @param robot_radii Map of the robot radii keyed by name.
     *
     * @param M The number of particles to include in the particle filter.
     */
     * 
    MCLSwarmPoseEstimator(
        const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::vector<std::string>& robot_names, 
        std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
        std::map<std::string, std::string>& robot_odom_topics,
        std::map<std::string, std::string>& robot_base_links,
        std::map<std::string, float>& robot_radii, unsigned int M);
    ~MCLSwarmPoseEstimator();

    /**
     * @brief Update swarm pose estimate.
     *
     * Update estimate of the swarm and robots within the swarm. Publishes
     * the estimate to TF.
     */
    void update_estimate();
private:
};

#endif // __MCL_SWARM_POSE_ESTIMATOR_HPP__
