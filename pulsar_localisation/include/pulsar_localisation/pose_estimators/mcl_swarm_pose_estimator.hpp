#ifndef __MCL_SWARM_POSE_ESTIMATOR_HPP__
#define __MCL_SWARM_POSE_ESTIMATOR_HPP__

#include <map>
#include <vector>
#include <memory>
#include <random>

#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>

#include "pose_estimators/swarm_pose_estimator.hpp"
#include "robot_models/base_robot_model.hpp"
#include "map_manager/map_manager.hpp"
#include "cloud_generator/cloud_generator.hpp"

/**
 * @brief Estimates the pose of the whole swarm.
 *
 * Class to estimate the pose of an entire swarm of robots. Runs a modified
 * MCL filter, where pose estimates are tested among all robots in order
 * to attempt to reduce the total number of points required in the filter.
 */
class MCLSwarmPoseEstimator :public SwarmPoseEstimator {
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
     *
     * @param min_trans_update Minimum robot movement to consider when
     *                         calculating the odometry motion model.
     *
     * @param robot_model Which robot model to use.
     */
    MCLSwarmPoseEstimator(
        const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::vector<std::string>& robot_names, 
        std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
        std::map<std::string, std::string>& robot_odom_topics,
        std::map<std::string, std::string>& robot_base_links,
        std::map<std::string, float>& robot_radii, unsigned int M,
        double min_trans_update, std::string robot_model);
    ~MCLSwarmPoseEstimator();

    /**
     * @brief Update swarm pose estimate.
     *
     * Update estimate of the swarm and robots within the swarm. Publishes
     * the estimate to TF.
     */
    void update_estimate();
private:
    /**
     * Updates the pose cloud based on odometry estimates from each robot.
     */
    void update_odometry_estimates();

    /**
     * Assigns pose estimates to robots based on proximity.
     */
    void assign_robot_pose_estimates();

    /**
     * Weighs points and selects points to be kept based on their weight.
     */
    void weigh_and_keep_points();

    /**
     * Updates individual robot pose estimates, calculates their transforms
     * and publishes to TF.
     */
    void update_robot_pose_estimates();
    
    /**
     * Gets the closest robots to a requested point.
     *
     * @param p The pose to compare robot locations to.
     *
     * @return The closest X robots to the requested point.
     */
    std::vector<std::string> get_closest_robots(
        const geometry_msgs::Pose& p);
    
    /**
     * Matches each robot to a cluster ID after pose clustering.
     *
     * @param point_IDs Vector containing the cluster ID of each point in 
     * the pose_cloud_ vector.
     *
     * @param means Map of the means of each cluster.
     *
     * @return Map between robot names and cluster IDs.
     */
    std::map<std::string, unsigned int>
        match_robots_to_clusters(
            const std::vector<unsigned int>& clustered_points,
            const std::map<unsigned int, geometry_msgs::Pose>& means);

    const std::shared_ptr<MapManager>& map_man_;

    std::vector<geometry_msgs::Pose> pose_cloud_;
    std::vector<std::pair<std::string, geometry_msgs::Pose>> 
        unbounded_pose_cloud_;
    std::map<std::string, std::unique_ptr<BaseRobotModel>> robot_models_;
    std::map<std::string, std::vector<geometry_msgs::Pose>>
        robot_pose_clouds_;
    std::map<std::string, float> radii_;

    std::random_device rd_;
    std::mt19937_64 gen_;
    std::uniform_real_distribution<double> dist_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    unsigned int M_;

    // How many robots should we consider when finding the closest robots to
    // a given pose?
    int num_close_robots_;
    // Minimum number of points to be considered per robot
    double min_robot_cloud_size_;

    friend class LocalisationNode;
};

#endif // __MCL_SWARM_POSE_ESTIMATOR_HPP__
