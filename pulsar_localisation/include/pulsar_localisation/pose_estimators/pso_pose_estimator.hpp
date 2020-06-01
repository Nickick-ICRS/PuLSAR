#ifndef __PSO_POSE_ESTIMATOR_HPP__
#define __PSO_POSE_ESTIMATOR_HPP__

#include <map>
#include <vector>
#include <memory>
#include <random>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

#include "pose_estimators/swarm_pose_estimator.hpp"
#include "robot_models/base_robot_model.hpp"
#include "map_manager/map_manager.hpp"
#include "cloud_generator/cloud_generator.hpp"

struct PSOPoseParticleCloud {
public:
    std::vector<geometry_msgs::Pose> particles;
    std::vector<geometry_msgs::Vector3> velocities;
    std::vector<geometry_msgs::Pose> particle_bests;
    geometry_msgs::Pose best_pose;
    geometry_msgs::Pose best_frame_pose;
};

/**
 * @brief Estimates the pose of the whole swarm using PSO.
 *
 * Attempts to estimate the pose of a swarm of robots by using a variant
 * of the particle swarm optimisation algorithm.
 */
class PSOPoseEstimator :public SwarmPoseEstimator {
public:
    /**
     * Constructor for the PSOPoseEstimator class.
     *
     * @param cloud_gen Pointer to an initilised cloud generator class 
     *                  instance.
     *
     * @param map_man Pointer to an initialised map_manager class instance.
     *
     * @param map_frame The frame to localise the swarm relative to.
     *
     * @param robot_names Names of the different robots in the swarm.
     *
     * @param initial_robot_poses Map of the starting poses of the robots
     *                            in the swarm keyed by name.
     *
     * @param robot_odom_topics Map of the topic names of robot odometry
     *                          measurements keyed by name.
     *
     * @param robot_base_links Map of the robot base_link frames, keyed by
     *                         name.
     *
     * @param robot_radii Approximate radius of the robots.
     *
     * @param robot_model Which robot model to use  for calculating the
     *                    pose fitness.
     *
     * @param use_initial_poses Whether to initalise the swarm poses with
     *                          the given initial poses or with a random
     *                          uniform pose distribution.
     */
    PSOPoseEstimator(
        const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::vector<std::string>& robot_names,
        std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
        std::map<std::string, std::string>& robot_odom_topics,
        std::map<std::string, std::string>& robot_base_links,
        std::map<std::string, float>& robot_radii,
        unsigned int swarm_particles, unsigned int robot_particles,
        std::string robot_model, bool use_initial_poses = true);
    virtual ~PSOPoseEstimator();

    /**
     * @brief Update swarm pose estimate.
     */
    void update_estimate();

private:
    /**
     * @brief Update a particle in the PSO algorithm.
     *
     * Updates the velocity and position of a particle used within the PSO
     * algorithm. This should then be weighed and compared to the best
     * results found so far.
     *
     * @param x The current pose of the particle, which will be updated.
     *
     * @param v The current velocity of the particle, which will be updated.
     *
     * @param p The best known pose of the particle so far.
     *
     * @param g The best known pose of all estimates so far.
     */
    void calc_new_particle_position(
        geometry_msgs::Pose& x, geometry_msgs::Vector3& v,
        const geometry_msgs::Pose& p, const geometry_msgs::Pose& g);

    /**
     * Updates the pose estimates of a single robot.
     *
     * @param robot_name The name of the robot.
     *
     * @return Whether the pose estimate has improved.
     */
    bool update_robot_particles(std::string robot_name);

    void update_robot_pose_estimates();

    const std::shared_ptr<MapManager>& map_man_;
    
    std::map<std::string, std::unique_ptr<BaseRobotModel>> robot_models_;
    // Map of particle clouds containing particles and their corresponding
    // velocities
    std::map<std::string, PSOPoseParticleCloud> robot_particle_clouds_;
    PSOPoseParticleCloud swarm_particle_cloud_;

    std::map<std::string, float> radii_;

    std::random_device rd_;
    std::mt19937_64 gen_;
    std::uniform_real_distribution<double> dist_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    static int max_its_;
    static double omega_;
    static double phi_p_;
    static double phi_g_;
    static double phi_r_;

    friend class LocalisationNode;
};

#endif // __PSO_POSE_ESTIMATOR_HPP__
