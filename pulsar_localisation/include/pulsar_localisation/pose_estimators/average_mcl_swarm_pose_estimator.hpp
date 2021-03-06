#ifndef __AVERAGE_MCL_SWARM_POSE_ESTIMATOR_HPP__
#define __AVERAGE_MCL_SWARM_POSE_ESTIMATOR_HPP__

#include <map>
#include <memory>
#include <thread>

#include <geometry_msgs/Pose.h>

#include "pose_estimators/swarm_pose_estimator.hpp"
#include "pose_estimators/mcl_single_robot_pose_estimator.hpp"
#include "cloud_generator/cloud_generator.hpp"

/**
 * @brief Estimates the pose of the whole swarm.
 *
 * Class to estimate the pose of an entire swarm of robots. Estimates of the
 * pose of individual robots are combined with data points fromt the swarm
 * as a whole to estimate the pose of the whole swarm. From this, errors can
 * be found in pose estimates of swarm members and these may be iteratively
 * updated to improve pose estimates.
 */
template <class T>
class AverageMCLSwarmPoseEstimator :public SwarmPoseEstimator {
public:
    /**
     * Constructor for the AverageMCLSwarmPoseEstimator class.
     * 
     * @param cloud_gen Pointer to an initialised cloud generator class 
     *                  instance.
     *
     * @param map_man Pointer to an initialised map manager class instance.
     * 
     * @param robot_names Names of the different robots in the swarm.
     * 
     * @param initial_robot_poses Map of starting poses of the robots in 
     *                            the swarm keyed by name.
     *
     * @param robot_odom_topics Map of the topic names of robot odometry 
     *                          measurements keyed by name.
     * 
     * @param robot_base_links Map of the base_link frames of the robots,
     *                         keyed by name.
     *
     * @param robot_radii Map of the robot radii, keyed by name.
     *
     * @param map_frame The map frame of the whole swarm.
     *
     * @param M The number of particles to include in the single robot
     *          pose estimator particle filters.
     *
     * @param min_trans_update Minimum distance for robots to move before
     *                         performing a filter update.
     */
    AverageMCLSwarmPoseEstimator(
        const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame, 
        std::vector<std::string>& robot_names, 
        std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
        std::map<std::string, std::string>& robot_odom_topics,
        std::map<std::string, std::string>& robot_base_links,
        std::map<std::string, float>& robot_radii, unsigned int M,
        double min_trans_update, std::string robot_model)

        :SwarmPoseEstimator(
            cloud_gen, map_man, map_frame)
    {
        pose_pub_ =
            nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                "swarm_pose_estimate", 1);
        for(const std::string& name : robot_names) {
            robot_pose_estimators_[name].reset(
                new MCLSingleRobotPoseEstimator<T>(
                    name, cloud_gen, map_man, map_frame,
                    initial_robot_poses[name], robot_odom_topics[name],
                    robot_base_links[name], robot_radii[name], M,
                    min_trans_update)); 
            robot_pose_estimates_[name] = 
                robot_pose_estimators_[name]->get_pose_estimate();
            worker_threads_[name] = std::thread();
        }

        swarm_pose_estimate_.header.frame_id = map_frame;
    };
    ~AverageMCLSwarmPoseEstimator() {};

    /**
     * @brief Update robot pose estimates and swarm pose estimate.
     *
     * Update estimates of the pose of each robot within the swarm. Does
     * publish anything to TF. Also updates the pose estimate of
     * the entire swarm.
     */
    void update_estimate() {
        auto now = std::chrono::high_resolution_clock::now();
        // Run all the update functions in threads
        for(auto& pair : worker_threads_) {
            pair.second = std::thread(
                &MCLSingleRobotPoseEstimator<T>::update_estimate,
                robot_pose_estimators_[pair.first]);
        }
        for(auto& pair : worker_threads_) {
            pair.second.join();
            robot_pose_estimates_[pair.first] = 
                robot_pose_estimators_[pair.first]->get_pose_estimate();
        }
        update_estimate_covariance();
        std::chrono::duration<double, std::milli> dt = 
            std::chrono::high_resolution_clock::now() - now;
        ROS_INFO_STREAM("Update took " << dt.count() << " milliseconds.");
    };

private:
    std::map<std::string, std::shared_ptr<MCLSingleRobotPoseEstimator<T>>>
        robot_pose_estimators_;

    std::map<std::string, std::thread> worker_threads_;
};

#endif // __AVERAGE_MCL_SWARM_POSE_ESTIMATOR_HPP__
