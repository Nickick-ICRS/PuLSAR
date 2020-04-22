#ifndef __MCL_SINGLE_ROBOT_POSE_ESTIMATOR__
#define __MCL_SINGLE_ROBOT_POSE_ESTIMATOR__

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "pose_estimators/single_robot_pose_estimator.hpp"
#include "robot_models/robot_model.hpp"
#include "cloud_generator/cloud_generator.hpp"
#include "sensor_models/range_cloud_sensor_model.hpp"
#include "map_manager/map_manager.hpp"

/**
 * @brief Estimates the pose of a single robot.
 *
 * Class to estimate the pose of a single robot within a swarm of robots.
 * Uses laser scan and odometry information along with knowledge of the map
 * environment to estimate the pose of the robot.
 */
class MCLSingleRobotPoseEstimator :public SingleRobotPoseEstimator {
public:
    /**
     * Constructor for the MCLSingleRobotPoseEstimator class.
     *
     * @param name The name of the robot (as used by cloud_gen).
     *
     * @param cloud_gen Pointer to an initialised cloud generator class
     *                  instance.
     *
     * @param map_man Pointer to an initialised map manager class instance.
     *
     * @param initial_pose An initial estimate of the robot's pose relative
     *                     to the map frame.
     *
     * @param odom_topic The topic on which odometry measurements from the
     *                   robot are published.
     *
     * @param base_link_frame The name of the base_link frame of the robot.
     *
     * @param radius Approximate radius of the robot footprint.
     *
     * @param M Number of particles to include in the filter.
     *
     * @param min_trans_update Minimum distance to travel before running a
     *                         filter update.
     */
    MCLSingleRobotPoseEstimator(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame, 
        geometry_msgs::Pose initial_pose, std::string odom_topic,
        std::string base_link_frame, float radius, unsigned int M,
        double min_trans_update);
    ~MCLSingleRobotPoseEstimator();

    /**
     * @brief Updates the pose estimate of the robot.
     *
     * Updates the pose estimate of the robot by running the augmented MCL
     * particle filter and publishes to tf.
     */
    void update_estimate();

    /**
     * Get all of the current pose estimates. Useful for debugging.
     */
    std::vector<geometry_msgs::Pose>& get_pose_estimates() {
        return pose_estimate_cloud_;
    }
private:
    /**
     * @brief Runs an iteration of the augmented MCL algorithm.
     *
     * Runs one iteration of the augmented Monte Carlo Localisation
     * algorithm, finding a cloud of points all of which may be the pose
     * of this robot. Robust against local minima due to additions of
     * random additional poses. See Probabilistic Robotics by Thrun et al.
     *
     * @param Xt_1 The previous set of pose estimates.
     *
     * @param ut The most recent odometry measurements.
     *
     * @return The new (calculated) set of pose estimates.
     */
    std::vector<geometry_msgs::Pose> augmented_MCL(
        const std::vector<geometry_msgs::Pose>& Xt_1,
        const nav_msgs::Odometry& ut);

    /**
     * @brief Updates the overall pose estimate and covariance.
     *
     * Uses the current cloud of pose estimate points to calculate the
     * average pose estimate and covariance of the estimate.
     */
    void update_pose_estimate_with_covariance();

    std::vector<geometry_msgs::Pose> pose_estimate_cloud_;

    tf2_ros::Buffer tf2_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Exponential parameters for Augmented MCL, 0 <= aslow < afast
    static double aslow_, afast_;

    // Parameters for DBSCAN
    static int dbscan_min_points_;
    static double dbscan_epsilon_;

    // Saves us a lot of constructor parameters by having the above params
    // be static and making this class able to access them.
    friend class LocalisationNode;
};

#endif // __MCL_SINGLE_ROBOT_POSE_ESTIMATOR__
