#ifndef __ROBOT_MODEL_HPP__
#define __ROBOT_MODEL_HPP__

#include <random>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "cloud_generator/cloud_generator.hpp"
#include "map_manager/map_manager.hpp"
#include "sensor_models/range_cloud_sensor_model.hpp"

/**
 * @brief Contains the sensor and motion models for a single robot.
 */
class RobotModel {
public:
    /**
     * Constructor for the RobotModel class.
     *
     * @param name The name of the robot (as used by cloud_gen etc.).
     *
     * @param cloud_gen Pointer to an initialised cloud_generator class.
     *
     * @param map_man Pointer to an initialised map manager class instance.
     *
     * @param odom_topic The topic on which odometry measurements from the
     *                   robot are subscribed to.
     *
     * @param base_link_frame The name of the base_link frame of the robot.
     *
     * @param radius Approximate radius of the robot footprint.
     *
     * @param min_trans_update Minimum movement for translation update.
     */
    RobotModel(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::string odom_topic, std::string base_link_frame, float radius,
        double min_trans_update);
    virtual ~RobotModel();

    /**
     * @brief Generates a random valid pose for the robot.
     *
     * Uses knowledge of the map to generate a random valid (i.e. not in a
     * wall) pose that the robot may be in. Does not take into consideration
     * the poses of other robots in the swarm.
     *
     * @return A valid pose for the robot.
     */
    geometry_msgs::Pose gen_random_valid_pose();

    /**
     * @brief Generates a random valid pose for the robot about a yaw.
     *
     * Uses knowledge of the map to generate a random valid (i.e. not in a
     * wall) pose that the robot may be in. Does not take into consideration
     * the poses of other robots in the swarm.
     *
     * @param yaw The required yaw of the pose.
     *
     * @return A valid pose for the robot.
     */
    geometry_msgs::Pose gen_random_valid_pose_position(double yaw);

    /**
     * @brief Generates a random valid pose for the robot.
     *
     * Uses knowledge of the map to generate a random valid (i.e. not in a
     * wall) pose that the robot may be in. Does not take into consideration
     * the poses of other robots in the swarm.
     *
     * @param p A pose with covariance about which to generate the point.
     *
     * @return A valid pose for the robot.
     */
    geometry_msgs::Pose gen_random_valid_pose(
        const geometry_msgs::PoseWithCovariance& p);

    /**
     * @brief Samples from p(xt | ut, xt-1).
     *
     * Sample from p(xt | ut, xt-1) given odometry information. See 
     * Probabilistic Robotics by Thrun et al.
     *
     * @param ut The most recent odometry measurement.
     *
     * @param ut_1 The previously used odometry measurement.
     *
     * @param xt_1 The most recent pose estimate.
     *
     * @return The current (new) pose estimate.
     */
    geometry_msgs::Pose sample_motion_model_odometry(
        const geometry_msgs::Pose& ut, const geometry_msgs::Pose& ut_1,
        const geometry_msgs::Pose& xt_1);

protected:
    std::string name_;
    std::string base_link_frame_;
    std::string map_frame_;
    float radius_;

    nav_msgs::Odometry recent_odom_;
    nav_msgs::Odometry prev_odom_;

    std::mutex odom_mut_;

    ros::NodeHandle nh_;

    std::random_device rd_;
    std::mt19937_64 gen_;

    const std::shared_ptr<CloudGenerator> cloud_gen_;
    const std::shared_ptr<MapManager> map_man_;
    RangeCloudSensorModel range_model_;

private: 
    /**
     * Odometry callback from the robot.
     *
     * @param msg The odometry message.
     */
    void odom_cb(const nav_msgs::OdometryConstPtr& msg);

    bool first_odom_cb_;
    ros::Subscriber odom_sub_;

    double min_trans_update_;

    // Robot noise parameters
    static double a1_, a2_, a3_, a4_;

    // Saves us a lot of constructor parameters by having the above params
    // be static and making this class able to access them.
    friend class LocalisationNode;
};

#endif // __ROBOT_MODEL_HPP__
