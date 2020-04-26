#ifndef __BASE_ROBOT_MODEL_HPP__
#define __BASE_ROBOT_MODEL_HPP__

#include <random>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include "cloud_generator/cloud_generator.hpp"
#include "map_manager/map_manager.hpp"
#include "sensor_models/range_cloud_sensor_model.hpp"

/**
 * Abstract class to represent a robot model, including sensor and motion
 * models.
 */
class BaseRobotModel {
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
    BaseRobotModel(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::string odom_topic, std::string base_link_frame, float radius,
        double min_trans_update);
    virtual ~BaseRobotModel();

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
     * @brief Sample a motion model to predict the robot's position.
     *
     * Samples from a motion model (e.g. pure odometry, odometry + sensor)
     * to generate a prediction of a robot pose, given the previous pose
     * estimate and current sensor information collected.
     *
     * @param xt_1 The estimated previous robot position.
     *
     * @return The estimated new robot position.
     */
    virtual geometry_msgs::Pose sample_motion_model(
        const geometry_msgs::Pose& xt_1) = 0;

    /**
     * @brief Call this after each full pose estimate iteration.
     *
     * Updates the odom useage. Call this after and iteration has been
     * completed successfully. Failure to call will result in increasingly
     * large jumps in the odometry sampler!
     */
    void update_odom();

    /**
     * Weigh the validity of an estimated pose based upon sensor models.
     *
     * @param p The pose to be weighed.
     *
     * @return The weight.
     */
    virtual double weigh_pose(const geometry_msgs::Pose& p) = 0;

    /**
     * Calculate the transform from the map frame to the odometry frame.
     *
     * @param p The pose relative to the map frame that the robot is at.
     *
     * @return The transform to be published to TF.
     */
    geometry_msgs::TransformStamped calculate_transform(
        const geometry_msgs::Pose& p);

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

    double min_trans_update_;

private: 
    /**
     * Odometry callback from the robot.
     *
     * @param msg The odometry message.
     */
    void odom_cb(const nav_msgs::OdometryConstPtr& msg);

    bool first_odom_cb_;
    ros::Subscriber odom_sub_;
};

#endif // __BASE_ROBOT_MODEL_HPP__
