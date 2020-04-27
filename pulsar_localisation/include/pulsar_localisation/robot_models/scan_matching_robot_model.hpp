#ifndef __SCAN_MATCHING_ROBOT_MODEL_HPP__
#define __SCAN_MATCHING_ROBOT_MODEL_HPP__

#include <vector>
#include <string>
#include <memory>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_types.h>

#include "robot_models/odometry_robot_model.hpp"
#include "map_manager/map_manager.hpp"
#include "cloud_generator/cloud_generator.hpp"

/**
 * Directions which may be checked when laser scanning.
 */
enum class Dir {
    PX, PY, PTH,
    NX, NY, NTH,
};

/**
 * Class to model robot movement via odometry and correlate with range
 * measurements, to ensure every modelled pose is valid within the
 * environment.
 */
class ScanMatchingRobotModel : public OdometryRobotModel {
public:
    /**
     * Constructor for the ScanMatchingRobotModel class.
     *
     * @param name The name of the robot (as used by cloud_gen etc.).
     *
     * @param cloud_gen Pointer to an initialised cloud_generator class.
     *
     * @param map_man Pointer to an initialised map_manager class.
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
    ScanMatchingRobotModel(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::string odom_topic, std::string base_link_frame, float radius,
        double min_trans_update);
    virtual ~ScanMatchingRobotModel();

     /**
     * @brief Samples from p(xt | ut, xt-1).
     *
     * Sample from p(xt | ut, xt-1) given odometry information. See
     * Probabilistic Robotics by Thrun et al. Uses the odometry stored in
     * this class.
     *
     * @param xt_1 The most recent pose estimate.
     *
     * @return The current (new) pose estimate.
     */
    virtual geometry_msgs::Pose sample_motion_model(
        const geometry_msgs::Pose& xt_1);

    /**
     * Weighs a pose based on how likely it is that it is correct.
     *
     * @return A weight based on how likely the point is correct. Large 
     *         is good.
     */
    virtual double weigh_pose(const geometry_msgs::Pose& p);

private:
    /**
     * @brief Adjusts a pose to fit the map.
     * 
     * Takes a pose and attempts to find the closest fitting pose in the
     * map using the most recent range finder data.
     *
     * @param xt The current pose estimate to be adjusted.
     *
     * @param xt_1 The previous pose estimate.
     *
     * @return The adjusted pose.
     */
    geometry_msgs::Pose scan_match_l2(
        const geometry_msgs::Pose& xt, const geometry_msgs::Pose& xt_1);

    /**
     * Move the scan measurement in one direction and find the optimal
     * placement according to the map.
     *
     * @param p The starting pose to check from.
     *
     * @param Z The measurement data to use for comparison.
     *
     * @param dir The direction to move in (PX, NX, PY, or NY).
     *
     * @return The distance to move in the requested direction, and the l2
     *         error between the resulting pose and the map.
     */
    std::pair<double, double> move_in_dir(
        const geometry_msgs::Pose& p,
        const std::vector<geometry_msgs::Point>& Z, Dir dir);

    /**
     * Rotate the scan measurement in one direction and find the optimal
     * placement and point to rotate about according to the map.
     *
     * @param p The starting pose to check from.
     *
     * @param Z The measurement data to use for comparison.
     *
     * @param dir The direction to move in (PTH or NTH).
     *
     * @return The distance to rotate in the requested direction, the l2 
     * error between the resulting pose and the map, and the pivot point.
     */
    std::tuple<double, double, geometry_msgs::Point> rotate_in_dir(
        const geometry_msgs::Pose& p,
        const std::vector<geometry_msgs::Point>& Z, Dir dir);

    /**
     * Gets the sensor data transformed into the robot coordinate frame.
     *
     * @param p The pose we want the data relative to.
     *
     * @return The sensor data in the robot coordinate frame.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_sensor_data(
        const geometry_msgs::Pose& p);

    tf2_ros::Buffer tf2_;
    tf2_ros::TransformListener tf_listener_;

    // Parameter for adjusting how much continuity is worth when weighing
    // a scan-matched pose.
    static double lamcont_;

    friend class LocalisationNode;
};

#endif // __SCAN_MATCHING_ROBOT_MODEL_HPP__
