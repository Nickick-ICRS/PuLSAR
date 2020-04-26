#ifndef __ODOMETRY_ROBOT_MODEL_HPP__
#define __ODOMETRY_ROBOT_MODEL_HPP__

#include "robot_models/base_robot_model.hpp"
#include "cloud_generator/cloud_generator.hpp"
#include "map_manager/map_manager.hpp"
#include "sensor_models/range_cloud_sensor_model.hpp"

/**
 * @brief Contains the sensor and motion models for a single robot.
 */
class OdometryRobotModel : public BaseRobotModel {
public:
    /**
     * Constructor for the OdometryRobotModel class.
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
    OdometryRobotModel(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame,
        std::string odom_topic, std::string base_link_frame, float radius,
        double min_trans_update);
    virtual ~OdometryRobotModel();

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
     * Weigh the validity of an estimated pose based upon sensor models.
     *
     * @param p The pose to be weighed.
     *
     * @return The weight.
     */
    double weigh_pose(const geometry_msgs::Pose& p);
protected:
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

private:
    // Robot noise parameters
    static double a1_, a2_, a3_, a4_;

    // Saves us a lot of constructor parameters by having the above params
    // be static and making this class able to access them.
    friend class LocalisationNode;
};

#endif // __ODOMETRY_ROBOT_MODEL_HPP__
