#ifndef __RANGE_CLOUD_SENSOR_MODEL_HPP__
#define __RANGE_CLOUD_SENSOR_MODEL_HPP__

#include <cmath>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Range.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "map_manager/map_manager.hpp"

/**
 * @brief A model of a cloud of points from range finder sensors.
 *
 * Class to model measurements from range sensors, stored as a cloud of
 * points over time. Considers drift of robot from measurements and relies
 * more heavily on newer measurements than older measurements.
 */
class RangeCloudSensorModel {
public:
    /**
     * Constructor for the RangeCloudSensorModel class.
     *
     * @param map_frame The map frame of the robot.
     *
     * @param map_man An initialised instance of the map manager class.
     */
    RangeCloudSensorModel(
        std::string map_frame, const std::shared_ptr<MapManager>& map_man);
    ~RangeCloudSensorModel();

    /**
     * @brief Add a pose estimate to the model history.
     *
     * Adds a pose estimate to the history of the model, allowing it to
     * account for robot movement over the course of measurements.
     *
     * N.B. Currently unused.
     *
     * @param pose The pose estimate.
     */
    void add_pose_estimate(
        const geometry_msgs::PoseWithCovarianceStamped& p);

    /**
     * @brief Weighs how likely a pose estimate is to being true.
     *
     * Considering the current measurement data and knowledge of
     * surroundings via the map, considers how likely a given pose is to
     * be accurate, and weighs it as such.
     *
     * @param p The pose being weighed. This should be relative to the map
     *          frame.
     *
     * @param c The cloud of raw measurement data.
     *
     * @param robot_name The name (tf_prefix) of the robot.
     *
     * @param base_link_frame The frame name of the robot's base_link.
     *
     * @return A probability of the pose being correct.
     */
    float model(
        const geometry_msgs::Pose& p, 
        const std::vector<sensor_msgs::Range>& c, std::string robot_name, 
        std::string base_link_frame);
private:
    /**
     * Calculates the probability that a measurement was correct. See
     * Probabilistic Robotics by Thrun et al.
     *
     * @param z The measurement.
     */
    float phit(const sensor_msgs::Range& z, double ideal_z);

    /**
     * Calculates the probability that a measurement fell short. See
     * Probabilistic Robotics by Thrun et al.
     *
     * @param z The measurement.
     */
    float pshort(const sensor_msgs::Range& z, double ideal_z);

    /**
     * Calculates the probability that a measurement was max range. See
     * Probabilistic Robotics by Thrun et al.
     *
     * @param z The measurement.
     */
    float pmax(const sensor_msgs::Range& z);

    /**
     * Calculates the probability that a measurement was random noise. See
     * Probabilistic Robotics by Thrun et al.
     *
     * @param z The measurement.
     */
    float prand(const sensor_msgs::Range& z);

    std::vector<geometry_msgs::PoseWithCovarianceStamped> pose_estimates_;

    std::shared_ptr<MapManager> map_man_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_;

    std::string map_frame_;

    static double lamshort_;
    static double sigmahit_;
    static double ztime_;
    static double zhit_;
    static double zshort_;
    static double zmax_;
    static double zrand_;

    static float history_length_;
    static float time_resolution_;

    friend class LocalisationNode;
};

#endif // __RANGE_CLOUD_SENSOR_MODEL_HPP__
