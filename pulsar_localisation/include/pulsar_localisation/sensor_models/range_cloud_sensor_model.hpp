#ifndef __RANGE_CLOUD_SENSOR_MODEL_HPP__
#define __RANGE_CLOUD_SENSOR_MODEL_HPP__

#include <cmath>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_type.h>

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
     */
    RangeCloudSensorModel(
        std::shared_ptr<MapManager> map_man, float history_length,
        float time_resolution, std::string map_frame);
    ~RangeCloudSensorModel();

    /**
     * @brief Add a pose estimate to the model history.
     *
     * Adds a pose estimate to the history of the model, allowing it to
     * account for robot movement over the course of measurements.
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
     * @param c The measurement data in point cloud form. This should be
     *          stored with respect to the base_link frame of the robot.
     *          The intensity (I) of the data should be the time at which it
     *          was acquired (in seconds).
     *
     * @param frame The coordinate frame of the point cloud data.
     *
     * @return A probability of the pose being correct.
     */
    float model(
        const geometry_msgs::Pose& p, 
        const pcl::PointCloud<pcl::PointXYZI>& c, std::string frame);
private:
    /**
     * Calculates the probability that a measurement was correct.
     *
     * @param z The measurement.
     */
    float phit(const geometry_msgs::PointStamped& z);

    /**
     * Calculates the probability that a measurement fell short.
     *
     * @param z The measurement.
     */
    float pshort(const geometry_msgs::PointStamped& z);

    /**
     * Calculates the probability that a measurement was max range.
     *
     * @param z The measurement.
     */
    float pmax(const geometry_msgs::PointStamped& z);

    /**
     * Calculates the probability that a measurement was random noise.
     *
     * @param z The measurement.
     */
    float prand(const geometry_msgs::PointStamped& z);

    // Map
    std::shared_ptr<MapManager> map_man_;

    std::vector<geometry_msgs::PoseWithCovarianceStamped> pose_estimates_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_;

    float history_length_;
    float time_resolution_;
    std::string map_frame_;
};

#endif // __RANGE_CLOUD_SENSOR_MODEL_HPP__
