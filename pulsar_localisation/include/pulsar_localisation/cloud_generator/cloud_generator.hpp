#ifndef __CLOUD_GENERATOR_HPP__
#define __CLOUD_GENERATOR_HPP__

#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * @brief Generates point clouds from range data.
 *
 * Class to process range data from multiple PuLSARs and store the data
 * in point clouds for processing. Data is stored for a certain amount of
 * time and the timestamps of the data are stored within the point clouds
 * as intensity values.
 */
class CloudGenerator {
public:
    /**
     * Constructor for the CloudGenerator class
     *
     * @param range_topic_names An array of all the topics from which range
     *                          data should be read and incorporated into
     *                          the whole point cloud.
     *
     * @param cycle_sensor_readings The most recent X measurements are
     *                              stored in the cloud.
     *
     * @param odom_name The name of the odom of the robots.
     *                       Defaults to "odom". Currently assumed that
     *                       all robots have the same odom name but
     *                       have separate tf_prefixes.
     */
    CloudGenerator(
        std::vector<std::string> range_topic_names, 
        float cycle_sensor_readings, std::string odom_name = "odom");
    ~CloudGenerator();

    /**
     * Publishes a requested point cloud so that it may be visualised on
     * e.g. RVIZ.
     *
     * @param name The name (tf_prefix) of the robot for which the cloud
     *             should be published.
     */
    void publish_cloud(std::string name);

    /**
     * @brief Updates all point clouds by removing old data.
     *
     * Updates all point clouds by removing any points with intensity values
     * lower than the current time - cycle_sensor_readings_.
     */
    void clean_all_clouds();

    /**
     * @brief Updates a single point cloud by removing old data.
     *
     * Updates a single point cloud by removing any points with intensity
     * values lower than the current time - cycle_sensor_readings_.
     *
     * @param name The name (tf_prefix of the robot) of the point cloud.
     */
    void clean_cloud(std::string name);

    /**
     * Get the raw data from the cloud.
     *
     * @param robot_name Which robot's data to get.
     *
     * @return The data.
     */
    const std::vector<sensor_msgs::Range>& get_raw_data(
        std::string robot_name);
private:
    /**
     * @brief Updates a single raw data cloud by removing old data.
     *
     * Updates a single raw data cloud by removing any data gathered earlier
     * than the current time - cycle_sensor_readings_.
     *
     * @param cloud The raw data vector to be updated.
     */
    void clean_cloud(std::vector<sensor_msgs::Range>& cloud);

    /**
     * Callback for range finder data from one or more PuLSAR robots.
     *
     * @param msg The message received to be processed in the callback.
     */
    void range_cb(const sensor_msgs::RangeConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::vector<ros::Subscriber> range_subs_;
    std::map<std::string, ros::Publisher> cloud_pubs_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_;

    std::map<std::string, std::vector<sensor_msgs::Range>> robot_raw_data_;
    /**
     * std::recursive_mutex is used here as there may be some functions
     * which lock and then call another function which also locks. This
     * *shouldn't* happen, but a simple programmer mistake could cause
     * some deadlocks so best to avoid it.
     */
    std::map<std::string, std::recursive_mutex> robot_cloud_muts_;

    /**
     * Name of the odom frame for each robot - assumed that each robot
     * is separated by a tf_prefix
     */
    std::string odom_name_;

    /**
     * Only this many readings are stored between updates.
     */
    int cycle_sensor_readings_;
};

#endif // __CLOUD_GENERATOR_HPP__
