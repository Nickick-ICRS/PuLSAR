#ifndef __CLOUD_GENERATOR_HPP__
#define __CLOUD_GENERATOR_HPP__

/// Generates point clouds from range data.
/**
 * Class to process range data from multiple PuLSARs and store the data
 * in point clouds for processing. Data is stored for a certain amount of
 * time and the timestamps of the data are stored within the point clouds
 * as intensity values.
 */

#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class CloudGenerator {
public:
    /**
     * Constructor for the CloudGenerator class
     *
     * @param range_topic_names An array of all the topics from which range
     *                          data should be read and incorporated into
     *                          the whole point cloud.
     *
     * @param data_time_range Data older than this number of seconds will be
     *                        discarded and removed from the cloud.
     *
     * @param odom_name The name of the odom of the robots.
     *                       Defaults to "odom". Currently assumed that
     *                       all robots have the same odom name but
     *                       have separate tf_prefixes.
     */
    CloudGenerator(
        std::vector<std::string> range_topic_names, float data_time_range,
        std::string odom_name = "odom");
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
     * Publishes the full point cloud including data from each robot
     */
    void publish_full_cloud();
private:
    /**
     * Callback for range finder data from one or more PuLSAR robots.
     *
     * @param msg The message received to be processed in the callback.
     */
    void range_cb(const sensor_msgs::RangeConstPtr& msg);

    /**
     * Converts a pcl point cloud to a ROS point cloud for publishing.
     *
     * @param cloud Reference to the cloud to be converted.
     *
     * @param frame_name The tf frame the cloud is in, so that the ROS
     *                   message can be filled out properly.
     *
     * @return The ROS PointCloud2 for publishing.
     */
    sensor_msgs::PointCloud2 convert_cloud(
        const pcl::PointCloud<pcl::PointXYZI>& cloud,
        std::string frame_name);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::vector<ros::Subscriber> range_subs_;
    std::map<std::string, ros::Publisher> cloud_pubs_;
    ros::Publisher full_cloud_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_;

    /**
     * We use a PointXYZI so that we can store data timestamps in the
     * intensity field - essentially, the newer the data, the more we care
     * about it.
     */
    pcl::PointCloud<pcl::PointXYZI> full_cloud_;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZI>> robot_clouds_;

    /**
     * Name of the odom frame for each robot - assumed that each robot
     * is separated by a tf_prefix
     */
    std::string odom_name_;
};

#endif // __CLOUD_GENERATOR_HPP__
