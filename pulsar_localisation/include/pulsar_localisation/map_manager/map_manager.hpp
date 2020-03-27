#ifndef __MAP_MANAGER_HPP__
#define __MAP_MANAGER_HPP__

#include <thread>
#include <shared_mutex>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

/**
 * @brief Keeps track of the world map occupancy grid.
 *
 * After a map has been registered, this class keeps track of it. It also
 * stores the poses of the swarm robots within the map, and can check to
 * see whether robots are colliding, or if poses are valid etc.
 */
class MapManager {
public:
    /**
     * Constructor for the MapManager class. Blocks until the first map
     * message has been received.
     *
     * @param map_topic The topic on which to listen for map data.
     */
    MapManager(std::string map_topic);
    ~MapManager();

    /**
     * @brief Checks whether a pose in the map is valid.
     *
     * Checks whether a pose in the map is valid. A pose is valid if it is
     * not outside of the walls of the map, and is not inside a wall. Does
     * not consider the positions of other robots.
     *
     * @param pose The pose to be checked.
     *
     * @return Whether the pose is valid (true) or not (false).
     */
    bool is_pose_valid(const geometry_msgs::Pose& pose);

    /**
     * @brief Checks whether a pose in the map is valid.
     *
     * Checks whether a pose in the map is valid. Takes into account the
     * poses of other robots within the map.
     *
     * @param pose The pose to be checked.
     *
     * @param robot_name The robot this pose is for. Leave blank if no
     *                   robots are to be checked. This robot will not be
     *                   considered when determining whether a pose is
     *                   valid or not.
     *
     * @return Whether the pose is valid (true) or not (false).
     */
    bool is_pose_valid(
        const geometry_msgs::Pose& pose, std::string robot_name);

    /**
     * Updates the position of a robot within the map.
     *
     * @param robot_name The name of the robot who's pose is being updated.
     *
     * @param pose The new pose of the robot.
     */
    void set_robot_pose(
        std::string robot_name, const geometry_msgs::Pose& pose);
private:
    void map_cb(const nav_msgs::OccupancyGridConstPtr& msg);

    nav_msgs::OccupancyGridConstPtr map_;

    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;

    /* We can have multiple threads reading the map, or we can have ONE
     * thread writing to it. */
    std::shared_mutex map_mutex_;
};

#endif // __MAP_MANAGER_HPP__
