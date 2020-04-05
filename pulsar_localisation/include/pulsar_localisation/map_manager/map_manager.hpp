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
     * @param safety_radius An additional area around the pose to be
     *                      checked. 0 means no additional area.
     *
     * @return Whether the pose is valid (true) or not (false).
     */
    bool is_pose_valid(
        const geometry_msgs::Pose& pose, float safety_radius = 0);

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
     * @param safety_radius An additional area around the pose to be
     *                      checked. 0 means no additional area.
     *
     * @return Whether the pose is valid (true) or not (false).
     */
    bool is_pose_valid(
        const geometry_msgs::Pose& pose, std::string robot_name,
        float safety_radius = 0);

    /**
     * Updates the position of a robot within the map.
     *
     * @param robot_name The name of the robot who's pose is being updated.
     *
     * @param pose The new pose of the robot.
     *
     * @param robot_radius The approximate radius of the robot for collision
     *                     checking.
     */
    void set_robot_pose(
        std::string robot_name, const geometry_msgs::Pose& pose,
        float robot_radius);

    /**
     * Cone cast from a frame without considering other robots.
     *
     * @param p Point from which to cone cast from, in the map frame.
     *
     * @param ang Angle of the cone. 0 is positive x, following REP 103.
     *
     * @param spread The cone angle.
     *
     * @return The distance from the frame to the first obstacle.
     */
    double cone_cast_plain_map(
        const geometry_msgs::Point& p, double ang, double spread);

    /**
     * Cone cast from a frame with consideration to other robots.
     *
     * @param p Point from which to cone cast from, in the map frame.
     *
     * @param ang Angle of the cone. 0 is positive x, following REP 103.
     *
     * @param spread The cone angle.
     *
     * @param robot_name Name of the robot which is measuring. This will be
     *                   ignored when considering cone collisions.
     *
     * @return The distance from the frame to the first obstacle.
     */
    double cone_cast_with_bots(
        const geometry_msgs::Point& p, double ang, double spread,
        std::string robot_name);
private:
    /**
     * Callback to receive the map stored on the map server.
     *
     * @param msg The message containing the map.
     */
    void map_cb(const nav_msgs::OccupancyGridPtr& msg);

    /**
     * Cast a ray defined by a point and angle.
     *
     * @param p The point from which to start the ray.
     *
     * @param ang The angle of the ray.
     *
     * @param map The map within which to cast.
     *
     * @return Distance to the first object hit by the ray.
     */
    double ray_cast(
        const geometry_msgs::Point& p, double ang, const int8_t *map);

    /**
     * @brief Perform a cone casting operation.
     *
     * Perform a cone casting operation. This probably doesn't work properly
     * if the spread angle is greater than 90 degrees, but that would be a
     * pretty unuseable range sensor so...
     *
     * @param p The point from which to start the cast.
     *
     * @param ang The angle at which direction to cast in.
     *
     * @param spread The coning angle.
     *
     * @param map The map.
     *
     * @return The distance to the first occupied grid tile found by the
     * cone cast (m).
     */
    double cone_cast(
        const geometry_msgs::Point& p, double ang, double spread_ang, 
        const int8_t *map);

    /**
     * Gets the tiles occupied by a point and radius on the map.
     *
     * @param p The point to search around.
     *
     * @param radius The radius around which we care.
     *
     * @return List of the tiles used by the robot within the map.
     */
    std::vector<unsigned int> get_tiles(
        const geometry_msgs::Point& p, float radius);

    /**
     * @brief Update the map with robot poses.
     *
     * Function to update the second map with the estimated positions of the
     * robots of the swarm. This allows us to check range readings with
     * robots within the map (i.e. if the robots are in the way of the scans
     * we know and can reduce false readings).
     */
    void update_map_with_robots();

    /**
     * Function to update the second map with the estimated position of a
     * single robot.
     *
     * @param robot_name The robot whose pose is being updated.
     */
    void update_map_with_robot(std::string robot_name);

    nav_msgs::OccupancyGridPtr map_;
    int8_t* map_with_robots_;

    std::map<std::string, geometry_msgs::Pose> robot_poses_;
    std::map<std::string, geometry_msgs::Pose> old_robot_poses_;
    std::map<std::string, float> robot_radii_;

    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;

    /* We can have multiple threads reading the map, or we can have ONE
     * thread writing to it. */
    std::shared_mutex map_mutex_;
};

#endif // __MAP_MANAGER_HPP__
