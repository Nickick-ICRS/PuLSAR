#include "map_manager/map_manager.hpp"

#include <chrono> // For std::chrono::seconds
#include <mutex> // For std::unique_lock

MapManager::MapManager(std::string map_topic) {
    map_sub_ = nh_.subscribe(map_topic, 1, &MapManager::map_cb, this);

    // Block until the map comes online
    while(not map_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ROS_WARN_STREAM_DELAYED_THROTTLE(
            5, "No map received from '" << map_topic 
            << "' yet. Will continue waiting...");
    }
}

MapManager::~MapManager() {
    // dtor
}

void MapManager::map_cb(const nav_msgs::OccupancyGridConstPtr& msg) {
    // This lock lets us have multiple reads *or* one write at a time
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    map_ = msg;
}

bool MapManager::is_pose_valid(const geometry_msgs::Pose& pose) {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);

    // First find the point in the occupancy grid that the pose is on
    // Map origin is the bottom left
    auto th = 2*acos(map_->info.origin.orientation.w);
    double dx = pose.position.x - map_->info.origin.position.x;
    double dy = pose.position.y - map_->info.origin.position.y;

    int row = (cos(th) * dx - sin(th) * dy) * map_->info.resolution;
    int col = (-sin(th) * dx - cos(th) * dy) * map_->info.resolution;

    // Check that the pose is within the map boundaries
    if(row < 0) return false;
    if(col < 0) return false;
    if(row >= map_->info.height) return false;
    if(col >= map_->info.width) return false;

    // Check if the pose is already occupied in the map (i.e. by a wall)
    if(map_->data[col*map_->info.height + row] != 0) return false;

    return true;
}

bool MapManager::is_pose_valid(
    const geometry_msgs::Pose& pose, std::string robot_name)
{
    return true;
}

void MapManager::set_robot_pose(
    std::string robot_name, const geometry_msgs::Pose& pose)
{

}
