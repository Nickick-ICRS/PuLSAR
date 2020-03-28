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
    if(map_with_robots_) {
        std::unique_lock<std::shared_mutex> lock(map_mutex_);
        delete map_with_robots_;
    }
}

void MapManager::map_cb(const nav_msgs::OccupancyGridConstPtr& msg) {
    // This lock lets us have multiple reads *or* one write at a time
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    map_ = msg;
    if(map_with_robots_) delete map_with_robots_;
    map_with_robots_ = new int8_t[msg->data.size()];
    update_map_with_robots();
}

bool MapManager::is_pose_valid(
    const geometry_msgs::Pose& pose, float safety_radius)
{
    std::shared_lock<std::shared_mutex> lock(map_mutex_);

    // First find the point in the occupancy grid that the pose is on
    // Map origin is the bottom left
    auto th = 2*acos(map_->info.origin.orientation.w);
    double dx = pose.position.x - map_->info.origin.position.x;
    double dy = pose.position.y - map_->info.origin.position.y;

    int row = (cos(th) * dx - sin(th) * dy) / map_->info.resolution + 0.5;
    int col = (sin(th) * dx + cos(th) * dy) / map_->info.resolution + 0.5;

    // Check that the pose is within the map boundaries
    if(row < 0) return false;
    if(col < 0) return false;
    if(row >= map_->info.height) return false;
    if(col >= map_->info.width) return false;

    // Check if the pose is already occupied in the map (i.e. by a wall)
    // Check the pose
    if(map_->data[col*map_->info.height + row] != 0) return false;
    // Check the area around the pose
    if(safety_radius < 0) safety_radius *= -1;
    if(safety_radius >= 1e-6) {
        for(int r = 0; r < 2*safety_radius / map_->info.resolution + 1; r++)
        {
            int r2 = r - safety_radius / map_->info.resolution;
            if(row + r2 < 0 || row + r2 >= map_->info.height)
                continue;
            for(int c = 0; c < 2*safety_radius / map_->info.resolution + 1;
                c++)
            {
                int c2 = c - safety_radius / map_->info.resolution;
                if(col + c2 < 0 || col + c2 >= map_->info.width)
                    continue;
                float dr = (float)r2 * map_->info.resolution;
                float dc = (float)c2 * map_->info.resolution;

                if(sqrtf(dr*dr + dc*dc) <= safety_radius) {
                    if(map_->data[(col+c2)*map_->info.height + row+r2])
                        return false;
                }
            }
        }
    }

    return true;
}

bool MapManager::is_pose_valid(
    const geometry_msgs::Pose& pose, std::string robot_name,
    float safety_radius)
{
    // First find the point in the occupancy grid that the pose is on
    // Map origin is the bottom left
    auto th = 2*acos(map_->info.origin.orientation.w);
    double dx = pose.position.x - map_->info.origin.position.x;
    double dy = pose.position.y - map_->info.origin.position.y;

    int row = (cos(th) * dx - sin(th) * dy) / map_->info.resolution + 0.5;
    int col = (sin(th) * dx + cos(th) * dy) / map_->info.resolution + 0.5;

    // Check that the pose is within the map boundaries
    if(row < 0) return false;
    if(col < 0) return false;
    if(row >= map_->info.height) return false;
    if(col >= map_->info.width) return false;

    // Check if the pose is already occupied in the map (i.e. by a wall)
    // Check the pose
    if(map_with_robots_[col*map_->info.height + row] != 0) return false;
    // Check the area around the pose
    if(safety_radius < 0) safety_radius *= -1;
    if(safety_radius >= 1e-6) {
        for(int r = 0; r < 2*safety_radius / map_->info.resolution + 1; r++)
        {
            int r2 = r - safety_radius / map_->info.resolution;
            if(row + r2 < 0 || row + r2 >= map_->info.height)
                continue;
            for(int c = 0; c < 2*safety_radius / map_->info.resolution + 1;
                c++)
            {
                int c2 = c - safety_radius / map_->info.resolution;
                if(col + c2 < 0 || col + c2 >= map_->info.width)
                    continue;
                float dr = (float)r2 * map_->info.resolution;
                float dc = (float)c2 * map_->info.resolution;

                if(sqrtf(dr*dr + dc*dc) <= safety_radius) {
                    if(map_with_robots_[(col+c2)*map_->info.height+row+r2])
                        return false;
                }
            }
        }
    }

    return true;
}

void MapManager::set_robot_pose(
    std::string robot_name, const geometry_msgs::Pose& pose,
    float robot_radius)
{
    if(robot_radius < 0) robot_radius *= -1;
    robot_poses_[robot_name] = pose;
    robot_radii_ [robot_name] = robot_radius;
    update_map_with_robot(robot_name);
}

void MapManager::update_map_with_robots() {
    for(const auto& pair : robot_poses_) {
        update_map_with_robot(pair.first);
    }
}

void MapManager::update_map_with_robot(std::string robot_name) {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);

    try {
        const auto& temp1 = robot_poses_[robot_name];
        const auto& temp2 = robot_radii_[robot_name];
    }
    catch(std::out_of_range) {
        ROS_ERROR_STREAM_THROTTLE(
            5, robot_name << " does not exist in the pose map.");
        return;
    }

    // Helper lambda to update the area around a pose within a given map
    // to a requested value
    static const auto update_map = [this](
        const geometry_msgs::Pose& pose, const float& radius, int8_t* map,
        int8_t new_val)
    {
        double th = 2*acos(map_->info.origin.orientation.w);
        double dx = pose.position.x - map_->info.origin.position.x;
        double dy = pose.position.y - map_->info.origin.position.y;

        int row = (cos(th)*dx - sin(th)*dy) / map_->info.resolution + 0.5;
        int col = (sin(th)*dx + cos(th)*dy) / map_->info.resolution + 0.5;

        map[col*map_->info.height + row] = new_val;

        // Set the area around the pose
        if(radius >= 1e-6) {
            for(int r = 0; r < 2*radius / map_->info.resolution + 1; r++)
            {
                int r2 = r - radius / map_->info.resolution;
                if(row + r2 < 0 || row + r2 >= map_->info.height)
                    continue;
                for(int c = 0; c < 2*radius / map_->info.resolution + 1;
                    c++)
                {
                    int c2 = c - radius / map_->info.resolution;
                    if(col + c2 < 0 || col + c2 >= map_->info.width)
                        continue;
                    float dr = (float)r2 * map_->info.resolution;
                    float dc = (float)c2 * map_->info.resolution;

                    if(sqrtf(dr*dr + dc*dc) <= radius) {
                        map[(col+c2)*map_->info.height+row+r2] = new_val;
                    }
                }
            }
        }
    };

    const geometry_msgs::Pose& new_pose = robot_poses_[robot_name];
    const float& radius = robot_radii_[robot_name];
    try {
        const auto& old_pose = old_robot_poses_.at(robot_name);
        // First set the tiles this robot was using to unoccupied
        update_map(old_pose, radius, map_with_robots_, 0);
    }
    catch(std::out_of_range) {
        // No problem here
    }

    // Now set the tiles the robot is using to occupied
    update_map(new_pose, radius, map_with_robots_, 100);

    // Finally, make sure that we've not lost any occupations from the main
    // map
    for(int i = 0; i < map_->data.size(); i++) {
        if(map_->data[i] && map_->data[i] != map_with_robots_[i]) {
            map_with_robots_[i] = map_->data[i];
        }
    }
}
