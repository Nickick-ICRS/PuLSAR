#include "map_manager/map_manager.hpp"

#include <chrono> // For std::chrono::milliseconds
#include <mutex> // For std::unique_lock

MapManager::MapManager(std::string map_topic) 
    :map_with_robots_(nullptr), map_(new nav_msgs::OccupancyGrid)
{
    map_sub_ = nh_.subscribe(map_topic, 1, &MapManager::map_cb, this);

    // Block until the map comes online
    waiting_for_map_ = true;
    ROS_ERROR("1");
    while(waiting_for_map_) {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ROS_WARN_STREAM_DELAYED_THROTTLE(
            5, "No map received from '" << map_topic 
            << "' yet. Will continue waiting...");
    }
    ROS_ERROR("2");
}

MapManager::~MapManager() {
    if(map_with_robots_) {
        std::lock_guard<std::shared_mutex> lock(map_mutex_);
        delete map_with_robots_;
    }
}

void MapManager::map_cb(const nav_msgs::OccupancyGridPtr& msg) {
    // This mutex lets us have multiple reads *or* one write at a time
    std::lock_guard<std::shared_mutex> lock(map_mutex_);
    map_->header = msg->header;
    map_->info = msg->info;
    map_->data = msg->data;
    if(map_with_robots_) delete map_with_robots_;
    map_with_robots_ = new int8_t[msg->data.size()];
    update_map_with_robots();
    waiting_for_map_ = false;
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
    const auto& tiles = get_tiles(pose.position, safety_radius);
    for(const auto& t : tiles) {
        if(map_->data[t])
            return false;
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
    
    // Get the current tiles occupied by this robot so they can be ignored
    std::vector<unsigned int> to_ignore;
    try {
        const auto& old_pose = robot_poses_.at(robot_name);
        const auto& old_radius = robot_radii_.at(robot_name);
        // First set the tiles this robot was using to unoccupied
        to_ignore = get_tiles(old_pose.position, old_radius);
    }
    catch(std::out_of_range) {
        // No problem here
    }

    // Check the area around the pose
    auto tiles = get_tiles(pose.position, safety_radius);
    for(const auto& t : tiles) {
        bool ignore = false;
        for(const auto& i : to_ignore) {
            if(t==i) {
                ignore = true;
                break;
            }
        }
        if(!ignore && map_with_robots_[t])
            return false;
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

    const geometry_msgs::Pose& new_pose = robot_poses_[robot_name];
    const float& radius = robot_radii_[robot_name];
    try {
        const auto& old_pose = old_robot_poses_.at(robot_name);
        // First set the tiles this robot was using to unoccupied
        auto tiles = get_tiles(old_pose.position, radius);
        for(const auto& t : tiles)
            map_with_robots_[t] = 0;
    }
    catch(std::out_of_range) {
        // No problem here
    }

    // Now set the tiles the robot is using to occupied
    auto tiles = get_tiles(new_pose.position, radius);
    for(const auto& t : tiles)
        map_with_robots_[t] = 100;

    // Finally, make sure that we've not lost any occupations from the main
    // map
    for(int i = 0; i < map_->data.size(); i++) {
        if(map_->data[i] && map_->data[i] != map_with_robots_[i]) {
            map_with_robots_[i] = map_->data[i];
        }
    }
}

std::vector<unsigned int> MapManager::get_tiles(
    const geometry_msgs::Point& p, float radius)
{
    std::vector<unsigned int> vec;

    if(radius < 0) radius *= -1;
    double th = 2*acos(map_->info.origin.orientation.w);
    double dx = p.x - map_->info.origin.position.x;
    double dy = p.y - map_->info.origin.position.y;

    int row = (cos(th)*dx - sin(th)*dy) / map_->info.resolution + 0.5;
    int col = (sin(th)*dx + cos(th)*dy) / map_->info.resolution + 0.5;

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
                    vec.push_back((col+c2)*map_->info.height+row+r2);
                }
            }
        }
    }
    return vec;
}

double MapManager::cone_cast_plain_map(
    const geometry_msgs::Point& p, double ang, double spread)
{
    return cone_cast(p, ang, spread, (int8_t*)map_->data.data());
}

double MapManager::cone_cast_with_bots(
    const geometry_msgs::Point& p, double ang, double spread, 
    std::string robot_name)
{
    int8_t *map_copy;
    {
        std::shared_lock<std::shared_mutex> lock(map_mutex_);
        map_copy = new int8_t[map_->data.size()];
        for(unsigned int i = 0; i < map_->data.size(); i++) {
            map_copy[i] = map_with_robots_[i];
        }
    }

    try {
        const auto& old_pose = robot_poses_.at(robot_name);
        const auto& radius = robot_radii_.at(robot_name);
        // Set the tiles this robot was using to unoccupied
        auto tiles = get_tiles(old_pose.position, radius);
        for(const auto& t : tiles)
            map_copy[t] = 0;
    }
    catch(std::out_of_range) {
        // No problem here
    }

    return cone_cast(p, ang, spread, map_copy);
}

double MapManager::cone_cast(
    const geometry_msgs::Point& p, double ang, double spread,
    const int8_t *map)
{
    std::shared_lock<std::shared_mutex> lock(map_mutex_);

    double r1ang = ang - spread/2;
    double r2ang = ang + spread/2;

    const double RESOLUTION = 0.01;

    double ray_ang = r1ang;
    double closest = ray_cast(p, r1ang, map);
    for(double ray_ang = r1ang+RESOLUTION; ray_ang < r2ang;
        ray_ang += RESOLUTION) 
    {
        double dist = ray_cast(p, ray_ang, map);
        if(dist < closest)
            closest = dist;
    }
    double dist = ray_cast(p, r2ang, map);
    if(dist < closest)
        closest = dist;

    return dist;
}

double MapManager::ray_cast(
    const geometry_msgs::Point& p, double ang, const int8_t* map)
{
    const double th = 2*acos(map_->info.origin.orientation.w);
    const double dx = p.x - map_->info.origin.position.x;
    const double dy = p.y - map_->info.origin.position.y;

    ang += th;
    
    const double rx = cos(ang);
    const double ry = sin(ang);

    const int signx = rx >= 0 ? 1 : -1;
    const int signy = ry >= 0 ? 1 : -1;

    const int offsetx = rx > 0 ? 1 : 0;
    const int offsety = ry > 0 ? 1 : 0;

    double curx = cos(th)*dx - sin(th)*dy;
    double cury = sin(th)*dx + cos(th)*dy;

    const double startx = curx;
    const double starty = cury;

    int tilex = curx / map_->info.resolution + 0.5;
    int tiley = cury / map_->info.resolution + 0.5;

    if(tilex >= map_->info.width) {
        tilex = map_->info.width - 1;
    }
    if(tiley >= map_->info.height) {
        tiley = map_->info.height - 1;
    }
    if(tilex < 0) {
        tilex = 0;
    }
    if(tiley < 0) {
        tiley = 0;
    }

    double t = 0;

    double dtx = ((tilex + offsetx)*map_->info.resolution - curx) / rx;
    double dty = ((tiley + offsety)*map_->info.resolution - cury) / ry;

    while(tilex >= 0 && tilex < map_->info.width &&
          tiley >= 0 && tiley < map_->info.height) 
    {
        if(map[tiley*map_->info.width + tilex]) return t;

        dtx = ((tilex + offsetx)*map_->info.resolution - curx) / rx; 
        dty = ((tiley + offsety)*map_->info.resolution - cury) / ry;

        if(dtx < dty) {
            t += dtx;
            tilex += signx;
        }
        else {
            t += dty;
            tiley += signy;
        }

        curx = startx + rx * t;
        cury = starty + ry * t;
    }

    return t;
}
