#include "map_manager/map_manager.hpp"

#include "maths/useful_functions.hpp"

#include <chrono> // For std::chrono::milliseconds
#include <mutex> // For std::unique_lock
#include <cmath>

MapManager::MapManager(std::string map_topic) 
    :map_(new nav_msgs::OccupancyGrid)
{
    map_sub_ = nh_.subscribe(map_topic, 1, &MapManager::map_cb, this);

    // Block until the map comes online
    waiting_for_map_ = true;
    while(waiting_for_map_) {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ROS_WARN_STREAM_DELAYED_THROTTLE(
            5, "No map received from '" << map_topic 
            << "' yet. Will continue waiting...");
    }
}

MapManager::~MapManager() {
    // dtor
}

void MapManager::publish_maps() {
    for(const auto& pair : map_with_robots_pubs_) {
        const ros::Publisher& pub = pair.second;
        std::string robot_name = pair.first;
        nav_msgs::OccupancyGrid msg;
        std::shared_lock<std::shared_mutex> lock(map_mutex_);
        msg.header = map_->header;
        msg.info = map_->info;
        msg.header.stamp = ros::Time::now();
        msg.data = maps_with_robots_[robot_name];
        pub.publish(msg);
    }
}

void MapManager::map_cb(const nav_msgs::OccupancyGridPtr& msg) {
    // This mutex lets us have multiple reads *or* one write at a time
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    map_->header = msg->header;
    map_->info = msg->info;
    map_data_ = msg->data;

    update_map_with_robots();
    map_th_ = quat_to_yaw(msg->info.origin.orientation);
    map_x_ = msg->info.origin.position.x;
    map_y_ = msg->info.origin.position.y;
    map_res_ = msg->info.resolution;
    map_height_ = msg->info.height;
    map_width_ = msg->info.width;

    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for(int c = 0; c < map_width_; c++) {
        for(int r = 0; r < map_height_; r++) {
            if(map_data_[r * map_height_ + c]) {
                // Check that at least one of the adjacent squares is empty
                bool edge = false;
                for(int i = -1; i < 2; i++) {
                    for(int k = -1; k < 2; k++) {
                        if(k != i && r+i < map_height_ && r+i >= 0 && 
                           c+k < map_width_ && c+k >= 0)
                        {
                            if(!map_data_[(r+i) * map_height_ + c+k])
                                edge = true;
                        }
                    }
                }
                if(edge) {
                    double x = (c * cos(map_th_) - r * sin(map_th_))
                             * map_res_ + map_x_;
                    double y = (r * cos(map_th_) + c * sin(map_th_))
                             * map_res_ + map_y_;
                    map_cloud_->push_back(pcl::PointXYZ(x, y, 0));
                }
            }
        }
    }
    map_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    map_tree_->setInputCloud(map_cloud_);
    
    waiting_for_map_ = false;
}

bool MapManager::is_pose_valid(
    const geometry_msgs::Pose& pose, float safety_radius)
{
    // First find the point in the occupancy grid that the pose is on
    // Map origin is the bottom left
    double dx = pose.position.x - map_x_;
    double dy = pose.position.y - map_y_;

    int row = ( cos(map_th_) * dx + sin(map_th_) * dy) / map_res_ + 0.5;
    int col = (-sin(map_th_) * dx + cos(map_th_) * dy) / map_res_ + 0.5;

    // Check that the pose is within the map boundaries
    if(row < 0) return false;
    if(col < 0) return false;
    if(row >= map_height_) return false;
    if(col >= map_width_) return false;

    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    // Check that the pose is already occupied in the map (i.e. by a wall)
    const auto& tiles = get_tiles(pose.position, safety_radius);
    for(const auto& t : tiles) {
        if(map_data_[t])
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
    double dx = pose.position.x - map_x_;
    double dy = pose.position.y - map_y_;

    int row = ( cos(map_th_) * dx + sin(map_th_) * dy) / map_res_ + 0.5;
    int col = (-sin(map_th_) * dx + cos(map_th_) * dy) / map_res_ + 0.5;

    // Check that the pose is within the map boundaries
    if(row < 0) return false;
    if(col < 0) return false;
    if(row >= map_height_) return false;
    if(col >= map_width_) return false;

    // Check if the pose is already occupied in the map (i.e. by a wall)
    auto tiles = get_tiles(pose.position, safety_radius);
    for(const auto& t : tiles) {
        if(maps_with_robots_[robot_name][t])
            return false;
    }

    return true;
}

geometry_msgs::Pose MapManager::make_pose_valid(
        const geometry_msgs::Pose& pose, float safety_radius)
{
    if(is_pose_valid(pose, safety_radius))
        return pose;

    const double DELTA = map_res_;

    double yaw = quat_to_yaw(pose.orientation);
    double dx = -cos(yaw) * DELTA;
    double dy = -sin(yaw) * DELTA;
    geometry_msgs::Pose p(pose);
    double d = 0;
    double max_d = sqrt(pow(map_x_, 2) + pow(map_y_, 2));
    for(d = DELTA; d < safety_radius; d+= DELTA) {
        p.position.x += dx;
        p.position.y += dy;
        d += DELTA;
        if(is_pose_valid(p, safety_radius))
            return p;
    }
    while(!is_pose_valid(p, safety_radius) && d < max_d) {
        p.position.x += dx;
        p.position.y += dy;
        d += DELTA;
    }

    if(d >= max_d) {
        d = 0;
        p = pose;
        while(!is_pose_valid(p, safety_radius) && d < max_d) {
            p.position.x -= dx;
            p.position.y -= dy;
            d += DELTA;
        }
        if(d >= max_d) {
            throw PoseInvalidException();
        }
    }

    return p;
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
    try {
        const auto& temp1 = robot_poses_.at(robot_name);
        const auto& temp2 = robot_radii_.at(robot_name);
    }
    catch(std::out_of_range) {
        ROS_ERROR_STREAM_THROTTLE(
            5, robot_name << " does not exist in the pose map.");
        return;
    }

    // Check that the map for the robot exists
    try {
        const auto& temp = maps_with_robots_.at(robot_name);
    }
    catch(std::out_of_range) {
        std::unique_lock<std::shared_mutex> lock(map_mutex_);
        maps_with_robots_[robot_name] = map_data_;
        map_with_robots_pubs_[robot_name] = 
            nh_.advertise<nav_msgs::OccupancyGrid>(robot_name+"/map", 10);
    }

    const geometry_msgs::Pose& new_pose = robot_poses_[robot_name];
    const float& radius = robot_radii_[robot_name];
    std::vector<unsigned int> old_tiles;
    try {
        const auto& old_pose = old_robot_poses_.at(robot_name);
        // First get the tiles this robot was using to set to unoccupied
        old_tiles = get_tiles(old_pose.position, radius);
    }
    catch(std::out_of_range) {
        // No problem here
    }
    old_robot_poses_[robot_name] = new_pose;

    // Now get the tiles the robot is using to set to occupied
    auto new_tiles = get_tiles(new_pose.position, radius);

    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    for(auto& pair : maps_with_robots_) {
        // Don't update this robot's map
        if(pair.first == robot_name)
            continue;

        for(const auto& t : old_tiles)
            pair.second[t] = 0;
        
        for(const auto& t : new_tiles)
            pair.second[t] = 100;
        
        // Also make sure we didn't lose any map boundaries
        for(int i = 0; i < map_data_.size(); i++) {
            if(map_data_[i] && map_data_[i] != pair.second[i]) {
                pair.second[i] = map_data_[i];
            }
        }
    }
}

std::vector<unsigned int> MapManager::get_tiles(
    const geometry_msgs::Point& p, float radius)
{
    std::vector<unsigned int> vec;

    if(radius < 0) radius *= -1;
    double dx = p.x - map_x_;
    double dy = p.y - map_y_;

    int row = ( cos(map_th_)*dx + sin(map_th_)*dy) / map_res_ + 0.5;
    int col = (-sin(map_th_)*dx + cos(map_th_)*dy) / map_res_ + 0.5;

    // Set the area around the pose
    if(radius >= 1e-6) {
        for(int r = 0; r < 2*radius / map_res_ + 1; r++)
        {
            int r2 = r - radius / map_res_;
            if(row + r2 < 0 || row + r2 >= map_height_)
                continue;
            for(int c = 0; c < 2*radius / map_res_ + 1;
                c++)
            {
                int c2 = c - radius / map_res_;
                if(col + c2 < 0 || col + c2 >= map_width_)
                    continue;
                float dr = (float)r2 * map_res_;
                float dc = (float)c2 * map_res_;

                if(sqrtf(dr*dr + dc*dc) <= radius) {
                    vec.push_back((col+c2)*map_height_+row+r2);
                }
            }
        }
    }
    return vec;
}

double MapManager::cone_cast_plain_map(
    const geometry_msgs::Point& p, double ang, double spread)
{
    return cone_cast(p, ang, spread, map_data_);
}

double MapManager::cone_cast_with_bots(
    const geometry_msgs::Point& p, double ang, double spread, 
    std::string robot_name)
{
    try {
        auto& map = maps_with_robots_.at(robot_name);
    }
    catch(std::out_of_range) {
        std::unique_lock<std::shared_mutex> lock(map_mutex_);
        maps_with_robots_[robot_name] = map_data_;
    }
    return cone_cast(p, ang, spread, maps_with_robots_[robot_name]);
}

double MapManager::cone_cast(
    const geometry_msgs::Point& p, double ang, double spread,
    const std::vector<int8_t>& map)
{
    double r1ang = ang - spread/2;
    double r2ang = ang + spread/2;

    const double RESOLUTION = spread/5;

    std::shared_lock<std::shared_mutex> lock(map_mutex_);
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
    const geometry_msgs::Point& p, double ang,
    const std::vector<int8_t>& map)
{
    ang -= map_th_;

    const double dx = p.x - map_x_;
    const double dy = p.y - map_y_;
    
    const double rx = cos(ang);
    const double ry = sin(ang);

    const int signx = rx >= 0 ? 1 : -1;
    const int signy = ry >= 0 ? 1 : -1;

    const int offsetx = rx > 0 ? 1 : 0;
    const int offsety = ry > 0 ? 1 : 0;

    double curx =  cos(map_th_)*dx + sin(map_th_)*dy;
    double cury = -sin(map_th_)*dx + cos(map_th_)*dy;

    const double startx = curx;
    const double starty = cury;

    int tilex = curx / map_res_ + 0.5;
    int tiley = cury / map_res_ + 0.5;

    if(tilex >= map_width_) {
        tilex = map_width_ - 1;
    }
    if(tiley >= map_height_) {
        tiley = map_height_ - 1;
    }
    if(tilex < 0) {
        tilex = 0;
    }
    if(tiley < 0) {
        tiley = 0;
    }

    double t = 0;

    double dtx = ((tilex + offsetx)*map_res_ - curx) / rx;
    double dty = ((tiley + offsety)*map_res_ - cury) / ry;

    while(tilex >= 0 && tilex < map_width_ &&
          tiley >= 0 && tiley < map_height_) 
    {
        if(map[tiley*map_width_ + tilex]) return t;

        dtx = ((tilex + offsetx)*map_res_ - curx) / rx; 
        dty = ((tiley + offsety)*map_res_ - cury) / ry;

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
