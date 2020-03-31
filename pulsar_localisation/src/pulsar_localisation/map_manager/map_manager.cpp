#include "map_manager/map_manager.hpp"

#include <chrono> // For std::chrono::milliseconds
#include <mutex> // For std::unique_lock

MapManager::MapManager(std::string map_topic) :map_with_robots_(nullptr) {
    map_sub_ = nh_.subscribe(map_topic, 1, &MapManager::map_cb, this);

    // Block until the map comes online
    bool waiting_for_map = true;
    ROS_ERROR("1");
    while(waiting_for_map) {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ROS_WARN_STREAM_DELAYED_THROTTLE(
            5, "No map received from '" << map_topic 
            << "' yet. Will continue waiting...");
        std::shared_lock<std::shared_mutex> lock(map_mutex_);
        if(map_)
            waiting_for_map = false;
    }
    ROS_ERROR("2");
}

MapManager::~MapManager() {
    if(map_with_robots_) {
        std::unique_lock<std::shared_mutex> lock(map_mutex_);
        delete map_with_robots_;
    }
}

void MapManager::map_cb(const nav_msgs::OccupancyGridPtr& msg) {
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
        map_copy = new int8_t(map_->data.size());
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
    while(r1ang > M_PI) r1ang -= 2*M_PI;
    while(r1ang < -M_PI) r1ang += 2*M_PI;
    while(r2ang > M_PI) r2ang -= 2*M_PI;
    while(r2ang < -M_PI) r2ang += 2*M_PI;

    auto ray1 = ray_cast(p, r1ang);
    auto ray2 = ray_cast(p, r2ang);
    // Loop through tiles from the first ray to the second ray, starting 
    // at the origin
    // Both top left or bottom left - ray1 will always be higher
    if(M_PI >= r1ang && r2ang >= 0) {
        auto it1 = ray1.begin();
        auto it2 = ray2.begin();
        double dist = 1e10;
        unsigned int max_y = map_->info.height;
        for(unsigned int x = it1->first; x != -1; x--) {
            while(it1->first >= x && it1 != ray1.end()) it1++;
            while(it2->first >= x && it2 != ray2.end()) it2++;
            it1--;
            it2--;
            unsigned int y = it1->second;
            if(y > max_y) y = max_y;
            if(y < it2->second) return dist;

            for(y; y != it2->second-1; y--) {
                if(map[map_->info.width*y + x]) {
                    // We've hit an object, if it's closer then store
                    // max y and distance
                    double new_dist = sqrt(pow(y-p.y, 2) + pow(x-p.x, 2));
                    new_dist *= map_->info.resolution;
                    if(new_dist < dist) {
                        dist = new_dist;
                        max_y = y;
                    }
                }
            }
        }
        // In case we didn't return in the loop
        return dist;
    }
    // Both on right - ray2 will always be higher
    else if(0 >= r2ang && r1ang >= -M_PI) {
        auto it1 = ray1.begin();
        auto it2 = ray2.begin();
        double dist = 1e10;
        unsigned int max_y = map_->info.height;
        for(unsigned int x = it1->first; x != map_->info.width; x++) {
            while(it1->first <= x && it1 != ray1.end()) it1++;
            while(it2->first <= x && it2 != ray2.end()) it2++;
            it1--;
            it2--;
            unsigned int y = it2->second;
            if(y > max_y) y = max_y;
            if(y < it1->second) return dist;

            for(y; y != it1->second-1; y--) {
                if(map[map_->info.width*y + x]) {
                    // We've hit an object, if it's closer then store
                    // max y and distance
                    double new_dist = sqrt(pow(y-p.y, 2) + pow(x-p.x, 2));
                    new_dist *= map_->info.resolution;
                    if(new_dist < dist) {
                        dist = new_dist;
                        max_y = y;
                    }
                }
            }
        }
        // In case we didn't return in the loop
        return dist;
    }
    // Ray 2 is on the left, but ray 1 is on the right - swap x and y
    else if(0 > r1ang) {
        auto it1 = ray1.begin();
        auto it2 = ray2.begin();
        double dist = 1e10;
        unsigned int max_x = map_->info.width;
        for(unsigned int y = it1->second; y != map_->info.height; y++) {
            while(it1->second <= y && it1 != ray1.end()-1) it1++;
            while(it2->second <= y && it2 != ray2.end()-1) it2++;
            it1--;
            it2--;
            unsigned int x = it1->first;
            if(x > max_x) x = max_x;
            if(x < it2->first) return dist;

            for(x; x != it2->first-1; x--) {
                if(map[map_->info.width*y + x]) {
                    // We've hit an object, if it's closer then store
                    // max y and distance
                    double new_dist = sqrt(pow(y-p.y, 2) + pow(x-p.x, 2));
                    new_dist *= map_->info.resolution;
                    if(new_dist < dist) {
                        dist = new_dist;
                        max_x = x;
                    }
                }
            }
        }
        // In case we didn't return in the loop
        return dist;
    }
    // Ray 1 is on the left, but ray 2 is on the right
    else {
        auto it1 = ray1.begin();
        auto it2 = ray2.begin();
        double dist = 1e10;
        unsigned int max_x = map_->info.width;
        for(unsigned int y = it1->second; y != -1; y--) {
            while(it1->second >= y && it1 != ray1.end()-1) it1++;
            while(it2->second >= y && it2 != ray2.end()-1) it2++;
            it1--;
            it2--;
            unsigned int x = it2->first;
            if(x > max_x) x = max_x;
            if(x < it1->first) return dist;

            for(x; x != it1->first-1; x--) {
                if(map[map_->info.width*y + x]) {
                    // We've hit an object, if it's closer then store
                    // max y and distance
                    double new_dist = sqrt(pow(y-p.y, 2) + pow(x-p.x, 2));
                    new_dist *= map_->info.resolution;
                    if(new_dist < dist) {
                        dist = new_dist;
                        max_x = x;
                    }
                }
            }
        }
        // In case we didn't return in the loop
        return dist;
    }
}

std::vector<std::pair<unsigned int, unsigned int>> MapManager::ray_cast(
    const geometry_msgs::Point& p, double ang)
{
    double th = 2*acos(map_->info.origin.orientation.w);
    double dx = p.x - map_->info.origin.position.x;
    double dy = p.y - map_->info.origin.position.y;

    ang -= th;
    
    double rx = cos(ang);
    double ry = sin(ang);

    int signx = rx >= 0 ? 1 : -1;
    int signy = ry >= 0 ? 1 : -1;

    int offsetx = rx > 0 ? 0 : -1;
    int offsety = ry > 0 ? 0 : -1;

    double curx = p.x;
    double cury = p.y;

    double tilex = (cos(th)*dx - sin(th)*dy) / map_->info.resolution + 0.5;
    double tiley = (sin(th)*dx + cos(th)*dy) / map_->info.resolution + 0.5;

    if(tilex >= map_->info.width) {
        ROS_INFO_STREAM("tilex: " << tilex);
        tilex = map_->info.width - 1;
    }
    if(tiley >= map_->info.height) {
        ROS_INFO_STREAM("tiley: " << tiley);
        tiley = map_->info.height - 1;
    }
    if(tilex < 0) {
        ROS_INFO_STREAM("tilex: " << tilex);
        tilex = 0;
    }
    if(tiley < 0) {
        ROS_INFO_STREAM("tiley: " << tiley);
        tiley = 0;
    }

    double t = 0;

    double dtx = ((tilex + offsetx)*map_->info.resolution - curx) / rx;
    double dty = ((tiley + offsety)*map_->info.resolution - cury) / ry;

    std::vector<std::pair<unsigned int, unsigned int>> ray;
    while(tilex >= 0 && tilex < map_->info.width &&
          tiley >= 0 && tiley < map_->info.height) 
    {
        ray.emplace_back(tilex, tiley);

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

        curx = p.x + rx * t;
        cury = p.y + ry * t;
    }

    return ray;
}
