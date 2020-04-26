#include "robot_models/scan_matching_robot_model.hpp"

#include "maths/useful_functions.hpp"

#include <numeric>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>

ScanMatchingRobotModel::ScanMatchingRobotModel(
    std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::string odom_topic, std::string base_link_frame, float radius,
    double min_trans_update)

    :OdometryRobotModel(
        name, cloud_gen, map_man, map_frame, odom_topic, base_link_frame, 
        radius, min_trans_update), tf_listener_(tf2_)
{
    // ctor
}
ScanMatchingRobotModel::~ScanMatchingRobotModel() {
    // dtor
}
geometry_msgs::Pose ScanMatchingRobotModel::sample_motion_model(
    const geometry_msgs::Pose& xt_1)
{
    // Sample odometry motion model for initial estimate
    auto xt = OdometryRobotModel::sample_motion_model(xt_1);

    // Laser scan match to adjust pose to fit the map.
    return scan_match_l2(xt);
}

geometry_msgs::Pose ScanMatchingRobotModel::scan_match_l2(
    const geometry_msgs::Pose& p)
{
    // Helper lambdas to calculate errors
    static auto l2_pos_err = [](
        const geometry_msgs::Pose& a, const geometry_msgs::Pose& b,
        double scalex, double scaley)
    {
        double dx = a.position.x - b.position.x;
        double dy = a.position.y - b.position.y;
        double dz = quat_to_yaw(a.orientation) - quat_to_yaw(b.orientation);
        if(dz > M_PI) dz -= 2*M_PI;
        if(dz <= -M_PI) dz += 2*M_PI;

        dx /= scalex;
        dy /= scaley;
        dz /= M_PI;

        return dx * dx + dy * dy + dz * dz;
    };

    // Order to check data
    static const std::vector<std::tuple<Dir, Dir, Dir>> movement_orders = {
        { Dir::PX, Dir::PY, Dir::PTH }, {Dir::PX, Dir::PY, Dir::NTH},
        { Dir::PX, Dir::NY, Dir::PTH }, {Dir::PX, Dir::NY, Dir::NTH},
        { Dir::NX, Dir::PY, Dir::PTH }, {Dir::NX, Dir::PY, Dir::NTH},
        { Dir::NX, Dir::NY, Dir::PTH }, {Dir::NX, Dir::NY, Dir::NTH}
    };

    double scalex = map_man_->get_width();
    double scaley = map_man_->get_height();

    const auto Z_raw = cloud_gen_->get_raw_data(name_);

    // Transform the range data into our coordinate frame
    std::vector<geometry_msgs::Point> Z;
    for(const auto& z : Z_raw) {
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = z.range;
        ps.pose.orientation.w = 1;
        ps.header = z.header;
        try {
            tf2_.transform(ps, ps, base_link_frame_);
        }
        catch(const tf2::TransformException & ex) {
            ROS_WARN_STREAM(
                "move_in_dir caught exception: " << ex.what());
            continue;
        }

        double x = ps.pose.position.x;
        double y = ps.pose.position.y;
        double yaw = quat_to_yaw(p.orientation);

        ps.pose.position.x = p.position.x + x * cos(yaw) + y * sin(yaw);
        ps.pose.position.y = p.position.y - x * sin(yaw) + y * cos(yaw);

        Z.push_back(ps.pose.position);
    }

    std::vector<geometry_msgs::Pose> poses;
    std::vector<double> weights;

    for(auto& tup : movement_orders) {
        geometry_msgs::Pose p2(p);
        // Can't for loop this coz the compiler complains
        std::vector<Dir> dirs(3);
        dirs[0] = std::get<0>(tup);
        dirs[1] = std::get<1>(tup);
        dirs[2] = std::get<2>(tup);
        double dist, weight;
        geometry_msgs::Point pivot;
        for(const auto& dir : dirs) {
            double yaw, x, y;
            std::pair<double, double> pair;
            std::tuple<double, double, geometry_msgs::Point> rot_tup;
            switch(dir) {
            case Dir::PX:
                pair = move_in_dir(p2, Z, dir);
                dist = pair.first;
                weight = pair.second;
                p2.position.x += dist;
                break;
            case Dir::NX:
                pair = move_in_dir(p2, Z, dir);
                dist = pair.first;
                weight = pair.second;
                p2.position.x -= dist;
                break;
            case Dir::PY:
                pair = move_in_dir(p2, Z, dir);
                dist = pair.first;
                weight = pair.second;
                p2.position.y += dist;
                break;
            case Dir::NY:
                pair = move_in_dir(p2, Z, dir);
                dist = pair.first;
                weight = pair.second;
                p2.position.y -= dist;
                break;
            case Dir::PTH:
            case Dir::NTH:
                break;
                /*
            case Dir::PTH:
                rot_tup = rotate_in_dir(p2, Z, dir);
                dist = std::get<0>(rot_tup);
                weight = std::get<1>(rot_tup);
                pivot = std::get<2>(rot_tup);
                x = p2.position.x - pivot.x;
                y = p2.position.y - pivot.y;
                p2.position.x =  x * cos(dist) + y * sin(dist) + pivot.x;
                p2.position.y = -x * sin(dist) + y * cos(dist) + pivot.y;
                yaw = quat_to_yaw(p.orientation) + dist;
                p2.orientation = yaw_to_quat(yaw);
                break;
            case Dir::NTH:
                rot_tup = rotate_in_dir(p2, Z, dir);
                dist = std::get<0>(rot_tup);
                weight = std::get<1>(rot_tup);
                pivot = std::get<2>(rot_tup);
                x = p2.position.x - pivot.x;
                y = p2.position.y - pivot.y;
                p2.position.x = x * cos(dist) - y * sin(dist) + pivot.x;
                p2.position.y = x * sin(dist) + y * cos(dist) + pivot.y;
                yaw = quat_to_yaw(p.orientation) - dist;
                p2.orientation = yaw_to_quat(yaw);
                break;*/
            default:
                throw std::logic_error("Requested dir not implemented!");
            }
        }
        poses.push_back(p2);
        weights.push_back(weight);
    }

    // Now pick the pose with the least error
    double best_err = std::numeric_limits<double>::max();
    auto& best_pose = poses[0];
    // Find normalising values
    std::vector<double> l2(poses.size());
    double norm_l2 = 0;
    double norm_weight = std::numeric_limits<double>::max();

    for(unsigned int i = 0; i < poses.size(); i++) {
        double err = l2_pos_err(p, poses[i], scalex, scaley);
        l2[i] = err;
        if(err > norm_l2)
            norm_l2 = err;
        if(weights[i] < norm_weight && weights[i] != 0)
            norm_weight = weights[i];
    }

    for(unsigned int i = 0; i < poses.size(); i++) {
        double err = l2[i] / norm_l2 + norm_weight / weights[i];
        if(err < best_err) {
            best_err = err;
            best_pose = poses[i];
        }
    }

    return best_pose;
}


std::pair<double, double> ScanMatchingRobotModel::move_in_dir(
    const geometry_msgs::Pose& p,
    const std::vector<geometry_msgs::Point>& Z, Dir dir)
{
    // Used to calculate distances
    std::vector<double> dists;
    if(dir == Dir::PX || dir == Dir::PY || dir == Dir::NX || dir == Dir::NY)
    {
        double ang = 0;
        ang = dir == Dir::NX ?    M_PI : ang;
        ang = dir == Dir::PY ?  M_PI/2 : ang;
        ang = dir == Dir::NY ? -M_PI/2 : ang;

        // Calculate the distances 
        for(const auto& z : Z) {
            dists.push_back(map_man_->cone_cast_with_bots(
                z, ang, 0, name_));
        }
    }
    else {
        throw std::logic_error("Requested dir not implemented!");
    }

    // If we have no data warn and return 0 distance, max error
    if(!dists.size()) {
        ROS_WARN("No distance data during range matching");
        return {0, std::numeric_limits<double>::max()};
    }
    
    // Minimise the square of the distances
    double best_score = std::numeric_limits<double>::max();
    double best_dist = dists[0];

    for(unsigned int i = 0; i < dists.size(); i++) {
        double score = 0;
        if(dists[i] == std::numeric_limits<double>::max())
            continue;
        for(unsigned int j = 0; j < dists.size(); j++) {
            if(i != j && dists[j] != std::numeric_limits<double>::max()) {
                score += pow(dists[i] - dists[j], 2);
            }
        }
        if(score < best_score) {
            best_score = score;
            best_dist = dists[i];
        }
    }
    return { best_dist, best_score };
}

std::tuple<double, double, geometry_msgs::Point> 
    ScanMatchingRobotModel::rotate_in_dir(
        const geometry_msgs::Pose& p,
        const std::vector<geometry_msgs::Point>& Z, Dir dir)
{
    if(dir != Dir::PTH && dir != Dir::NTH) {
        throw std::logic_error("Requested dir not implemented!");
    }
    // TODO: Find a smarter way of doing this than just rotating in
    // small intervals.
    double ROT = M_PI / 20 * (dir == Dir::PTH ? 1 : -1);
    auto best_pivot = Z.front();
    double best_pivot_weight = std::numeric_limits<double>::max();
    double best_pivot_ang = 0;
    for(const auto& z : Z) {
        std::vector<geometry_msgs::Point> pivot_Z;
        for(const auto& z2: Z) {
            if(z != z2)
                pivot_Z.push_back(z2);
        }
        double total_ang = 0;
        // Initial weight
        double iw = map_man_->cone_cast_with_bots(z, -M_PI/2, 0, name_);
        for(unsigned short i = 0; i < 3; i++) {
            double w = map_man_->cone_cast_with_bots(z, i*M_PI/2, 0, name_);
            if(w < iw)
                iw = w;
        }

        double best_weight = std::numeric_limits<double>::max();
        double best_ang = total_ang;

        while(fabs(total_ang) < M_PI) {
            double weight = iw;
            for(auto& z2 : pivot_Z) {
                // Pivot point about the pivot
                double x = z2.x - z.x;
                double y = z2.y - z.y;
                z2.x =  x * cos(total_ang) + y * sin(total_ang) + z.x;
                z2.y = -x * sin(total_ang) + y * cos(total_ang) + z.y;
                // Weigh point
                double w = map_man_->cone_cast_with_bots(
                    z2, -M_PI/2, 0, name_);
                for(unsigned short i = 0; i < 3; i++) {
                    double w2 = map_man_->cone_cast_with_bots(
                        z2, i*M_PI/2, 0, name_);
                    if(w2 < w)
                        w = w2;
                }
                weight += w;
            }

            if(weight < best_weight) {
                best_weight = weight;
                best_ang = total_ang;
            }

            total_ang += ROT;
        }
        if(best_weight < best_pivot_weight) {
            best_pivot_weight = best_weight;
            best_pivot = z;
            best_pivot_ang = best_ang;
        }
    }

    return std::make_tuple(
        best_pivot_ang, pow(best_pivot_weight, 2), best_pivot);
}
