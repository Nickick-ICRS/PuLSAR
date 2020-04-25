#include "pose_estimators/mcl_swarm_pose_estimator.hpp"

#include "maths/useful_functions.hpp"
#include "maths/kmeans.hpp"

#include <chrono>
#include <cmath>

MCLSwarmPoseEstimator::MCLSwarmPoseEstimator(
    const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::vector<std::string>& robot_names, 
    std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
    std::map<std::string, std::string>& robot_odom_topics,
    std::map<std::string, std::string>& robot_base_links,
    std::map<std::string, float>& robot_radii, unsigned int M,
    double min_trans_update)

    :SwarmPoseEstimator(cloud_gen, map_man, map_frame), map_man_(map_man),
    M_(M), radii_(robot_radii), num_close_robots_(sqrt(robot_names.size())),
    min_robot_cloud_size_(M / pow(robot_names.size(), 2)), gen_(rd_()),
    dist_(0, 1)
{
    for(const auto& name : robot_names) {
        robot_models_[name].reset(new RobotModel(
            name, cloud_gen, map_man, map_frame, robot_odom_topics[name], 
            robot_base_links[name], robot_radii[name], min_trans_update));
        robot_pose_estimates_[name].header.frame_id = map_frame;
        robot_pose_estimates_[name].pose.pose = initial_robot_poses[name];
        robot_pose_estimates_[name].pose.covariance[0]  = 1e-6;
        robot_pose_estimates_[name].pose.covariance[7]  = 1e-6;
        robot_pose_estimates_[name].pose.covariance[35] = 1e-6;
        robot_pose_clouds_[name] = {};
        map_man_->set_robot_pose(
            name, initial_robot_poses[name], radii_[name]);
    }

    for(int i = 0; i < M; i+=robot_names.size()) {
        for(const auto& name : robot_names) {
            pose_cloud_.push_back(
                robot_models_[name]->gen_random_valid_pose(
                    robot_pose_estimates_[name].pose));
        }
    }

    while(pose_cloud_.size() > M)
        pose_cloud_.pop_back();

    swarm_pose_estimate_.header.frame_id = map_frame;
}

MCLSwarmPoseEstimator::~MCLSwarmPoseEstimator() {
    // dtor
}

void MCLSwarmPoseEstimator::update_estimate() {
    auto now = std::chrono::high_resolution_clock::now();

    // Apply odometry estimates to each point from X nearest robots.
    update_odometry_estimates();
    
    // Loop through the points and assign to robot with best weight.
    assign_robot_pose_estimates();

    // Keep points with best weight and go again.
    weigh_and_keep_points();
    
    // Update individual robot pose estimates.
    update_robot_pose_estimates();

    // Update covariance estimate.
    update_estimate_covariance();

    std::chrono::duration<double, std::milli> dt =
        std::chrono::high_resolution_clock::now() - now;
    ROS_INFO_STREAM("Update took " << dt.count() << " milliseconds.");
}

void MCLSwarmPoseEstimator::update_odometry_estimates() {
    unbounded_pose_cloud_.clear();
    for(const auto& pose : pose_cloud_) {
        std::vector<std::string> closest_bots = get_closest_robots(pose);
        for(const auto& name : closest_bots) {
            unbounded_pose_cloud_.emplace_back(name,
                robot_models_[name]->sample_motion_model_odometry(pose));
        }
    }
}

std::vector<std::string> MCLSwarmPoseEstimator::get_closest_robots(
    const geometry_msgs::Pose& p)
{
    std::vector<std::string> closest_bots;
    std::vector<double> closest_dists;
    for(const auto& pair : robot_pose_estimates_) {
        const auto& name = pair.first;
        const auto& pose = pair.second.pose.pose;
        double delta = sqrt(
            pow(pose.position.x - p.position.x, 2) 
            + pow(pose.position.y - p.position.y, 2));
        bool inserted = false;
        for(int i = 0; i < closest_bots.size(); i++) {
            if(closest_dists[i] > delta) {
                closest_bots.insert(closest_bots.begin() + i, name);
                closest_dists.insert(closest_dists.begin() + i, delta);
                if(closest_bots.size() > num_close_robots_) {
                    closest_bots.pop_back();
                    closest_dists.pop_back();
                }
                inserted = true;
                break;
            }
        }
        if(closest_bots.size() < num_close_robots_ && !inserted) {
            closest_bots.push_back(name);
            closest_dists.push_back(delta);
        }
    }
    return closest_bots;
}

void MCLSwarmPoseEstimator::assign_robot_pose_estimates() {
    for(auto& pair : robot_pose_clouds_) { 
        const auto& name = pair.first;
        auto& vec = pair.second;
        vec.clear();
        for(const auto& pair2 : unbounded_pose_cloud_) {
            const auto& name2 = pair2.first;
            const auto& pose = pair2.second;
            if(name == name2) {
                vec.push_back(pose);
            }
        }

        int counter = 0;
        int MAX = 1e4;
        while(vec.size() < min_robot_cloud_size_ && counter < MAX) {
            auto pose = robot_models_[name]->gen_random_valid_pose(
                robot_pose_estimates_[name].pose);
            pose = robot_models_[name]->sample_motion_model_odometry(pose);

            auto closest_bots = get_closest_robots(pose);
            for(const auto& bot : closest_bots) {
                if(bot == name) {
                    vec.push_back(pose);
                    break;
                }
            }
            counter++;
        }
        if(counter >= MAX) {
            ROS_ERROR_STREAM(
                "Max attempts exceeded. Only " << vec.size() 
                << " points in " << name << "'s pose estimates.");
            auto pose = robot_models_[name]->gen_random_valid_pose(
                robot_pose_estimates_[name].pose);
            pose = robot_models_[name]->sample_motion_model_odometry(pose);
            auto closest_bots = get_closest_robots(pose);
        }
    }
}

void MCLSwarmPoseEstimator::weigh_and_keep_points() {
    std::map<std::string, std::vector<double>> weights;
    std::map<std::string, double> total_weights;
    std::vector<std::string> robot_names;
    for(const auto& pair : robot_pose_clouds_) {
        const auto& name = pair.first;
        const auto& vec = pair.second;
        total_weights[name] = 0;
        for(const auto& p : vec) {
            double w = robot_models_[name]->weigh_pose(p);
            weights[name].push_back(w);
            total_weights[name] += w;
        }
        robot_names.push_back(name);
    }

    // Now points are weighed, select some at random
    double r;

    // Helper lambda function
    auto random_pose_from_robot_cloud = [this, weights, total_weights](
            std::string name)
    {
        double r = dist_(gen_) * total_weights.at(name);
        int count = 0;
        while(r > 0 && count < weights.at(name).size()) {
            r -= weights.at(name)[count];
            count++;
        }
        // We actually exit with count 1 higher than the correct value
        count--;
        return robot_pose_clouds_[name][count];
    };

    pose_cloud_.clear();
    while(pose_cloud_.size() < M_) {
        r = dist_(gen_);
        if(r > 0.9999) r = 0.9999;
        r *= robot_names.size();
        const auto& name = robot_names[(int)r];
        const auto& pose = random_pose_from_robot_cloud(name);
        pose_cloud_.push_back(pose);
    }

    // Cluster the pose cloud
    KMeans kmeans(pose_cloud_);
    // Initialise the cluster with the previous pose estimates
    std::vector<geometry_msgs::Pose> init;
    for(const auto& name : robot_names) {
        const auto& pwcs = robot_pose_estimates_[name];
        init.push_back(pwcs.pose.pose);
    }
    auto cluster_ids = kmeans.run(robot_names.size(), init);

/* 
    // Match each robot to a cluster
    auto means = kmeans.get_means();
    auto robot_cluster_map = match_robots_to_clusters(
        cluster_ids, means);
*/

    // Temporary map to store robot pose estimates
    std::map<std::string, std::vector<geometry_msgs::Pose>> temp_pose_map;
    for(unsigned int i = 0; i < cluster_ids.size(); i++) {
        unsigned int ID = cluster_ids[i];
        temp_pose_map[robot_names[ID]].push_back(pose_cloud_[i]);
    }

/*
    for(const auto& pair : robot_cluster_map) {
        const auto& name = pair.first;
        const auto& id = pair.second;
        for(int i = 0; i < cluster_ids.size(); i++) {
            if(cluster_ids[i] == id) {
                temp_pose_map[name].push_back(pose_cloud_[name]);
            }
        }
    }
*/

    // Ensure that we have enough points for each robot estimate
    for(auto& pair : temp_pose_map) {
        const auto& name = pair.first;
        auto& vec = pair.second;
        while(vec.size() < min_robot_cloud_size_) {
            vec.push_back(random_pose_from_robot_cloud(name));
        }
    }

    // Update the new robot_pose_cloud_ map
    robot_pose_clouds_ = temp_pose_map;
}

void MCLSwarmPoseEstimator::update_robot_pose_estimates() {
    for(const auto& pair : robot_pose_clouds_) {
        const auto& name = pair.first;
        const auto& vec = pair.second;
        robot_pose_estimates_[name].header.stamp = ros::Time::now();
        const auto pose = calculate_pose_with_covariance(vec);
        robot_pose_estimates_[name].pose = pose;

        auto tf = robot_models_[name]->calculate_transform(pose.pose);
        tf_broadcaster_.sendTransform(tf);
        map_man_->set_robot_pose(name, pose.pose, radii_[name]);
    }
}


std::map<std::string, unsigned int>
    MCLSwarmPoseEstimator::match_robots_to_clusters(
        const std::vector<unsigned int>& clustered_points,
        const std::map<unsigned int, geometry_msgs::Pose>& means)
{
    static auto l2 = [](
        const geometry_msgs::Pose& rbt, 
        const geometry_msgs::Pose& cluster_mean) 
    {
        double dx = rbt.position.x - cluster_mean.position.x;
        double dy = rbt.position.y - cluster_mean.position.y;
        return dx * dx + dy * dy;
    };

    // Initial assignments 
}
