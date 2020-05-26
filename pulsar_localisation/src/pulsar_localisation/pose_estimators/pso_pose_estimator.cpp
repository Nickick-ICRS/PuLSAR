#include "pose_estimators/pso_pose_estimator.hpp"

#include "maths/useful_functions.hpp"

#include "robot_models/scan_matching_robot_model.hpp"

int    PSOPoseEstimator::max_its_;
double PSOPoseEstimator::omega_;
double PSOPoseEstimator::phi_p_;
double PSOPoseEstimator::phi_g_;
double PSOPoseEstimator::phi_r_;

ros::Publisher test_pub;

PSOPoseEstimator::PSOPoseEstimator(
    const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::vector<std::string>& robot_names,
    std::map<std::string, geometry_msgs::Pose>& initial_robot_poses,
    std::map<std::string, std::string>& robot_odom_topics,
    std::map<std::string, std::string>& robot_base_links,
    std::map<std::string, float>& robot_radii,
    unsigned int swarm_particles, unsigned int robot_particles,
    std::string robot_model, bool use_initial_poses)

    :SwarmPoseEstimator(cloud_gen, map_man, map_frame), gen_(rd_()),
    dist_(0, 1), radii_(robot_radii), map_man_(map_man)
{
    test_pub = nh_.advertise<sensor_msgs::PointCloud>("test2", 1);
    // Initialise robot particle clouds
    for(const auto& name : robot_names) {
        if(robot_model == SCAN_MATCHING_ROBOT_MODEL) {
            robot_models_[name].reset(new ScanMatchingRobotModel(
                name, cloud_gen, map_man, map_frame,
                robot_odom_topics[name], robot_base_links[name],
                robot_radii[name], 0.01));
        }
        else if(robot_model == ODOMETRY_ROBOT_MODEL) {
            robot_models_[name].reset(new OdometryRobotModel(
                name, cloud_gen, map_man, map_frame,
                robot_odom_topics[name], robot_base_links[name],
                robot_radii[name], 0.01));
        }
        else {
            ROS_ERROR_STREAM(
                "Unknown robot model: " << robot_model 
                << ". Available options: " << SCAN_MATCHING_ROBOT_MODEL
                << ", " << ODOMETRY_ROBOT_MODEL);
            throw;
        }

        if(robot_particles < 1) robot_particles = 1;
        for(int i = 0; i < robot_particles; i++) {
            if(use_initial_poses) {
                geometry_msgs::Pose p = initial_robot_poses[name];
                p.position.x += dist_(gen_) * 0.1 - 0.05;
                p.position.y += dist_(gen_) * 0.1 - 0.05;
                p.orientation = yaw_to_quat(
                    quat_to_yaw(p.orientation) + dist_(gen_) * 0.5 - 0.25);
                robot_particle_clouds_[name].particles.push_back(p);
            }
            else {
                robot_particle_clouds_[name].particles.push_back(
                    robot_models_[name]->gen_random_valid_pose());
            }
            robot_particle_clouds_[name].particle_bests.push_back(
                robot_particle_clouds_[name].particles.back());
            robot_particle_clouds_[name].velocities.emplace_back();
        }
        robot_particle_clouds_[name].best_pose =
            robot_particle_clouds_[name].particle_bests[0];
        robot_particle_clouds_[name].best_frame_pose =
            robot_particle_clouds_[name].best_pose;
    }

    // For random pose generation
    auto& model = robot_models_[robot_names[0]];

    // Initialise swarm particle cloud
    if(swarm_particles < 1) swarm_particles = 1;
    geometry_msgs::Pose avg_pose;
    if(use_initial_poses) {
        for(auto& pair : robot_particle_clouds_) {
            auto& cloud = pair.second;
            avg_pose.position.x += cloud.best_pose.position.x;
            avg_pose.position.y += cloud.best_pose.position.y;
        }
        avg_pose.position.x /= robot_names.size();
        avg_pose.position.y /= robot_names.size();
    }
    for(int i = 0; i < swarm_particles; i++) {
        if(use_initial_poses) {
            geometry_msgs::Pose p = avg_pose;
            p.position.x += dist_(gen_) * 0.1 - 0.05;
            p.position.y += dist_(gen_) * 0.1 - 0.05;
            p.orientation = yaw_to_quat(
                quat_to_yaw(p.orientation) + dist_(gen_) * 0.5 - 0.25);
            swarm_particle_cloud_.particles.push_back(p);
        }
        else {
            swarm_particle_cloud_.particles.push_back(
                model->gen_random_valid_pose());
        }
        swarm_particle_cloud_.particle_bests.push_back(
            swarm_particle_cloud_.particles.back());
        swarm_particle_cloud_.velocities.emplace_back();
    }
    swarm_particle_cloud_.best_pose =
        swarm_particle_cloud_.particle_bests[0];
}

PSOPoseEstimator::~PSOPoseEstimator() {
    // dtor
}

void PSOPoseEstimator::update_estimate() {
    auto now = std::chrono::high_resolution_clock::now();
    bool update_complete = false;
    int its = 0;
    // reset robot cloud bests
    for(auto& pair : robot_particle_clouds_) {
        auto& cloud = pair.second;
        auto& model = robot_models_[pair.first];
        cloud.best_pose = cloud.particle_bests[0];
        for(int i = 0; i < cloud.particles.size(); i++) {
            cloud.particle_bests[i] = cloud.particles[i];
            if(model->weigh_pose(cloud.particles[i])
               > model->weigh_pose(cloud.best_pose))
            {
                cloud.best_pose = cloud.best_frame_pose;
                cloud.best_frame_pose = cloud.particles[0];
            }
        }
    }
    while(!update_complete) {
        update_complete = false;
        for(auto& pair : robot_models_) {
            // If robot estimates improve then we continue looping
            if(update_robot_particles(pair.first))
                update_complete = false;
        }
        // If the swarm estimate improves then we continue looping
        if(update_swarm_particles())
            update_complete = false;
        if(++its >= max_its_ && max_its_ > 0)
            update_complete = true;

        sensor_msgs::PointCloud pc;
        geometry_msgs::Point swarm;
        swarm.x = swarm_pose_estimate_.pose.pose.position.x;
        swarm.y = swarm_pose_estimate_.pose.pose.position.y;
        for(const auto& pair : robot_particle_clouds_) {
            for(const auto& p : pair.second.particles) {
                geometry_msgs::Point32 p32;
                p32.x = p.position.x;// + swarm.x;
                p32.y = p.position.y;// + swarm.y;
                pc.points.push_back(p32);
            }
        }
        pc.header.frame_id = "map";
        pc.header.stamp = ros::Time::now();
        test_pub.publish(pc);
    }

    update_robot_pose_estimates();

    update_estimate_covariance();

    std::chrono::duration<double, std::milli> dt =
        std::chrono::high_resolution_clock::now() - now;
    ROS_INFO_STREAM("Update took " << dt.count() << " milliseconds.");
}

void PSOPoseEstimator::calc_new_particle_position(
    geometry_msgs::Pose& x, geometry_msgs::Vector3& v,
    const geometry_msgs::Pose& p, const geometry_msgs::Pose& g)
{
    // Random between 0 and 1
    double rp = dist_(gen_);
    double rg = dist_(gen_);

    // Update particle velocity
    v.x = omega_ * v.x + phi_p_ * rp * (p.position.x - x.position.x)
            + phi_g_ * rg * (g.position.x - x.position.x) 
            + phi_r_ * (2 * dist_(gen_) - 1);
    v.y = omega_ * v.y + phi_p_ * rp * (p.position.y - x.position.y)
            + phi_g_ * rg * (g.position.y - x.position.y)
            + phi_r_ * (2 * dist_(gen_) - 1);

    double pz = quat_to_yaw(p.orientation);
    double xz = quat_to_yaw(x.orientation);
    double gz = quat_to_yaw(g.orientation);
    v.z = omega_ * v.z + phi_p_ * rp * clamp_angle(pz - xz)
            + phi_g_ * rg * clamp_angle(gz - xz)
            + phi_r_ * (2 * dist_(gen_) - 1);

    // Limit velocity to reasonable values
    if(fabs(v.x) > 0.5)
        v.x = v.x > 0 ? 0.5 : -0.5;
    if(fabs(v.y) > 0.5)
        v.y = v.y > 0 ? 0.5 : -0.5;

    auto swarm_pos = swarm_pose_estimate_.pose.pose.position;
    swarm_pos.x = 0;
    swarm_pos.y = 0;

    // If the particle has left the map then flip the relevant velocity
    if(x.position.x + v.x + swarm_pos.x < -1)
        v.x = fabs(v.x);
    if(1 < x.position.x + v.x + swarm_pos.x)
        v.x = -fabs(v.x);
    if(x.position.y + v.y + swarm_pos.y < -1)
        v.y = fabs(v.y);
    if(1 < x.position.y + v.y + swarm_pos.y)
        v.y = -fabs(v.y);

    
    // Update particle position
    x.position.x += v.x;
    x.position.y += v.y;
    x.orientation = yaw_to_quat(xz + v.z);

    if(!map_man_->is_pose_valid(x)) {
        ROS_INFO_STREAM("Pose invalid: " << p.position);
        for(const auto& pair : robot_models_) {
            x = pair.second->gen_random_valid_pose();
            break;
        }
    }
}

bool PSOPoseEstimator::update_robot_particles(std::string robot_name) {
    // Assume we haven't improved upon our estimate
    bool improved_estimate = false;
    auto& model = robot_models_[robot_name];

    // Note that swarm orientation doesn't really mean anything
    auto swarm_pos = swarm_pose_estimate_.pose.pose.position;
    swarm_pos.x = 0;
    swarm_pos.y = 0;
    auto& cloud = robot_particle_clouds_[robot_name];
    
    // Update each particle modelling the robot
    for(int i = 0; i < cloud.particles.size(); i++) {
        auto& x = cloud.particles[i];
        auto& v = cloud.velocities[i];
        auto& p = cloud.particle_bests[i];
        auto& g = cloud.best_pose;
        auto& fg = cloud.best_frame_pose;

        // Update the particle pose
        calc_new_particle_position(x, v, p, g);

        // Calculate the pose wrt map frame (poses are stored wrt swarm
        // origin)
        auto xprime = x;
        xprime.position.x += swarm_pos.x;
        xprime.position.y += swarm_pos.y;
        auto pprime = p;
        pprime.position.x += swarm_pos.x;
        pprime.position.y += swarm_pos.y;
        auto gprime = g;
        gprime.position.x += swarm_pos.x;
        gprime.position.y += swarm_pos.y;

        // Update best particles
        if(model->weigh_pose(xprime) > model->weigh_pose(pprime)) {
            p = x;
            if(model->weigh_pose(pprime) < model->weigh_pose(fg)) {
                fg = p;
            }
            if(model->weigh_pose(pprime) < model->weigh_pose(gprime)) {
                g = p;
                improved_estimate = true;
            }
        }
    }

    return improved_estimate;
}

bool PSOPoseEstimator::update_swarm_particles() {
    return false;
    // Assume we haven't imrpoved the estimate
    bool improved_estimate = false;

    for(int i = 0; i < swarm_particle_cloud_.particles.size(); i++) {
        auto& x = swarm_particle_cloud_.particles[i];
        auto& v = swarm_particle_cloud_.velocities[i];
        auto& p = swarm_particle_cloud_.particle_bests[i];
        auto& g = swarm_particle_cloud_.best_pose;

        // Update the particle pose
        calc_new_particle_position(x, v, p, g);

        // Update best estimates
        if(weigh_swarm_pose(x) > weigh_swarm_pose(p)) {
            p = x;
            if(weigh_swarm_pose(p) > weigh_swarm_pose(g)) {
                g = p;
            }
            improved_estimate = true;
        }
    }

    return improved_estimate;
}

double PSOPoseEstimator::weigh_swarm_pose(const geometry_msgs::Pose& p) {
    // Our weight is just the sum of all robot particle weights given the
    // pose estimate
    double weight = 0;
    for(auto& pair : robot_particle_clouds_) {
        auto& pose = pair.second.best_pose;
        auto& model = robot_models_[pair.first];

        // Calculate the robot pose in the map frame
        // Note that we don't consider the swarm orientation because it
        // doesn't really mean anything
        auto map_pose = pose;
        map_pose.position.x += p.position.x;
        map_pose.position.y += p.position.y;

        // Update weight
        weight += model->weigh_pose(map_pose);
    }
    return weight;
}

void PSOPoseEstimator::update_robot_pose_estimates() {
    auto& map_pose = swarm_pose_estimate_.pose.pose;// swarm_particle_cloud_.best_pose;

    for(const auto& pair : robot_particle_clouds_) {
        const auto& name = pair.first;
        const auto& cloud = pair.second;

        robot_pose_estimates_[name].header.stamp = ros::Time::now();
        robot_pose_estimates_[name].pose = calculate_pose_with_covariance(
            cloud.best_pose, cloud.particle_bests);
        robot_pose_estimates_[name].pose.pose.position.x +=
            0;//map_pose.position.x;
        robot_pose_estimates_[name].pose.pose.position.y +=
            0;//map_pose.position.y;

        auto tf = robot_models_[name]->calculate_transform(
            robot_pose_estimates_[name].pose.pose);
        tf_broadcaster_.sendTransform(tf);
        map_man_->set_robot_pose(
            name, robot_pose_estimates_[name].pose.pose, radii_[name]);
    }
}

void PSOPoseEstimator::update_estimate_covariance() {
    SwarmPoseEstimator::update_estimate_covariance();
    return;
    swarm_pose_estimate_.header.stamp = ros::Time::now();

    geometry_msgs::PoseWithCovariance new_pose;
    new_pose.pose.position = swarm_particle_cloud_.best_pose.position;
    new_pose.pose.orientation.w = 1;

    auto& cov_mat = new_pose.covariance;
    auto& pos = new_pose.pose.position;
    double n = swarm_particle_cloud_.particle_bests.size();

    for(const auto& p : swarm_particle_cloud_.particle_bests) {
        cov_mat[0] += pow(p.position.x - pos.x, 2) / n;
        cov_mat[1] += (p.position.x - pos.x) * (p.position.y - pos.y) / n;
        cov_mat[7] += pow(p.position.y - pos.y, 2) / n;
    }
    cov_mat[6] = cov_mat[1];

    swarm_pose_estimate_.pose = new_pose;
}
