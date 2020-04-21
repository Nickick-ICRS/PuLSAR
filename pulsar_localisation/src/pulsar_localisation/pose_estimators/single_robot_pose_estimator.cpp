#include "pose_estimators/single_robot_pose_estimator.hpp"

#include "maths/useful_functions.hpp"
#include "maths/dbscan.hpp"

#include <cmath>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double SingleRobotPoseEstimator::aslow_, 
       SingleRobotPoseEstimator::afast_,
       SingleRobotPoseEstimator::dbscan_epsilon_;

int    SingleRobotPoseEstimator::dbscan_min_points_;

SingleRobotPoseEstimator::SingleRobotPoseEstimator(
    std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame, 
    geometry_msgs::Pose initial_pose, std::string odom_topic,
    std::string base_link_frame, float radius, unsigned int M,
    double min_trans_update)
    
    :RobotModel(name, cloud_gen, map_man, map_frame, odom_topic, 
    base_link_frame, radius, min_trans_update), tf_listener_(tf2_)
{
    pose_estimate_.pose.pose = initial_pose;
    pose_estimate_.header.frame_id = map_frame;
    pose_estimate_.header.stamp = ros::Time::now();

    pose_estimate_.pose.covariance[0]  = 1e-6;
    pose_estimate_.pose.covariance[7]  = 1e-6;
    pose_estimate_.pose.covariance[35] = 1e-6;

    map_man_->set_robot_pose(name, initial_pose, radius);

    for(int i = 0; i < M; i++) {
        geometry_msgs::Pose p = gen_random_valid_pose(pose_estimate_.pose);
        pose_estimate_cloud_.push_back(p);
    }
}

SingleRobotPoseEstimator::~SingleRobotPoseEstimator() {
    // dtor
}

void SingleRobotPoseEstimator::update_estimate() {
    // Get the most recent odometry message without blocking for too long
    nav_msgs::Odometry odom;
    {
        std::lock_guard<std::mutex> lock(odom_mut_);
        odom = recent_odom_;
    }
    // Clean any old data points
    cloud_gen_->clean_cloud(name_);
    // Run an iteration of the augmented MCL filter
    auto new_cloud = augmented_MCL(pose_estimate_cloud_, odom);
    pose_estimate_cloud_ = new_cloud;

    // Find the mean and covariance of the cloud and store in the overall 
    // pose estimate
    update_pose_estimate_with_covariance();
    auto tf = calculate_transform(odom);
    tf_broadcaster_.sendTransform(tf);
}

void SingleRobotPoseEstimator::update_pose_estimate_with_covariance() {
    
    std::vector<dbscan::Point> dbscan_points;

    for(auto& pose : pose_estimate_cloud_) {
        dbscan::Point p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = pose.position.z;
        p.clusterID = UNCLASSIFIED;
        p.pose = &pose;
        dbscan_points.push_back(p);
    }

    dbscan::DBSCAN db(dbscan_min_points_, dbscan_epsilon_, dbscan_points);
    db.run();

    std::map<int, std::vector<geometry_msgs::Pose>> clustered_poses;
    int kept = 0;
    for(const auto& p : db.m_points) {
        if(p.clusterID != NOISE && p.clusterID != UNCLASSIFIED) {
            clustered_poses[p.clusterID].push_back(*p.pose);
            kept++;
        }
    }
    
    if(!kept) {
        ROS_WARN_STREAM_THROTTLE(2, "Failed to group any pose estimates.");
        return;
    }

    int best_it = 0;
    int max_size = 0;

    for(const auto& pair : clustered_poses) {
        if(pair.second.size() > max_size) {
            max_size = pair.second.size();
            best_it = pair.first;
        }
    }

    auto& best_cluster = clustered_poses[best_it];

    geometry_msgs::Point avg_point;
    double avg_yaw;

    // First calculate means
    for(const auto& pose : best_cluster) {
        avg_point.x += pose.position.x;
        avg_point.y += pose.position.y;
        avg_point.z += pose.position.z;
        avg_yaw += quat_to_yaw(pose.orientation);
    }
    double n = best_cluster.size();
    avg_point.x /= n;
    avg_point.y /= n;
    avg_point.z /= n;

    avg_yaw /= n;
    while(avg_yaw <= -M_PI) avg_yaw += 2*M_PI;

    // Helper lambda for calculating the difference between two angles
    static auto ang_diff = [](double a, double b) {
        double diff = a-b;
        if(diff > M_PI/2)   diff -= M_PI;
        if(diff <= -M_PI/2) diff += M_PI;
        return diff;
    };

    // Now calculate the covariance matrix
    auto& cov_mat = pose_estimate_.pose.covariance;
    // Clear previous covariance values
    for(int i = 0; i < 36; i++)
        cov_mat[i] = 0;
    // Sample covariance so reduce n by 1
    n -= 1;
    for(const auto& pose : best_cluster) {
        double yaw_diff = ang_diff(quat_to_yaw(pose.orientation), avg_yaw);
        cov_mat[0]  += pow(pose.position.x - avg_point.x, 2) / n;
        cov_mat[1]  += (pose.position.x - avg_point.x) 
                     * (pose.position.y - avg_point.y) / n;
        cov_mat[5]  += (pose.position.x - avg_point.x) * yaw_diff / n;
        cov_mat[7]  += pow(pose.position.y - avg_point.y, 2) / n;
        cov_mat[11] += (pose.position.y - avg_point.y) * yaw_diff / n;
        cov_mat[35] += pow(yaw_diff, 2) / n;
    }
    
    // Make the matrix symmetric
    cov_mat[6]  = cov_mat[1];  // y-x , x-y 
    cov_mat[30] = cov_mat[5];  // th-x, x-th
    cov_mat[31] = cov_mat[11]; // th-y, y-th

    pose_estimate_.header.stamp = ros::Time::now();
    pose_estimate_.pose.pose.position = avg_point;
    pose_estimate_.pose.pose.orientation = yaw_to_quat(avg_yaw);
}

// TODO: Consider trying KLD_Sampling_MCL from Probabilistic Robotics
std::vector<geometry_msgs::Pose> 
    SingleRobotPoseEstimator::augmented_MCL(
        const std::vector<geometry_msgs::Pose>& Xt_1,
        const nav_msgs::Odometry& ut)
{
    static double wslow = 0;
    static double wfast = 0;

    // New set of poses
    std::vector<geometry_msgs::Pose> Xt;

    // Pairs of poses with weights
    std::vector<std::pair<geometry_msgs::Pose, float>> Xtbar;

    // Motion model estimates
    std::vector<geometry_msgs::Pose> xt;
    // Measurement model weights
    std::vector<float> wt;

    float wavg = 0;
    float wtotal = 0;
    double avg_yaw;
    int M = Xt_1.size();

    // Transform the odometry message into the most recent map frame
    // estimate
    geometry_msgs::Pose utp, utp_1;
    utp = ut.pose.pose;
    utp_1 = prev_odom_.pose.pose;
    prev_odom_ = ut;

    const auto Z = cloud_gen_->get_raw_data(name_);

    for(int m = 0; m < M; m++) {
        // Update the motion model
        xt.push_back(sample_motion_model_odometry(utp, utp_1, Xt_1[m]));
        // Update the measurement model
        wt.push_back(range_model_.model(xt[m], Z, name_, base_link_frame_));
        wtotal += wt[m];
        // Update the cloud + weights
        Xtbar.emplace_back(xt[m], wt[m]);
        avg_yaw += quat_to_yaw(xt.back().orientation);
    }
    avg_yaw /= M;
    while(avg_yaw > M_PI)  avg_yaw -= 2*M_PI;
    while(avg_yaw < -M_PI) avg_yaw += 2*M_PI;

    wavg = wtotal / M;

    if(wavg < 1e-06 or std::isnan(wavg)) wavg = 1e-06;

    wslow = wslow + aslow_ * (wavg - wslow);
    wfast = wfast + afast_ * (wavg - wfast);
    if(wfast < 1e-06) wfast = 1e-06;
    if(wslow < 1e-06) wslow = 1e-06;

    static std::uniform_real_distribution<double> dist(0, 1);
    for(int m = 0; m < M; m++) {
        double p = 0.0 - wfast/wslow;
        if(p < 0) p = 0;
        if(dist(gen_) < p) {
            // Add a random pose to Xt - not distributed about our current
            // estimate as this is to try and ensure we don't hit local
            // minima. As we have good angular odometry we only want
            // a random x and y
            Xt.emplace_back(gen_random_valid_pose_position(avg_yaw));
        }
        else {
            // Place a random point from the cloud into the new one, based
            // on its weight
            // While loop just incase somehow we didn't select an item.
            double p2 = 1;
            p2 = dist(gen_) * wtotal;
            for(const auto& pair : Xtbar) {
                p2 -= pair.second;
                if(p2 <= 0) {
                    try {
                        Xt.emplace_back(map_man_->make_pose_valid(
                            pair.first, radius_));
                    }
                    catch(PoseInvalidException) {
                        Xt.emplace_back(
                            gen_random_valid_pose_position(avg_yaw));
                    }
                    break;
                }
            }
            if(p2 > 0) {
                Xt.emplace_back(map_man_->make_pose_valid(
                    Xtbar.back().first, radius_));
            }
        }
    }

    return Xt;
}

geometry_msgs::TransformStamped 
    SingleRobotPoseEstimator::calculate_transform(
        const nav_msgs::Odometry& odom)
{
    double mRo = quat_to_yaw(pose_estimate_.pose.pose.orientation)
               - quat_to_yaw(odom.pose.pose.orientation);

    // Create the message and return
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom.header.frame_id;
    double oTbx = odom.pose.pose.position.x * cos(mRo)
                - odom.pose.pose.position.y * sin(mRo);
    double oTby = odom.pose.pose.position.y * cos(mRo)
                + odom.pose.pose.position.x * sin(mRo);

    tf.transform.translation.x =
        pose_estimate_.pose.pose.position.x - oTbx;//odom.pose.pose.position.x;
    tf.transform.translation.y =
        pose_estimate_.pose.pose.position.y - oTby;//odom.pose.pose.position.y;
//    tf.transform.translation.z =
//        pose_estimate_.pose.pose.position.z - odom.pose.pose.position.z;
    tf.transform.rotation = yaw_to_quat(mRo);
    return tf;
}
