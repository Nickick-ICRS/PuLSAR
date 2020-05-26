#ifndef __MCL_SINGLE_ROBOT_POSE_ESTIMATOR__
#define __MCL_SINGLE_ROBOT_POSE_ESTIMATOR__

#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pose_estimators/single_robot_pose_estimator.hpp"
#include "cloud_generator/cloud_generator.hpp"
#include "sensor_models/range_cloud_sensor_model.hpp"
#include "map_manager/map_manager.hpp"
#include "maths/useful_functions.hpp"
#include "maths/dbscan.hpp"

namespace MCLSingleRobotPoseEstimatorConstants {
    // Exponential parameters for Augmented MCL, 0 <= aslow < afast
    static double aslow_, afast_;

    // Parameters for DBSCAN
    static int dbscan_min_points_;
    static double dbscan_epsilon_;
}


/**
 * @brief Estimates the pose of a single robot.
 *
 * Class to estimate the pose of a single robot within a swarm of robots.
 * Uses laser scan and odometry information along with knowledge of the map
 * environment to estimate the pose of the robot.
 */

template<class T>
class MCLSingleRobotPoseEstimator :public SingleRobotPoseEstimator<T>{
public:
    /**
     * Constructor for the MCLSingleRobotPoseEstimator class.
     *
     * @param name The name of the robot (as used by cloud_gen).
     *
     * @param cloud_gen Pointer to an initialised cloud generator class
     *                  instance.
     *
     * @param map_man Pointer to an initialised map manager class instance.
     *
     * @param initial_pose An initial estimate of the robot's pose relative
     *                     to the map frame.
     *
     * @param odom_topic The topic on which odometry measurements from the
     *                   robot are published.
     *
     * @param base_link_frame The name of the base_link frame of the robot.
     *
     * @param radius Approximate radius of the robot footprint.
     *
     * @param M Number of particles to include in the filter.
     *
     * @param min_trans_update Minimum distance to travel before running a
     *                         filter update.
     */
    MCLSingleRobotPoseEstimator(
        std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
        const std::shared_ptr<MapManager>& map_man, std::string map_frame, 
        geometry_msgs::Pose initial_pose, std::string odom_topic,
        std::string base_link_frame, float radius, unsigned int M,
        double min_trans_update)
    
        :SingleRobotPoseEstimator<T>(name, cloud_gen, map_man, map_frame, 
        odom_topic, base_link_frame, radius, min_trans_update,
        initial_pose), tf_listener_(tf2_)
    {
        for(int i = 0; i < M; i++) {
            geometry_msgs::Pose p =
                this->gen_random_valid_pose(this->pose_estimate_.pose);
            pose_estimate_cloud_.push_back(p);
        }
    };
    ~MCLSingleRobotPoseEstimator() {
        // dtor
    };

    /**
     * @brief Updates the pose estimate of the robot.
     *
     * Updates the pose estimate of the robot by running the augmented MCL
     * particle filter and publishes to tf.
     */
    void update_estimate() {
        // Get the most recent odometry message without blocking for too
        // long
        nav_msgs::Odometry odom;
        {
            std::lock_guard<std::mutex> lock(this->odom_mut_);
            odom = this->recent_odom_;
        }
        // Clean any old data points
        this->cloud_gen_->clean_cloud(this->name_);
        // Run an iteration of the augmented MCL filter
        auto new_cloud = augmented_MCL(pose_estimate_cloud_, odom);
        pose_estimate_cloud_ = new_cloud;

        // Find the mean and covariance of the cloud and store in the
        // overall pose estimate
        update_pose_estimate_with_covariance();
        auto tf = this->calculate_transform(this->pose_estimate_.pose.pose);
        tf_broadcaster_.sendTransform(tf);

        // Set the pose in the map
        this->map_man_->set_robot_pose(
            this->name_, this->pose_estimate_.pose.pose, this->radius_);
    };

    /**
     * Get all of the current pose estimates. Useful for debugging.
     */
    std::vector<geometry_msgs::Pose>& get_pose_estimates() {
        return pose_estimate_cloud_;
    };

private:
    /**
     * @brief Updates the overall pose estimate and covariance.
     *
     * Uses the current cloud of pose estimate points to calculate the
     * average pose estimate and covariance of the estimate.
     */
    void update_pose_estimate_with_covariance() {
        using namespace MCLSingleRobotPoseEstimatorConstants;

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

        dbscan::DBSCAN db(
            dbscan_min_points_, dbscan_epsilon_, dbscan_points);
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
            ROS_WARN_STREAM_THROTTLE(
                2, "Failed to group any pose estimates.");
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

        const auto& best_cluster = clustered_poses[best_it];

        this->pose_estimate_.header.stamp = ros::Time::now();
        this->pose_estimate_.pose =
            calculate_pose_with_covariance(best_cluster);
    };

    /**
     * @brief Runs an iteration of the augmented MCL algorithm.
     *
     * Runs one iteration of the augmented Monte Carlo Localisation
     * algorithm, finding a cloud of points all of which may be the pose
     * of this robot. Robust against local minima due to additions of
     * random additional poses. See Probabilistic Robotics by Thrun et al.
     *
     * @param Xt_1 The previous set of pose estimates.
     *
     * @param ut The most recent odometry measurements.
     *
     * @return The new (calculated) set of pose estimates.
     */
    std::vector<geometry_msgs::Pose> augmented_MCL(
        const std::vector<geometry_msgs::Pose>& Xt_1,
        const nav_msgs::Odometry& ut)
    {
        using namespace MCLSingleRobotPoseEstimatorConstants;
        
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
        utp_1 = this->prev_odom_.pose.pose;
        this->prev_odom_ = ut;

        const auto Z = this->cloud_gen_->get_raw_data(this->name_);

        for(int m = 0; m < M; m++) {
            // Update the motion model
            xt.push_back(
                this->sample_motion_model_odometry(utp, utp_1, Xt_1[m]));
            // Update the measurement model
            wt.push_back(
                this->range_model_.model(
                    xt[m], Z, this->name_, this->base_link_frame_));
            wtotal += wt[m];
            // Update the cloud + weights
            Xtbar.emplace_back(xt[m], wt[m]);
            double yaw = quat_to_yaw(xt.back().orientation);
            if(std::isfinite(yaw))
                avg_yaw += quat_to_yaw(xt.back().orientation);
            else {
                ROS_WARN_STREAM(
                    "Ignoring non-finite yaw in estimate cloud");
            }
        }
        avg_yaw /= M;
        if(!std::isfinite(avg_yaw)) {
            ROS_WARN_STREAM(
                "Non-finite average yaw in MCL: " << avg_yaw 
                << "... Skipping update of " << this->name_ << ". M: "
                << M);
            return Xt_1;
        }
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
            if(dist(this->gen_) < p) {
                // Add a random pose to Xt - not distributed about our
                // current estimate as this is to try and ensure we don't
                // hit local minima. As we have good angular odometry we
                // only want a random x and y
                Xt.emplace_back(
                    this->gen_random_valid_pose_position(avg_yaw));
            }
            else {
                // Place a random point from the cloud into the new one,
                // based on its weight
                // While loop just incase somehow we didn't select an item.
                double p2 = 1;
                p2 = dist(this->gen_) * wtotal;
                for(const auto& pair : Xtbar) {
                    p2 -= pair.second;
                    if(p2 <= 0) {
                        try {
                            Xt.emplace_back(this->map_man_->make_pose_valid(
                                pair.first, this->radius_));
                        }
                        catch(PoseInvalidException) {
                            Xt.emplace_back(
                                this->gen_random_valid_pose_position(
                                    avg_yaw));
                        }
                        break;
                    }
                }
                if(p2 > 0) {
                    Xt.emplace_back(this->map_man_->make_pose_valid(
                        Xtbar.back().first, this->radius_));
                }
            }
        }

        if(Xt.size() == 0) {
            ROS_WARN_STREAM(
                this->name_ << ": Xt was size 0, skipping update...");
            Xt = Xt_1;
        }

        return Xt;
    };

    std::vector<geometry_msgs::Pose> pose_estimate_cloud_;

    tf2_ros::Buffer tf2_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Saves us a lot of constructor parameters by having the above params
    // be static and making this class able to access them.
    friend class LocalisationNode;
};

#endif // __MCL_SINGLE_ROBOT_POSE_ESTIMATOR__
