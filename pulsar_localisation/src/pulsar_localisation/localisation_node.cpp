#include <memory>
#include <thread>

#include <ros/ros.h>

#include "cloud_generator/cloud_generator.hpp"
#include "pose_estimators/swarm_pose_estimator.hpp"
#include "map_manager/map_manager.hpp"
#include "sensor_models/range_cloud_sensor_model.hpp"

/**
 * @brief Localisation main class
 *
 * Class which controls the localisation node, including the main loop and 
 * several sub classes.
 */
class LocalisationNode {
public:
    /**
     * Constructor for the main localisation node
     */
    LocalisationNode();
    /**
     * Destructor for the main localisation node
     */
    ~LocalisationNode();
private:
    /**
     * @brief Threaded main loop for the node.
     *
     * Main loop for the localisation node. Runs in a thread to process any
     * updates, while the program continues to run via ros::spin().
     */
    void loop();

    /**
     * Gets parameters from the ROS parameter server.
     */
    void get_ros_parameters();

    // Thread for the main loop
    bool running_;
    std::thread main_thread_;

    /**
     * Class to handle generation of point clouds from the robot sensor
     * streams.
     */
    std::shared_ptr<CloudGenerator> cloud_gen_;

    /**
     * Class to handle swarm pose estimates.
     */
    std::shared_ptr<SwarmPoseEstimator> pose_est_;

    /**
     * Class to handle the map of the environment.
     */
    std::shared_ptr<MapManager> map_man_;

    /* ************** *
     * ROS parameters *
     * ************** */
    // Topic that the map data is published on
    std::string map_topic_;

    /* ********************** *
     * Robot group parameters *
     * ********************** */
    // List of all the different types of robots
    std::vector<std::string> robots_;
    // Map of the number of each different robot
    std::map<std::string, int> num_robots_;
    // Map of the topics on which range sensor data is published for each
    // robot type
    std::map<std::string, std::vector<std::string>> range_sensor_topics_;
    // Map of the topics on which odometry data is published for each robot
    // type
    std::map<std::string, std::string> odometry_sensor_topics_;
    // Map of the base link frame name for each robot type
    std::map<std::string, std::string> base_link_frames_;
    // Map of initial poses of the robots
    std::map<std::string, geometry_msgs::Pose> initial_pose_estimates_;

    /* ******************** *
     * Algorithm parameters *
     * ******************** */
    // How long range data is kept for
    int cycle_sensor_readings_;

    // How many particles are kept in the filter(s) 
    // see SingleRobotPoseEstimator
    int particle_filter_size_;

    // Minimum translation required to run a filter update
    double min_trans_update_;

    // Frequency of filter updates (Hz)
    double update_rate_hz_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pulsar_localisation");
    std::shared_ptr<LocalisationNode> node(new LocalisationNode());
    ros::Rate r(100);
    ros::spin();
    // Ensure we fully cleanup before exiting
    node.reset();
    return 0;
}

LocalisationNode::LocalisationNode() :running_(true) {
    // Get parameters from the ROS parameter server
    get_ros_parameters();

    std::vector<std::string> all_topics;
    std::vector<std::string> robot_names;
    std::map<std::string, std::string> robot_odom_map;
    std::map<std::string, std::string> robot_base_link_map;
    std::map<std::string, float> robot_radii_map;
    std::stringstream ss;
    std::string s;
    // Create the full set of topics and maps for each robot
    for(auto const & pair : num_robots_) {
        for(int i = 0; i < pair.second; i++) {
            ss.clear();
            s.clear();
            ss << i;
            ss >> s;
            for(auto const & topic : range_sensor_topics_[pair.first]) {
                all_topics.push_back(pair.first+s+"/"+topic);
            }
            robot_names.push_back(pair.first+s);
            robot_base_link_map[pair.first+s] = 
                pair.first+s+"/"+base_link_frames_[pair.first];
            robot_odom_map[pair.first+s] = 
                pair.first+s + "/" + odometry_sensor_topics_[pair.first];
            robot_radii_map[pair.first+s] = 0.035;
        }
    }

    cloud_gen_.reset(new CloudGenerator(all_topics, cycle_sensor_readings_));

    map_man_.reset(new MapManager(map_topic_));

    pose_est_.reset(new SwarmPoseEstimator(
        cloud_gen_, map_man_, "map", robot_names, initial_pose_estimates_, 
        robot_odom_map, robot_base_link_map, robot_radii_map, 
        particle_filter_size_, min_trans_update_));

    main_thread_ = std::thread(&LocalisationNode::loop, this);
}

LocalisationNode::~LocalisationNode() {
    running_ = false;
    main_thread_.join();
}

#include <geometry_msgs/PoseArray.h>
void LocalisationNode::loop() {
    ros::Rate sleeper(update_rate_hz_);

    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("test", 1);
    geometry_msgs::Point p;
    geometry_msgs::PoseArray c;
    c.header.frame_id = "map";
    while(running_) {
        sleeper.sleep();
        //pose_est_->robot_pose_estimators_["pulsar_0"]->update_estimate();
        pose_est_->update_estimate();
        cloud_gen_->publish_cloud("pulsar_0");
        pose_est_->publish_pose_estimate();
        //auto& pts = pose_est_->robot_pose_estimators_["pulsar_0"]->get_pose_estimates();
        auto& pts = pose_est_->get_pose_estimates();
        c.poses.clear();
        for(auto& pair : pts) {
            c.poses.push_back(pair.second.pose.pose);
        }
        c.header.stamp = ros::Time::now();
        pub.publish(c);
    }
}

void LocalisationNode::get_ros_parameters() {
    /* ************** *
     * ROS parameters *
     * ************** */
    if(!ros::param::param<std::string>("~map_topic", map_topic_, "map")) {
        ROS_WARN_STREAM(
            "Failed to get param 'map_topic'. Defaulting to '" << map_topic_
            << "'.");
    }
    /* ********************** *
     * Robot group parameters *
     * ********************** */
    std::string s1;
    std::stringstream ss;
    std::string robot_prefix;
    int i = 0;
    ss << i;
    ss >> s1;
    while(true) {
        if(!ros::param::get("~robot"+s1, robot_prefix)) {
            if(i == 0) {
                ROS_ERROR_STREAM(
                    "Failed to get any params 'robotX' (with X "
                    << "starting at 0). No robot data will be read..."
                    << "\n\n PLEASE NOTE, THIS PARAM SHOULD BE THE TF "
                    << "PREFIX OF THE ROBOTS IN THIS GROUP "
                    << "(e.g. 'pulsar_' for a group of robots with "
                    << "tf_prefixes of 'pulsar_0', 'pulsar_1' ...)!!\n");
            }
            else {
                ROS_INFO_STREAM("Got " << i << " different robot types.");
            }
            break;
        }
        robots_.push_back(robot_prefix);
        if(!ros::param::param<int>(
            "~robot"+s1+"_quantity", num_robots_[robot_prefix], 0))
        {
            ROS_ERROR_STREAM(
                "Failed to get param '" << "robot" << s1 << "_quantity"
                << "'. Defaulting to 0. No robots of this type will be "
                << "included in the localisation.");
        }
        if(!ros::param::param<std::string>(
            "~robot"+s1+"_odom_topic", 
            odometry_sensor_topics_[robot_prefix], "odometry/filtered"))
        {
            ROS_WARN_STREAM(
                "Failed to get param '" << "robot" << s1 << "_odom_topic"
                << "'. Defaulting to '"
                << odometry_sensor_topics_[robot_prefix] 
                << "'. No robots of this type will be "
                << "included in the localisation.");
        }
        if(!ros::param::param<std::string>(
            "~robot"+s1+"_base_link_frame",
            base_link_frames_[robot_prefix], "base_link"))
        {
            ROS_WARN_STREAM(
                "Failed to get param 'robot" << s1 << "_base_link_frame"
                << "'. Defaulting to '" << base_link_frames_[robot_prefix]
                << "'.");
        }
        int j = 0;
        std::string s2;
        std::string topic_name;
        ss.clear();
        ss << j;
        ss >> s2;
        while(true) {
            if(!ros::param::get(
                "~robot"+s1+"_range_topic"+s2, topic_name))
            {
                if(j == 0) {
                    ROS_ERROR_STREAM(
                        "Failed to get any params 'robot" << s1
                        << "_range_topicX' (with X starting at 0). No "
                        << "range data will be read for these robots...");
                }
                else {
                    ROS_INFO_STREAM(
                        "Got " << j << " range sensor topics for robot '"
                        << robot_prefix << "'.");
                }
                break;
            }
            range_sensor_topics_[robot_prefix].push_back(topic_name);
            s2.clear();
            ss.clear();
            ss << ++j;
            ss >> s2;
        }
        
        // This parameter is expected to be set through whatever script
        // spawns all the robots
        std::vector<float> initial_pose; 
        std::string r;
        for(j = 0; j < num_robots_[robot_prefix]; j++) {
            r.clear();
            ss.clear();
            ss << j;
            ss >> r;
            r = robot_prefix+r;
            ros::param::get("/"+r+"/initial_pose", initial_pose);
            if(initial_pose.size() < 7) {
                ROS_WARN_STREAM(
                    "Only received " << initial_pose.size()
                    << " initial poses in the parameter '/" << r
                    << "/initial_pose' (7 expected). Remaining values will "
                    << "default to identity transforms.");
            }
            else if(initial_pose.size() > 7) {
                ROS_WARN_STREAM(
                    "Received "<< initial_pose.size()
                    << " initial poses in the parameter '/" << r
                    << "/initial_pose' (7 expected). Excess values will be "
                    << "ignored.");
            }
            if(initial_pose.size() > 0)
                initial_pose_estimates_[r].position.x = initial_pose[0];
            else
                initial_pose_estimates_[r].position.x = 0;
            if(initial_pose.size() > 1)
                initial_pose_estimates_[r].position.y = initial_pose[1];
            else
                initial_pose_estimates_[r].position.y = 0;
            if(initial_pose.size() > 2)
                initial_pose_estimates_[r].position.z = initial_pose[2];
            else
            if(initial_pose.size() > 3)
                initial_pose_estimates_[r].orientation.x = initial_pose[3];
            else
                initial_pose_estimates_[r].orientation.x = 0;
            if(initial_pose.size() > 4)
                initial_pose_estimates_[r].orientation.y = initial_pose[4];
            else
                initial_pose_estimates_[r].orientation.y = 0;
            if(initial_pose.size() > 5)
                initial_pose_estimates_[r].orientation.z = initial_pose[5];
            else
                initial_pose_estimates_[r].orientation.z = 0;
            if(initial_pose.size() > 6)
                initial_pose_estimates_[r].orientation.w = initial_pose[6];
            else
                initial_pose_estimates_[r].orientation.w = 1;
        }
        
        s1.clear();
        ss.clear();
        ss << ++i;
        ss >> s1;
    }

    /* ******************** *
     * Algorithm parameters *
     * ******************** */
    if(!ros::param::param<int>(
        "~cycle_sensor_readings", cycle_sensor_readings_, 6)) 
    {
        ROS_WARN_STREAM(
           "Failed to get param 'cycle_sensor_readings'. Defaulting to: "
           << cycle_sensor_readings_ << ".");
    }
    RangeCloudSensorModel::cycle_sensor_readings_ = cycle_sensor_readings_;

    if(!ros::param::param<float>(
        "~time_resolution", RangeCloudSensorModel::time_resolution_, 1e-2)) 
    {
        ROS_WARN_STREAM(
           "Failed to get param 'time_resolution'. Defaulting to: "
           << RangeCloudSensorModel::time_resolution_ << ".");
    }

    if(!ros::param::param<double>(
        "~a1", SingleRobotPoseEstimator::a1_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'a1'. Defaulting to '"
            << SingleRobotPoseEstimator::a1_ << "'.");
    }

    if(!ros::param::param<double>(
        "~a2", SingleRobotPoseEstimator::a2_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'a2'. Defaulting to '"
            << SingleRobotPoseEstimator::a2_ << "'.");
    }

    if(!ros::param::param<double>(
        "~a3", SingleRobotPoseEstimator::a3_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'a3'. Defaulting to '"
            << SingleRobotPoseEstimator::a3_ << "'.");
    }

    if(!ros::param::param<double>(
        "~a4", SingleRobotPoseEstimator::a4_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'a4'. Defaulting to '"
            << SingleRobotPoseEstimator::a4_ << "'.");
    }

    if(!ros::param::param<double>(
        "~aslow", SingleRobotPoseEstimator::aslow_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'aslow'. Defaulting to '"
            << SingleRobotPoseEstimator::aslow_ << "'.");
    }

    if(!ros::param::param<double>(
        "~afast", SingleRobotPoseEstimator::afast_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'afast'. Defaulting to '"
            << SingleRobotPoseEstimator::afast_ << "'.");
    }

    if(!ros::param::param<double>(
        "~lamshort", RangeCloudSensorModel::lamshort_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'lamshort'. Defaulting to '"
            << RangeCloudSensorModel::lamshort_ << "'.");
    }

    if(!ros::param::param<double>(
        "~sigmahit", RangeCloudSensorModel::sigmahit_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'sigmahit'. Defaulting to '"
            << RangeCloudSensorModel::sigmahit_ << "'.");
    }

    if(!ros::param::param<double>(
        "~ztime", RangeCloudSensorModel::ztime_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'ztime'. Defaulting to '"
            << RangeCloudSensorModel::ztime_ << "'.");
    }

    if(!ros::param::param<double>(
        "~zhit", RangeCloudSensorModel::zhit_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'zhit'. Defaulting to '"
            << RangeCloudSensorModel::zhit_ << "'.");
    }

    if(!ros::param::param<double>(
        "~zshort", RangeCloudSensorModel::zshort_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'zshort'. Defaulting to '"
            << RangeCloudSensorModel::zshort_ << "'.");
    }

    if(!ros::param::param<double>(
        "~zmax", RangeCloudSensorModel::zmax_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'zmax'. Defaulting to '"
            << RangeCloudSensorModel::zmax_ << "'.");
    }

    if(!ros::param::param<double>(
        "~zrand", RangeCloudSensorModel::zrand_, 0.2))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'zrand'. Defaulting to '"
            << RangeCloudSensorModel::zrand_ << "'.");
    }

    if(!ros::param::param<int>(
        "~num_particles", particle_filter_size_, 50))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'num_particles'. Defaulting to '"
            << particle_filter_size_ << "'.");
    }

    if(!ros::param::param<int>(
        "~dbscan_min_points", SingleRobotPoseEstimator::dbscan_min_points_,
        5))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'dbscan_min_points'. Defaulting to '"
            << SingleRobotPoseEstimator::dbscan_min_points_ << "'.");
    }

    if(!ros::param::param<double>(
        "~dbscan_epsilon", SingleRobotPoseEstimator::dbscan_epsilon_,
        0.05*0.05))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'dbscan_epsilon'. Defaulting to '"
            << SingleRobotPoseEstimator::dbscan_epsilon_ << "'.");
    }

    if(!ros::param::param<double>(
        "~min_trans_update", min_trans_update_, 0.05))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'min_trans_update'. Defaulting to '"
            << min_trans_update_ << "'.");
    }

    if(!ros::param::param<double>(
        "~update_rate", update_rate_hz_, 15))
    {
        ROS_WARN_STREAM(
            "Failed to get param 'update_rate'. Defaulting to '"
            << update_rate_hz_ << "'.");
    }
}
