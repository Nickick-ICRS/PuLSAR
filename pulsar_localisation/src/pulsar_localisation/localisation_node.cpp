#include <memory>
#include <thread>

#include <ros/ros.h>

#include "cloud_generator/cloud_generator.hpp"
#include "pose_estimators/swarm_pose_estimator.hpp"

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
    // Map of initial poses of the robots
    std::map<std::string, geometry_msgs::Pose> initial_pose_estimates_;

    /* ******************** *
     * Algorithm parameters *
     * ******************** */
     // How long range data is kept for
     float history_length_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pulsar_localisation");
    std::shared_ptr<LocalisationNode> node(new LocalisationNode());
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
    std::stringstream ss;
    std::string s;
    // Create the full set of topics for each robot
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
            robot_odom_map[pair.first+s] = 
                pair.first+s + "/" + odometry_sensor_topics_[pair.first];
        }
    }

    cloud_gen_.reset(new CloudGenerator(all_topics, history_length_));

    pose_est_.reset(new SwarmPoseEstimator(
        cloud_gen_, "map", robot_names, initial_pose_estimates_, 
        robot_odom_map));

    main_thread_ = std::thread(&LocalisationNode::loop, this);
}

LocalisationNode::~LocalisationNode() {
    running_ = false;
    main_thread_.join();
}

void LocalisationNode::loop() {
    ros::Rate sleeper(100);
    while(running_) {
        cloud_gen_->clean_all_clouds();
        sleeper.sleep();

        cloud_gen_->publish_cloud("pulsar_0");
        cloud_gen_->publish_cloud("pulsar_1");
        cloud_gen_->publish_cloud("pulsar_2");
        cloud_gen_->publish_cloud("pulsar_3");
        cloud_gen_->publish_cloud("pulsar_4");
        cloud_gen_->publish_cloud("pulsar_5");
        cloud_gen_->publish_cloud("pulsar_6");
        cloud_gen_->publish_cloud("pulsar_7");
        cloud_gen_->publish_cloud("pulsar_8");
        cloud_gen_->publish_cloud("pulsar_9");
    }
}

void LocalisationNode::get_ros_parameters() {
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
            ROS_INFO_STREAM(
                "Got initial pose of :\n" << initial_pose_estimates_[r]
                << " for " << r);
        }
        
        s1.clear();
        ss.clear();
        ss << ++i;
        ss >> s1;
    }

    /* ******************** *
     * Algorithm parameters *
     * ******************** */
     if(!ros::param::param<float>("~history_length", history_length_, 5)) {
         ROS_WARN_STREAM(
            "Failed to get param 'history_length'. Defaulting to: "
            << history_length_ << ".");
     }
}
