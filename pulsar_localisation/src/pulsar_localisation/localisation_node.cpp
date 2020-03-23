#include <memory>
#include <thread>

#include <ros/ros.h>

#include "cloud_generator.hpp"

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
    // type (TODO: Decide if we need this)
    // std::map<std::string, std::string> odometry_sensor_topics_;

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
    std::stringstream ss;
    std::string s;
    for(auto const & pair : num_robots_) {
        for(int i = 0; i < pair.second; i++) {
            ss.clear();
            s.clear();
            ss << i;
            ss >> s;
            for(auto const & topic : range_sensor_topics_[pair.first]) {
                all_topics.push_back(pair.first+s+"/"+topic);
            }
        }
    }

    cloud_gen_.reset(new CloudGenerator(all_topics, history_length_));

    main_thread_ = std::thread(&LocalisationNode::loop, this);
}

LocalisationNode::~LocalisationNode() {
    running_ = false;
    main_thread_.join();
}

void LocalisationNode::loop() {
    while(running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    std::string robot;
    int i = 0;
    ss << i;
    ss >> s1;
    while(true) {
        if(!ros::param::get("~robot"+s1, robot)) {
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
        robots_.push_back(robot);
        if(!ros::param::param<int>(
            "~robot"+s1+"_quantity", num_robots_[robot], 0))
        {
            ROS_ERROR_STREAM(
                "Failed to get param '" << "robot" << s1 << "_quantity"
                << "'. Defaulting to 0. No robots of this type will be "
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
                        << robot << "'.");
                }
                break;
            }
            range_sensor_topics_[robot].push_back(topic_name);
            s2.clear();
            ss.clear();
            ss << ++j;
            ss >> s2;
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
