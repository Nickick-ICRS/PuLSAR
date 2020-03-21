#include <memory>
#include <thread>

#include <ros/ros.h>

#include "cloud_generator.hpp"

class LocalisationNode {
public:
    LocalisationNode();
    ~LocalisationNode();
private:
    void loop();

    bool running_;
    std::thread main_thread_;
    std::shared_ptr<CloudGenerator> cloud_gen_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pulsar_localisation");
    LocalisationNode node;
    ros::spin();
    return 0;
};

LocalisationNode::LocalisationNode() :running_(true) {
    std::vector<std::string> range_topic_names = {
        "pulsar_0/left_range_finder/range",
        "pulsar_0/middle_range_finder/range",
        "pulsar_0/right_range_finder/range"
    };
    cloud_gen_.reset(new CloudGenerator(range_topic_names, 10));

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
    }
}
