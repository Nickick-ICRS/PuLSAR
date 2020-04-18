#include "cloud_generator/cloud_generator.hpp"

CloudGenerator::CloudGenerator(
    std::vector<std::string> range_topic_names, float cycle_sensor_readings,
    std::string odom_name) 
        :private_nh_("~"), odom_name_(odom_name),
        tf_buffer_(ros::Duration(cycle_sensor_readings)), tf2_(tf_buffer_), 
        cycle_sensor_readings_(cycle_sensor_readings)
{
    for(std::string name : range_topic_names) {
        range_subs_.emplace_back(
            nh_.subscribe(name, 1, &CloudGenerator::range_cb, this));
    }
}

CloudGenerator::~CloudGenerator() {
    // dtor
}

void CloudGenerator::publish_cloud(std::string name) {
    std::vector<sensor_msgs::Range> *data;
    try {
        data = &robot_raw_data_.at(name);
    }
    catch(std::out_of_range) {
        ROS_WARN_STREAM_DELAYED_THROTTLE(5, 
            "[CloudGenerator]: No robots with name '" << name 
            << "' have data stored within the cloud generator.");
        return;
    }
    
    ros::Publisher *pub;
    try {
        pub = &cloud_pubs_.at(name);
    }
    catch(std::out_of_range) {
        cloud_pubs_[name] = private_nh_.advertise<sensor_msgs::PointCloud>(
            name + "_point_cloud", 1);
        pub = &cloud_pubs_.at(name);
    }
    sensor_msgs::PointCloud cloud;
    std::string odom = name + '/' + odom_name_;
    for(const auto& z : *data) {
        geometry_msgs::PointStamped p;
        geometry_msgs::Point32 p32;
        p.point.x = z.range;
        p.header = z.header;
        try {
            tf_buffer_.transform(p, p, odom);
        }
        catch(tf2::TransformException &ex) {
            ROS_WARN("Failed to transform point: %s\n", ex.what());
            continue;
        }
        p32.x = p.point.x; p32.y = p.point.y; p32.z = p.point.z;
        cloud.points.push_back(p32);
    }
    cloud.header.frame_id = odom;
    cloud.header.stamp = ros::Time::now();
    pub->publish(cloud);
}

const std::vector<sensor_msgs::Range>& CloudGenerator::get_raw_data(
        std::string robot_name)
{
    std::lock_guard<std::recursive_mutex> lock(
        robot_cloud_muts_[robot_name]);
    clean_cloud(robot_name);
    return robot_raw_data_[robot_name];
}

void CloudGenerator::clean_all_clouds() {
    for(auto & pair : robot_raw_data_) {
        std::lock_guard<std::recursive_mutex> lock(
            robot_cloud_muts_[pair.first]);
        clean_cloud(pair.second);
    }
}

void CloudGenerator::clean_cloud(std::string name) {
    std::lock_guard<std::recursive_mutex> lock(robot_cloud_muts_[name]);
    std::vector<sensor_msgs::Range>& range = robot_raw_data_[name];
    clean_cloud(range);
}

void CloudGenerator::clean_cloud(std::vector<sensor_msgs::Range>& cloud) {
    if(cloud.size() <= cycle_sensor_readings_)
        return;
    // Remove the points beyond the most recent cycle_sensor_readings_
    int num = cycle_sensor_readings_;
    for(auto it = cloud.begin(); it != cloud.end(); it++) {
        if(it + num != cloud.end())
            it = --cloud.erase(it);
        else
            return;
    }
}

void CloudGenerator::range_cb(const sensor_msgs::RangeConstPtr& msg) {
    // Find the tf_prefix so we know which point cloud to put this in
    std::stringstream ss;
    ss << msg->header.frame_id;
    std::string prefix;
    std::string temp;
    std::string temp2;
    int i = 0;
    while(std::getline(ss, temp, '/')) {
        prefix += temp2 + '/';
        temp2 = temp;
    }
    if(prefix.empty()) {
        ROS_WARN_STREAM("Received range message with no tf_prefix...");
        prefix = "";
    }
    else {
        prefix.pop_back();
        if(!prefix.empty() && prefix[0] == '/')
            prefix.erase(prefix.begin());
    }
    std::lock_guard<std::recursive_mutex> lock(robot_cloud_muts_[prefix]);
    robot_raw_data_[prefix].push_back(*msg);
}
