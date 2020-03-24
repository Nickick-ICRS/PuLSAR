#include "cloud_generator.hpp"

CloudGenerator::CloudGenerator(
    std::vector<std::string> range_topic_names, float history_length,
    std::string odom_name) 
        :private_nh_("~"), odom_name_(odom_name), tf2_(tf_buffer_),
        history_length_(history_length)
{
    for(std::string name : range_topic_names) {
        range_subs_.emplace_back(
            nh_.subscribe(name, 1, &CloudGenerator::range_cb, this));
    }

    full_cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
        "full_point_cloud", 1);
}

CloudGenerator::~CloudGenerator() {
    // dtor
}

void CloudGenerator::publish_cloud(std::string name) {
    // Initialise reference with a cloud we know exists
    pcl::PointCloud<pcl::PointXYZI>& cloud = full_cloud_;
    std::lock_guard<std::recursive_mutex> lock(robot_cloud_muts_[name]);
    try {
        cloud = robot_clouds_.at(name);
    }
    catch(std::out_of_range) {
        ROS_WARN_STREAM(
            "[CloudGenerator]: No robots with name '" << name 
            << "' have data stored within the cloud generator.");
        return;
    }
    ros::Publisher *pub;
    try {
        pub = &cloud_pubs_.at(name);
    }
    catch(std::out_of_range) {
        cloud_pubs_[name] = private_nh_.advertise<sensor_msgs::PointCloud2>(
            name + "_point_cloud", 1);
        pub = &cloud_pubs_.at(name);
    }
    pub->publish(convert_cloud(cloud, name + '/' + odom_name_));
}

void CloudGenerator::publish_full_cloud() {
    std::lock_guard<std::mutex> lock(full_cloud_mut_);
    full_cloud_pub_.publish(convert_cloud(full_cloud_, "map"));
}

void CloudGenerator::clean_all_clouds() {
    for(auto & pair : robot_clouds_) {
        std::lock_guard<std::recursive_mutex> lock(
            robot_cloud_muts_[pair.first]);
        clean_cloud(pair.second);
    }
    std::lock_guard<std::mutex> lock(full_cloud_mut_);
    clean_cloud(full_cloud_);
}

void CloudGenerator::clean_cloud(std::string name) {
    // Initialise reference with a cloud we know exists
    pcl::PointCloud<pcl::PointXYZI>& cloud = full_cloud_;
    try {
        cloud = robot_clouds_.at(name);
    }
    catch(std::out_of_range) {
        ROS_WARN_STREAM(
            "[CloudGenerator]: No robots with name '" << name 
            << "' have data stored within the cloud generator.");
        return;
    }
    std::lock_guard<std::recursive_mutex> lock(robot_cloud_muts_[name]);
    clean_cloud(cloud);
}

void CloudGenerator::clean_cloud(pcl::PointCloud<pcl::PointXYZI>& cloud) {
    // Calculate the cutoff point
    ros::Time cutoff_ros;
    try {
        cutoff_ros = (ros::Time::now() - ros::Duration(history_length_));
    }
    catch(std::runtime_error) {
        // This means that history_length_ seconds have not passed yet
        return;
    }
    float cutoff = cutoff_ros.sec + cutoff_ros.nsec / 1e9;
    // Remove the points which are too old
    for(auto it = cloud.begin(); it != cloud.end(); it++) {
        if(it->intensity < cutoff) {
            it = --cloud.erase(it);
        }
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

    // Transform the point from the range finder frame into the cloud frame
    std::string odom = prefix + '/' + odom_name_;
    geometry_msgs::PointStamped point;
    point.header = msg->header;
    point.point.x = msg->range;
    try {
        tf_buffer_.transform(point, point, odom);
    }
    catch(tf2::TransformException &ex) {
        ROS_WARN("Failed to transform point: %s\n", ex.what());
        return;
    }

    // Add the point to the point cloud
    pcl::PointXYZI pcl_point;
    pcl_point.x = point.point.x;
    pcl_point.y = point.point.y;
    pcl_point.z = point.point.z;
    pcl_point.intensity = 
        point.header.stamp.sec + point.header.stamp.nsec / 1e9;
    std::lock_guard<std::recursive_mutex> lock(robot_cloud_muts_[prefix]);
    robot_clouds_[prefix].push_back(pcl_point);

    // TODO: Add the point to the full cloud
}

sensor_msgs::PointCloud2 CloudGenerator::convert_cloud(
        const pcl::PointCloud<pcl::PointXYZI>& cloud,
        std::string frame_name)
{
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header.stamp = ros::Time::now();
    ros_cloud.header.frame_id = frame_name;
    return ros_cloud;
}
