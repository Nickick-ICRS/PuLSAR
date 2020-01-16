#include <pulsar_gazebo_plugin.hpp>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(PulsarGazeboPlugin)

PulsarGazeboPlugin::PulsarGazeboPlugin() :ModelPlugin() {
    // ctor
}

PulsarGazeboPlugin::~PulsarGazeboPlugin() {
    if(nh_) {
        nh_->shutdown();
        ros_queue_.clear();
        ros_queue_.disable();
        callback_queue_thread_.join();
        delete nh_;
    }
}

void PulsarGazeboPlugin::Load(
    physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if(!ros::isInitialized()) {
        ROS_FATAL_STREAM(
            "A ROS node for Gazebo has not been initialized, unable to load"
            << " plugin. Load the Gazebo system plugin "
            << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
        return;
    }

    model_ = _model;

    // ROS Namespace this plugin exists within
    std::string ns;
    if (_sdf->HasElement("namespace"))
        ns = _sdf->Get<std::string>("namespace");
    else 
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter 'namespace'. "
            << "Defaulting to '" << ns << "'.");
    
    // ROS topic on which to publish odometry
    std::string odom_topic = "odometry/filtered";
    if(_sdf->HasElement("odomTopic"))
        odom_topic = _sdf->Get<std::string>("odomTopic");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter 'odomTopic'. "
            << "Defaulting to '" << odom_topic << "'.");

    // ROS topic on which to listen for cmd velocities
    std::string cmd_vel_topic = "cmd_vel";
    if(_sdf->HasElement("cmdVelTopic"))
        cmd_vel_topic = _sdf->Get<std::string>("cmdVelTopic");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter 'cmdVelTopic'. "
            << "Defaulting to '" << cmd_vel_topic << "'.");

    // Create a node handle within the namespace
    nh_ = new ros::NodeHandle(ns);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic, 10);
    cmd_vel_sub_ = nh_->subscribe(
        cmd_vel_topic, 1, &PulsarGazeboPlugin::cmd_vel_cb, this);
}

void PulsarGazeboPlugin::ROSQueueThread() {
    
}

void PulsarGazeboPlugin::cmd_vel_cb(
    const geometry_msgs::TwistConstPtr& msg)
{

}

}
