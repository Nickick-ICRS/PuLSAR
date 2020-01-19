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

    // Kalman filter Q
    Eigen::Matrix5f Q_est = Eigen::Matrix5f::Zero();
    if(_sdf->HasElement("Q_x") && _sdf->HasElement("Q_y") 
       && _sdf->HasElement("Q_th") && _sdf->HasElement("Q_x_dot")
       && _sdf->HasElement("Q_th_dot")) {
        Q_est(0, 0) = _sdf->Get<float>("Q_x");
        Q_est(1, 1) = _sdf->Get<float>("Q_y");
        Q_est(2, 2) = _sdf->Get<float>("Q_th");
        Q_est(3, 3) = _sdf->Get<float>("Q_x_dot");
        Q_est(4, 4) = _sdf->Get<float>("Q_th_dot");
    }
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameters 'Q_x', 'Q_y', "
            << "'Q_th', 'Q_x_dot' and 'Q_th_dot'. Defaulting EKF process "
            << "noise Q to: '" << Q_est << "'.");

    // Motor P control constant
    float mtr_kp = 1;
    if(_sdf->HasElement("motorKp"))
        mtr_kp = _sdf->Get<float>("motorKp");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter 'motorKp'. "
            << "Defaulting to '" << mtr_kp << "'.");

    // Motor I control constant
    float mtr_ki = 1;
    if(_sdf->HasElement("motorKi"))
        mtr_ki = _sdf->Get<float>("motorKi");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter 'motorKi'. "
            << "Defaulting to '" << mtr_ki << "'.");

    // Counts per revolution for the encoders
    cpr_ = 4000;
    if(_sdf->HasElement("cpr"))
        cpr_ = _sdf->Get<int>("cpr");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter 'cpr'. "
            << "Defaulting to '" << cpr_ << "'.");

    // Radius of the wheels
    float wheel_radius = 0.015f;
    if(_sdf->HasElement("wheelRadius"))
        wheel_radius = _sdf->Get<float>("wheelRadius");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter 'wheelRadius'. "
            << "Defaulting to '" << wheel_radius << "'.");

    // Separation of the wheels
    float wheel_separation = 0.02f;
    if(_sdf->HasElement("wheelSeparation"))
        wheel_radius = _sdf->Get<float>("wheelSeparation");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter"
            << " 'wheelSeparation'. Defaulting to '" << wheel_separation 
            << "'.");

    // Odometry update rate
    odom_update_freq_;
    if(_sdf->HasElement("odomUpdateRate"))
        wheel_radius = _sdf->Get<float>("odomUpdateRate");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter "
            << "'odomUpdateRate'. Defaulting to '" << odom_update_freq_
            << "'.");

    // Create the KalmanFilter object
    ekf_ = std::unique_ptr<KalmanFilter>(new KalmanFilter(Q_est));
    // Create the motor controller objects
    l_motor_ = std::unique_ptr<MotorControl>(
        new MotorControl(mtr_kp, mtr_ki));
    r_motor_ = std::unique_ptr<MotorControl>(
        new MotorControl(mtr_kp, mtr_ki));
    // Create the odometry object
    odom_ = std::unique_ptr<Odometry>(
        new Odometry(cpr_, wheel_radius, wheel_separation));

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
