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

    // ROS topic on which to listen for imu data
    std::string imu_topic = "imu/data";
    if(_sdf->HasElement("imuTopic"))
        cmd_vel_topic = _sdf->Get<std::string>("imuTopic");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter 'imuTopic'. "
            << "Defaulting to '" << imu_topic << "'.");

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
    update_freq_ = 10;
    if(_sdf->HasElement("updateRate"))
        update_freq_ = _sdf->Get<float>("updateRate");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter "
            << "'odomUpdateRate'. Defaulting to '" << update_freq_
            << "'.");

    // Left wheel joint name
    std::string joint_name = "left_wheel_joint";
    if(_sdf->HasElement("leftWheelJointName"))
        joint_name = _sdf->Get<std::string>("leftWheelJointName");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter "
            << "'leftWheelJointName'. Defaulting to '" << joint_name
            << "'.");
    l_wheel_joint_ = model_->GetJoint(joint_name);

    if(!l_wheel_joint_) {
        ROS_FATAL_STREAM(
            "Left wheel joint '" << joint_name << "' could not be found "
            << "within the model. Aborting.");
        return;
    }

    motor_max_torque_ = 1;
    if(_sdf->HasElement("motorMaxTorque"))
        motor_max_torque_ = _sdf->Get<float>("motorMaxTorque");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter "
            << "'motorMaxTorque'. Defaulting to '" << motor_max_torque_
            << "'.");

    // Right wheel joint name
    joint_name = "right_wheel_joint";
    if(_sdf->HasElement("rightWheelJointName"))
        joint_name = _sdf->Get<std::string>("rightWheelJointName");
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter "
            << "'rightWheelJointName'. Defaulting to '" << joint_name
            << "'.");
    r_wheel_joint_ = model_->GetJoint(joint_name);

    if(!r_wheel_joint_) {
        ROS_FATAL_STREAM(
            "Right wheel joint '" << joint_name << "' could not be found "
            << "within the model. Aborting.");
        return;
    }

    // Odometry random noise
    float odom_noise = 0;
    if(_sdf->HasElement("odomUniformNoise"))
        odom_noise = fabs(_sdf->Get<float>("odomUniformNoise"));
    else
        ROS_WARN_STREAM(
            "PulsarGazeboPlugin failed to get SDF parameter "
            << "'odomUniformNoise'. Defaulting to '" << odom_noise
            << "'.");
    
    if(odom_noise > 1.0) {
        ROS_FATAL_STREAM(
            "odomUniformNoise can't be greater than 1 (100%)!");
        return;
    }

    dist_ = std::uniform_real_distribution<float>(-odom_noise, odom_noise);

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
    imu_sub_ = nh_->subscribe(
        imu_topic, 1, &PulsarGazeboPlugin::imu_cb, this);

    // Store the current time
    last_update_time_ = ros::Time::now();

    // Connect to the update event from gazebo
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PulsarGazeboPlugin::OnUpdate, this));

    // Create the ROS callback thread
    callback_queue_thread_ = std::thread(
        &PulsarGazeboPlugin::ROSQueueThread, this);
}

void PulsarGazeboPlugin::OnUpdate() {
    // Calculate timestep
    auto dt = ros::Time::now() - last_update_time_;
    // Don't update the controllers etc. if we haven't waited long enough
    if(dt >= ros::Duration(1 / update_freq_)) {
        last_update_time_ += dt;
        float dt_s = dt.sec + dt.nsec / 1e9;

        // Update odometry
        odom_->update(dt_s);
        float x_dot = odom_->get_forwards_vel();
        float th_dot = odom_->get_theta_vel();
        float left_vel = odom_->get_left_vel();
        float right_vel = odom_->get_right_vel();
        float x_dot_cov = odom_->calculate_cov_forwards_vel();
        float th_dot_cov = odom_->calculate_cov_theta_vel();

        // Update kalman filter
        Eigen::Vector4f z_k(imu_theta_, x_dot, th_dot, imu_x_ddot_);
        Eigen::Matrix4f R_k;
        R_k << imu_theta_cov_,         0,          0,                0,
               0             , x_dot_cov,          0,                0,
               0             ,         0, th_dot_cov,                0,
               0             ,         0,          0,  imu_x_ddot_cov_;
        ekf_->update(dt_s, z_k, R_k);

        const Eigen::Vector5f& state = ekf_->get_state();
        const Eigen::Matrix5f& cov = ekf_->get_covariance();

        // Publish odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x = state[0];
        odom_msg.pose.pose.position.y = state[1];
        odom_msg.pose.pose.orientation.z = sin(state[2]/2);
        odom_msg.pose.pose.orientation.w = cos(state[2]/2);
        odom_msg.pose.covariance[0] = cov(0, 0);
        odom_msg.pose.covariance[7] = cov(1, 1);
        odom_msg.pose.covariance[35] = cov(2, 2);
        odom_msg.twist.twist.linear.x = state[3];
        odom_msg.twist.twist.angular.z = state[4];
        odom_msg.twist.covariance[0] = cov(3, 3);
        odom_msg.twist.covariance[35] = cov(4, 4);

        odom_pub_.publish(odom_msg);

        // Update motor controllers
        l_motor_->set_current_vel(left_vel);
        r_motor_->set_current_vel(right_vel);
        l_motor_->update(dt_s);
        r_motor_->update(dt_s);
    }

    // Apply torques to wheels at every timestep
    float pc = l_motor_->get_power_percentage();
    l_wheel_joint_->SetForce(0, pc * motor_max_torque_);
    pc = r_motor_->get_power_percentage();
    r_wheel_joint_->SetForce(0, pc * motor_max_torque_);
}

void PulsarGazeboPlugin::ROSQueueThread() {
    while(ros::ok()) {
        ros_queue_.callAvailable(ros::WallDuration(0.1));
    }
}

void PulsarGazeboPlugin::cmd_vel_cb(
    const geometry_msgs::TwistConstPtr& msg)
{
    // Calculate required wheel velocities and update the controllers
    float left, right;
    odom_->calculate_wheel_speeds(
        &left, &right, msg->linear.x, msg->angular.z);

    l_motor_->set_target_vel(left);
    r_motor_->set_target_vel(right);
}

void PulsarGazeboPlugin::imu_cb(const sensor_msgs::ImuConstPtr& msg) {
    imu_theta_ = 2 * acos(msg->orientation.w);
    imu_x_ddot_ = msg->linear_acceleration.x;
    imu_theta_cov_ = msg->orientation_covariance[8];
    imu_x_ddot_cov_ = msg->linear_acceleration_covariance[0];
}

void PulsarGazeboPlugin::update_odometry_measurements() {
    // Calculate change of angle
    float pos = l_wheel_joint_->Position();
    float delta = pos - l_prev_pos_;
    l_prev_pos_ = pos;

    // Calculate number of pulses that should be generated
    int counts = cpr_ * delta / (2 * 3.1415926);

    // Add some random noise
    float noise = dist_(rand_gen_);
    counts *= noise;

    // True if we're moving in a positive direction
    bool dir = (counts == abs(counts));
    counts = abs(counts);

    // Update the odometry measurement class
    for(int c = 0; c < counts; c++) {
        odom_->update_left_wheel_count(dir);
    }

    // Repeat for right wheel
    pos = r_wheel_joint_->Position();
    delta = pos - r_prev_pos_;
    r_prev_pos_ = pos;

    // Calculate number of pulses that should be generated
    counts = cpr_ * delta / (2 * 3.1415926);

    // Add some random noise
    noise = dist_(rand_gen_);
    counts *= noise;

    // True if we're moving in a positive direction
    dir = (counts == abs(counts));
    counts = abs(counts);

    for(int c = 0; c < counts; c++) {
        odom_->update_right_wheel_count(dir);
    }
}

}
