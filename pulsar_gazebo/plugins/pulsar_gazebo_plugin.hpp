#ifndef __PULSAR_GAZEBO_PLUGIN_HPP__
#define __PULSAR_GAZEBO_PLUGIN_HPP__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <pulsar_lib/odometry/odometry.hpp>
#include <pulsar_lib/kalman_filter/kalman_filter.hpp>
#include <pulsar_lib/motor_control/motor_control.hpp>

#include <thread>
#include <memory>
#include <random>

namespace gazebo {

class PulsarGazeboPlugin :public ModelPlugin {
public:
    // ctor
    PulsarGazeboPlugin();
    // dtor
    virtual ~PulsarGazeboPlugin();

    // Load function used by Gazebo to load the plugin
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Update function called by Gazebo at each simulation step
    void OnUpdate();

private:
    /****************
     * Gazebo Stuff *
     ****************/
    // Connection to the update event in Gazebo
    event::ConnectionPtr update_connection_;

    // Pointer to the model we are controlling
    physics::ModelPtr model_;

    // Pointer to the left and right wheel joints used to measure odometry
    physics::JointPtr l_wheel_joint_;
    physics::JointPtr r_wheel_joint_;

    /*************
     * ROS Stuff *
     *************/
    // Thread function to process ROS callbacks
    void ROSQueueThread();

    // Function to process new velocity commands
    void cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg);

    // Function to process IMU data
    void imu_cb(const sensor_msgs::ImuConstPtr& msg);

    // Pointer so that we can create this in the Load function
    ros::NodeHandle *nh_;
    // Odometry publisher
    ros::Publisher odom_pub_;
    // Cmd velocity subscriber
    ros::Subscriber cmd_vel_sub_;
    // IMU subscriber
    ros::Subscriber imu_sub_;
    // Allows us to manually process the callback queue
    ros::CallbackQueue ros_queue_;
    std::thread callback_queue_thread_;

    /***************
     * Other Stuff *
     ***************/

    // Fakes encoder data and passes it to the Odometry class below
    void update_odometry_measurements();

    // Classes used in the real robot
    std::unique_ptr<KalmanFilter> ekf_;
    std::unique_ptr<MotorControl> l_motor_;
    std::unique_ptr<MotorControl> r_motor_;
    std::unique_ptr<Odometry> odom_;

    // Encoder counts per revolution - used to fake encoder pulses for the
    // Odometry class
    int cpr_;
    // How regularly we update the kalman filter/odometry etc.
    float update_freq_;

    // Left and right joint positions
    float l_prev_pos_;
    float r_prev_pos_;

    // Maximum torque applied to a motor
    float motor_max_torque_;

    // IMU theta measurement
    float imu_theta_;
    // IMU x_ddot measurement
    float imu_x_ddot_;
    // IMU theta covariance
    float imu_theta_cov_;
    // IMU x_ddot covariance
    float imu_x_ddot_cov_;

    // Noise multiplied into odometry readings
    std::default_random_engine rand_gen_;
    std::uniform_real_distribution<float> dist_;

    // Last time we were updated
    ros::Time last_update_time_;
};

}

#endif // __PULSAR_GAZEBO_PLUGIN_HPP__
