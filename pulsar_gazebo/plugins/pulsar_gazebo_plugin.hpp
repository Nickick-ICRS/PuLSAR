#ifndef __PULSAR_GAZEBO_PLUGIN_HPP__
#define __PULSAR_GAZEBO_PLUGIN_HPP__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <pulsar_lib/odometry/odometry.hpp>
#include <pulsar_lib/kalman_filter/kalman_filter.hpp>
#include <pulsar_lib/motor_control/motor_control.hpp>

#include <thread>
#include <memory>

namespace gazebo {

class PulsarGazeboPlugin :public ModelPlugin {
public:
    // ctor
    PulsarGazeboPlugin();
    // dtor
    virtual ~PulsarGazeboPlugin();

    // Load function used by Gazebo to load the plugin
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
    /****************
     * Gazebo Stuff *
     ****************/
    // Connection to the update event in Gazebo
    event::ConnectionPtr update_connection_;

    // Pointer to the model we are controlling
    physics::ModelPtr model_;

    /*************
     * ROS Stuff *
     *************/
    // Thread function to process ROS callbacks
    void ROSQueueThread();

    // Function to process new velocity commands
    void cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg);

    // Pointer so that we can create this in the Load function
    ros::NodeHandle *nh_;
    // Odometry publisher
    ros::Publisher odom_pub_;
    // Cmd velocity subscriber
    ros::Subscriber cmd_vel_sub_;
    // Allows us to manually process the callback queue
    ros::CallbackQueue ros_queue_;
    std::thread callback_queue_thread_;

    /***************
     * Other Stuff *
     ***************/

    // Classes used in the real robot
    std::unique_ptr<KalmanFilter> ekf_;
    std::unique_ptr<MotorControl> l_motor_;
    std::unique_ptr<MotorControl> r_motor_;
    std::unique_ptr<Odometry> odom_;

    // Encoder counts per revolution - used to fake encoder pulses for the
    // Odometry class
    int cpr_;
    // How regularly we update odometry
    float odom_update_freq_;
};

}

#endif // __PULSAR_GAZEBO_PLUGIN_HPP__
