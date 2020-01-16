#ifndef __PULSAR_GAZEBO_PLUGIN_HPP__
#define __PULSAR_GAZEBO_PLUGIN_HPP__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <thread>

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

    // Calculate target wheel velocities from a given target Twist
    // input vx: x linear velocity w.r.t center of robot
    // input vtheta: z angular velocity w.r.t center of robot
    // output vl: left wheel angular velocity
    // output vr: right wheel angular velocity
    void calculate_required_wheel_velocities(
        float vx, float vtheta, float *vl, float *vr);
    // Kalman filter as used in the real PuLSAR robot
    void update_kalman_filter();
    // Raw odometry as used in the real PuLSAR robot
    void calculate_raw_odometry();
};

}

#endif // __PULSAR_GAZEBO_PLUGIN_HPP__
