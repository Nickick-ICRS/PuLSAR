#include "robot_models/base_robot_model.hpp"

#include "maths/useful_functions.hpp"

BaseRobotModel::BaseRobotModel(
    std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::string odom_topic, std::string base_link_frame, float radius,
    double min_trans_update)

    :name_(name), cloud_gen_(cloud_gen), map_man_(map_man), rd_(),
    gen_(rd_()), range_model_(map_frame, map_man), map_frame_(map_frame), 
    base_link_frame_(base_link_frame), radius_(radius), 
    min_trans_update_(min_trans_update), first_odom_cb_(true)
{
    odom_sub_ = nh_.subscribe(
        odom_topic, 1, &BaseRobotModel::odom_cb, this);
}

BaseRobotModel::~BaseRobotModel() {
    // dtor
}

geometry_msgs::Pose 
    BaseRobotModel::gen_random_valid_pose_position(double yaw)
{
    static std::uniform_real_distribution<double> xdist(/*map dims*/-1, 1);
    static std::uniform_real_distribution<double> ydist(/*map dims*/-1, 1);
    geometry_msgs::Pose pose;
    pose.orientation = yaw_to_quat(yaw);
    do {
        pose.position.x = xdist(gen_);
        pose.position.y = ydist(gen_);
    }
    while(!map_man_->is_pose_valid(pose, radius_));

    return pose;
}

void BaseRobotModel::update_odom() {
    std::lock_guard<std::mutex> lock(odom_mut_);
    prev_odom_ = recent_odom_;
}

geometry_msgs::Pose BaseRobotModel::gen_random_valid_pose() {
    static std::uniform_real_distribution<double> thdist(-M_PI, M_PI);
    geometry_msgs::Pose pose(gen_random_valid_pose_position(0));
    do {
        double th = thdist(gen_);
        pose.orientation = yaw_to_quat(th);
    }
    while(!map_man_->is_pose_valid(pose, radius_));

    return pose;
}

geometry_msgs::Pose BaseRobotModel::gen_random_valid_pose(
    const geometry_msgs::PoseWithCovariance& p) 
{
    geometry_msgs::Pose pose = p.pose;
    double th = quat_to_yaw(pose.orientation);
    int i = 0;
    do {
        pose.position.x += sample_normal_distribution(p.covariance[0]);
        pose.position.y += sample_normal_distribution(p.covariance[7]);
        th += sample_normal_distribution(p.covariance[35]);
        pose.orientation = yaw_to_quat(th);
        i++;
    }
    while(!map_man_->is_pose_valid(pose, radius_));

    return pose;
}

void BaseRobotModel::odom_cb(
    const nav_msgs::OdometryConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(odom_mut_);
    recent_odom_ = *msg;
    if(first_odom_cb_) {
        first_odom_cb_ = false;
        prev_odom_ = *msg;
    }
}

geometry_msgs::TransformStamped BaseRobotModel::calculate_transform(
        const geometry_msgs::Pose& p)
{
    std::lock_guard<std::mutex> lock(odom_mut_);
    double mRo = quat_to_yaw(p.orientation)
               - quat_to_yaw(recent_odom_.pose.pose.orientation);

    // Create the message and return
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = recent_odom_.header.frame_id;
    double oTbx = recent_odom_.pose.pose.position.x * cos(mRo)
                - recent_odom_.pose.pose.position.y * sin(mRo);
    double oTby = recent_odom_.pose.pose.position.y * cos(mRo)
                + recent_odom_.pose.pose.position.x * sin(mRo);

    tf.transform.translation.x = p.position.x - oTbx;
    tf.transform.translation.y = p.position.y - oTby;
    tf.transform.rotation = yaw_to_quat(mRo);
    return tf;
}
