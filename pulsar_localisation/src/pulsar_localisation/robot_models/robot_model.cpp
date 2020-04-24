#include "robot_models/robot_model.hpp"

#include "maths/useful_functions.hpp"

double RobotModel::a1_,
       RobotModel::a2_,
       RobotModel::a3_,
       RobotModel::a4_;

RobotModel::RobotModel(
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
        odom_topic, 1, &RobotModel::odom_cb, this);
}

RobotModel::~RobotModel() {
    // dtor
}

void RobotModel::update_odom() {
    std::lock_guard<std::mutex> lock(odom_mut_);
    prev_odom_ = recent_odom_;
}

double RobotModel::weigh_pose(const geometry_msgs::Pose& p) {
    const auto Z = cloud_gen_->get_raw_data(name_);
    return range_model_.model(p, Z, name_, base_link_frame_);
}

geometry_msgs::Pose RobotModel::sample_motion_model_odometry(
    const geometry_msgs::Pose& xt_1)
{
    std::lock_guard<std::mutex> lock(odom_mut_);
    return sample_motion_model_odometry(
        recent_odom_.pose.pose, prev_odom_.pose.pose, xt_1);
}

geometry_msgs::Pose RobotModel::sample_motion_model_odometry(
    const geometry_msgs::Pose& ut, const geometry_msgs::Pose& ut_1, 
    const geometry_msgs::Pose& xt_1)
{
    double tht = quat_to_yaw(ut.orientation);
    double tht_1 = quat_to_yaw(ut_1.orientation);
    double drot1 = atan2(ut.position.y - ut_1.position.y,
                        ut.position.x - ut_1.position.x) - tht_1;
    double dtrans = sqrt(pow(ut_1.position.x - ut.position.x, 2) +
                         pow(ut_1.position.y - ut.position.y, 2));
    if(dtrans <= min_trans_update_) {
        drot1 = 0;
    }
    double drot2 = tht - tht_1 - drot1;

    double sdrot1 = drot1 - sample_normal_distribution(
        a1_ * drot1 * drot1 + a2_ * dtrans * dtrans);
    double sdtrans = dtrans - sample_normal_distribution(
        a3_ * dtrans * dtrans + a4_ * drot1 * drot1 + a4_ * drot2 * drot2);
    double sdrot2 = drot2 - sample_normal_distribution(
        a1_ * drot2 * drot2 + a2_ * dtrans * dtrans);

    geometry_msgs::Pose xt;
    xt.position.x = xt_1.position.x + sdtrans * cos(tht_1 + sdrot1);
    xt.position.y = xt_1.position.y + sdtrans * sin(tht_1 + sdrot1);

    double th = quat_to_yaw(xt_1.orientation) + sdrot1 + sdrot2;
    xt.orientation = yaw_to_quat(th);
    return xt;
}

geometry_msgs::Pose 
    RobotModel::gen_random_valid_pose_position(double yaw)
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

geometry_msgs::Pose RobotModel::gen_random_valid_pose() {
    static std::uniform_real_distribution<double> thdist(-M_PI, M_PI);
    geometry_msgs::Pose pose(gen_random_valid_pose_position(0));
    do {
        double th = thdist(gen_);
        pose.orientation = yaw_to_quat(th);
    }
    while(!map_man_->is_pose_valid(pose, radius_));

    return pose;
}

geometry_msgs::Pose RobotModel::gen_random_valid_pose(
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

void RobotModel::odom_cb(
    const nav_msgs::OdometryConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(odom_mut_);
    recent_odom_ = *msg;
    if(first_odom_cb_) {
        first_odom_cb_ = false;
        prev_odom_ = *msg;
    }
}

geometry_msgs::TransformStamped RobotModel::calculate_transform(
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
