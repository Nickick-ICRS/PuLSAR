#include "robot_models/odometry_robot_model.hpp"

#include "maths/useful_functions.hpp"

double OdometryRobotModel::a1_,
       OdometryRobotModel::a2_,
       OdometryRobotModel::a3_,
       OdometryRobotModel::a4_;

OdometryRobotModel::OdometryRobotModel(
    std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::string odom_topic, std::string base_link_frame, float radius,
    double min_trans_update)
    
    :BaseRobotModel(
        name, cloud_gen, map_man, map_frame, odom_topic, base_link_frame,
        radius, min_trans_update)
{
    // ctor
}

OdometryRobotModel::~OdometryRobotModel() {
    // dtor
}

double OdometryRobotModel::weigh_pose(const geometry_msgs::Pose& p) {
    const auto Z = cloud_gen_->get_raw_data(name_);
    return range_model_.model(p, Z, name_, base_link_frame_);
}

geometry_msgs::Pose OdometryRobotModel::sample_motion_model(
    const geometry_msgs::Pose& xt_1)
{
    std::lock_guard<std::mutex> lock(odom_mut_);
    return sample_motion_model_odometry(
        recent_odom_.pose.pose, prev_odom_.pose.pose, xt_1);
}

geometry_msgs::Pose OdometryRobotModel::sample_motion_model_odometry(
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
