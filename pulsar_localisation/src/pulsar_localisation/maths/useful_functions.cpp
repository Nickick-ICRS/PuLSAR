#include "maths/useful_functions.hpp"

#include <cmath>
#include <random>

double quat_to_yaw(const geometry_msgs::Quaternion& q) {
    return 2*acos(q.w) * (q.z < 0 ? -1 : 1);
}

geometry_msgs::Quaternion yaw_to_quat(double yaw) {
    geometry_msgs::Quaternion q;
    q.w = cos(yaw/2);
    q.z = sin(yaw/2);
    return q;
}

double prob_normal_distribution(double a, double b2) {
    return (1.0/sqrt(2*M_PI*b2))*exp(-0.5 * a*a/b2);
}

double sample_normal_distribution(double b2) {
    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    std::normal_distribution<double> dist(0, sqrt(b2));
    return dist(gen);
}

geometry_msgs::PoseWithCovariance calculate_pose_with_covariance(
    const std::vector<geometry_msgs::Pose>& poses)
{
    if(poses.size() == 0)
        throw std::runtime_error(
            "Can't calculate the average of no points!");

    double avg_yaw = 0;
    geometry_msgs::Pose p;

    geometry_msgs::Point& avg_pt = p.position;

    double n = poses.size();

    for(auto& pose : poses) {
        avg_yaw += quat_to_yaw(pose.orientation);
        avg_pt.x += pose.position.x;
        avg_pt.y += pose.position.y;
    }

    avg_yaw /= n;
    avg_pt.x /= n;
    avg_pt.y /= n;

    if(!std::isfinite(avg_yaw))
        throw std::runtime_error("Average yaw is not finite!");

    while(avg_yaw > M_PI) avg_yaw -= 2*M_PI;
    while(avg_yaw <= -M_PI) avg_yaw += 2*M_PI;

    p.orientation = yaw_to_quat(avg_yaw);

    return calculate_pose_with_covariance(p, poses);
}

geometry_msgs::PoseWithCovariance calculate_pose_with_covariance(
    const geometry_msgs::Pose& pose,
    const std::vector<geometry_msgs::Pose>& poses)
{
    geometry_msgs::PoseWithCovariance ret;
    ret.pose = pose;

    auto yaw = quat_to_yaw(pose.orientation);
    auto& pt = pose.position;

    auto& cov_mat = ret.covariance;

    double n = poses.size() - 1;
    if(n <= 1e-3) n = 1e-3;

    for(auto& pose : poses) {
        double dx = pose.position.x - pt.x;
        double dy = pose.position.y - pt.y;
        double dz = quat_to_yaw(pose.orientation) - yaw;
        if(dz > M_PI) dz -= 2*M_PI;
        if(dz <= -M_PI) dz += 2*M_PI;
        cov_mat[0]  += dx * dx / n;
        cov_mat[1]  += dx * dy / n;
        cov_mat[5]  += dx * dz / n;
        cov_mat[7]  += dy * dy / n;
        cov_mat[11] += dy * dz / n;
        cov_mat[35] += dz * dz / n;
    }
    cov_mat[6]  = cov_mat[1];
    cov_mat[30] = cov_mat[5];
    cov_mat[31] = cov_mat[11];

    return ret;
}

double clamp_angle(double ang) {
    if(!std::isfinite(ang))
        return ang;
    while(ang > M_PI) ang -= 2*M_PI;
    while(ang <= -M_PI) ang += 2*M_PI;
    return ang;
}
