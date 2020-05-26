#include "robot_models/scan_matching_robot_model.hpp"

#include "maths/useful_functions.hpp"

#include <numeric>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>

double ScanMatchingRobotModel::lamcont_;

ScanMatchingRobotModel::ScanMatchingRobotModel(
    std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::string odom_topic, std::string base_link_frame, float radius,
    double min_trans_update)

    :OdometryRobotModel(
        name, cloud_gen, map_man, map_frame, odom_topic, base_link_frame, 
        radius, min_trans_update), tf_listener_(tf2_)
{
    // ctor
}
ScanMatchingRobotModel::~ScanMatchingRobotModel() {
    // dtor
}
geometry_msgs::Pose ScanMatchingRobotModel::sample_motion_model(
    const geometry_msgs::Pose& xt_1)
{
    // Sample odometry motion model for initial estimate
    auto xt = OdometryRobotModel::sample_motion_model(xt_1);

    // Laser scan match to adjust pose to fit the map.
    return scan_match_l2(xt, xt_1);
}

double ScanMatchingRobotModel::weigh_pose(const geometry_msgs::Pose& p) {
    if(!map_man_->is_pose_valid(p))
        return 0;
    auto Z_pcl = get_sensor_data(p);
    auto tree = map_man_->get_map_pcl_tree();
    double l2_err;
    std::vector<float> dist(1);
    std::vector<int> index(1);
    for(unsigned int i = 0; i < Z_pcl->size(); i++) {
        tree->nearestKSearch(Z_pcl->at(i), 1, index, dist);
        l2_err += pow(dist[0], 2);
    } 
    return 1/(0.1+l2_err);
}

/**
 * Find the point at which the dot product is zero.
 * Vector A, start of A As, start of B Bs
 *
 * Returns the intersection point P where A.dot(P-Bs) = 0.
 */
Eigen::Vector3f find_intersection(
    Eigen::Vector3f A, Eigen::Vector3f As, Eigen::Vector3f Bs)
{
    // Using Gram-Schmidt, here's a random vector
    Eigen::Vector3f u(0.432423, 0.432875984325, -0.43241324);
    double a = A.transpose() * u;
    double b = A.transpose() * A;
    Eigen::Vector3f B = u - (a/b) * A;
    typedef Eigen::Hyperplane<float, 2> Line2;
    Eigen::Vector2f A2(A.x(), A.y()), As2(As.x(), As.y()),
                    B2(B.x(), B.y()), Bs2(Bs.x(), Bs.y());
    Line2 Ap = Line2::Through(As2, As2 + A2);
    Line2 Bp = Line2::Through(Bs2, Bs2 + B2);
    Eigen::Vector2f res = Ap.intersection(Bp);
    return Eigen::Vector3f(res.x(), res.y(), 0);
}

geometry_msgs::Pose ScanMatchingRobotModel::scan_match_l2(
    const geometry_msgs::Pose& xt, const geometry_msgs::Pose& xt_1)
{
    auto Z_pcl = get_sensor_data(xt);
    // Convert from ROS to Eigen
    Eigen::Affine3d eig_posed;
    Eigen::Affine3f eig_pose;
    Eigen::fromMsg(xt, eig_posed);
    eig_pose = eig_posed.cast<float>();

    double MAX_TRANS(0.05);

    auto tree = map_man_->get_map_pcl_tree();
    auto map_cloud = map_man_->get_map_cloud();
    double best_err = std::numeric_limits<double>::max();
    Eigen::Affine3f best_pose;
    std::vector<float> dist(1);
    std::vector<int> index(1);
    for(unsigned int i = 0; i < Z_pcl->size(); i++) {
        pcl::PointCloud<pcl::PointXYZ> Z_trans;
        pcl::PointCloud<pcl::PointXYZ> Z_rot;
        pcl::PointCloud<pcl::PointXYZ> Z_final;
        auto z = Z_pcl->at(i);
        // Find the nearest point to this measurement on the map
        tree->nearestKSearch(z, 1, index, dist);
        auto pt = map_cloud->at(index[0]);
        // Transform the scan to that point, along the forwards vector
        Eigen::Vector3f tp(pt.x-z.x, pt.y-z.y, 0);
        Eigen::Vector3f forw = eig_pose.linear() * Eigen::Vector3f::UnitX();
        forw = forw.normalized();
        tp = tp.dot(forw) * forw;
        if(tp.norm() > MAX_TRANS)
            tp *= MAX_TRANS / tp.norm();
        Eigen::Affine3f aff(Eigen::Affine3f::Identity());
        aff.translation() = Eigen::Translation3f(tp).translation();
        pcl::transformPointCloud(*Z_pcl, Z_trans, aff);
        Eigen::Affine3f eig_pose_translated = aff * eig_pose;
        /*

        // To transform the cloud to the origin when rotating about the 
        // point
        Eigen::Affine3f ori_trans(Eigen::Translation3f(-pt.x, -pt.y, 0));
        for(unsigned int j = 0; j < Z_trans.size(); j++) {
            if(j == i) continue;
            auto z2 = Z_trans[j];

            tree->nearestKSearch(z2, 1, index, dist);
            pt = map_cloud->at(index[0]);
            Eigen::Vector3f z2z(z.x - z2.x, z.y-z2.y, 0);
            //Eigen::Vector2f z2pt(pt.x - z2.x, pt.y - z2.y);
            Eigen::Vector3f ptz(z.x - pt.x, z.y - pt.y, 0);
            double sig = asin(Eigen::Vector3f::UnitZ().cross(
                z2z.normalized()).dot(ptz.normalized()));


            Eigen::Affine3f rot;
            rot = ori_trans.inverse()
                * Eigen::AngleAxisf(sig, Eigen::Vector3f::UnitZ()) 
                * ori_trans;
            pcl::transformPointCloud(Z_trans, Z_rot, rot);

            Eigen::Affine3f eig_pose_rot = rot * eig_pose_translated;

            // Now move along the vector between the points to
            // minimise distance from the original pose
            Eigen::Vector3f pt_vec(z.x - z2.x, z.y - z2.y, 0);
            Eigen::Vector3f pt = eig_pose_rot.translation();
            // pt + X * pt_vec = new_pt
            Eigen::Vector3f new_pt = find_intersection(
                pt_vec, pt, eig_pose.translation());
            Eigen::Translation3f trans(new_pt - pt);

            pcl::transformPointCloud(
                Z_rot, Z_final, Eigen::Affine3f(trans));
            eig_pose_rot *= trans;
            */
            double l2_err;
            for(const auto& z3 : Z_trans/*final*/) {
                tree->nearestKSearch(z3, 1, index, dist);
                l2_err += pow(dist[0], 2);
            }
            // Finally, add the l2err of the distance between the poses
            double dx = eig_pose_translated/*rot*/.translation().x() - xt_1.position.x;
            double dy = eig_pose_translated/*rot*/.translation().y() - xt_1.position.y;
            double dth = 0;/*eig_pose_rot.rotation().eulerAngles(2, 1, 0)[0]
                       - quat_to_yaw(xt_1.orientation);*/
            if(dth > M_PI)   dth -= 2*M_PI;
            if(dth <= -M_PI) dth += 2*M_PI;
            l2_err += lamcont_ * (dx * dx + dy * dy + dth * dth);

            if(l2_err < best_err) {
                best_err = l2_err;
                best_pose = eig_pose_translated;//rot;
            }
        //}
    }
    
    eig_posed = best_pose.cast<double>();

    geometry_msgs::Pose ret = tf2::toMsg(eig_posed);

    return ret;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
    ScanMatchingRobotModel::get_sensor_data(const geometry_msgs::Pose& p)
{
    const auto Z_raw = cloud_gen_->get_raw_data(name_);

    // Transform the range data into our coordinate frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr Z_pcl(
        new pcl::PointCloud<pcl::PointXYZ>);
    for(const auto& z : Z_raw) {
        if(z.range >= z.max_range)
            continue;
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = z.range;
        ps.pose.orientation.w = 1;
        ps.header = z.header;
        try {
            tf2_.transform(ps, ps, base_link_frame_);
        }
        catch(const tf2::TransformException & ex) {
            ROS_WARN_STREAM(
                "get_sensor_data caught exception: " << ex.what());
            continue;
        }

        double x = ps.pose.position.x;
        double y = ps.pose.position.y;
        double yaw = quat_to_yaw(p.orientation);

        ps.pose.position.x = p.position.x + x * cos(yaw) - y * sin(yaw);
        ps.pose.position.y = p.position.y + x * sin(yaw) + y * cos(yaw);

        Z_pcl->push_back(pcl::PointXYZ(
            ps.pose.position.x, ps.pose.position.y, 0));
    }

    return Z_pcl;
}
