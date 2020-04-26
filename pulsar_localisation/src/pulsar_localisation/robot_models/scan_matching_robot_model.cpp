#include "robot_models/scan_matching_robot_model.hpp"

#include "maths/useful_functions.hpp"

#include <numeric>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher test;

ScanMatchingRobotModel::ScanMatchingRobotModel(
    std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
    const std::shared_ptr<MapManager>& map_man, std::string map_frame,
    std::string odom_topic, std::string base_link_frame, float radius,
    double min_trans_update)

    :OdometryRobotModel(
        name, cloud_gen, map_man, map_frame, odom_topic, base_link_frame, 
        radius, min_trans_update), tf_listener_(tf2_)
{
    test = nh_.advertise<sensor_msgs::PointCloud2>("/_test_pcl", 1);
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
    return scan_match_l2(xt);
}

double ScanMatchingRobotModel::weigh_pose(const geometry_msgs::Pose& p) {
    const auto Z_raw = cloud_gen_->get_raw_data(name_);

    // Transform the range data into our coordinate frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr Z_pcl(
        new pcl::PointCloud<pcl::PointXYZ>);
    for(const auto& z : Z_raw) {
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = z.range;
        ps.pose.orientation.w = 1;
        ps.header = z.header;
        try {
            tf2_.transform(ps, ps, base_link_frame_);
        }
        catch(const tf2::TransformException & ex) {
            ROS_WARN_STREAM(
                "move_in_dir caught exception: " << ex.what());
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
    
    sensor_msgs::PointCloud2 cl;
    pcl::toROSMsg(*Z_pcl, cl);
    cl.header.frame_id = "map";
    cl.header.stamp = ros::Time::now();
    test.publish(cl);
   
    auto tree = map_man_->get_map_pcl_tree();
    double l2_err;
    std::vector<float> dist(1);
    std::vector<int> index(1);
    for(unsigned int i = 0; i < Z_pcl->size(); i++) {
        tree->nearestKSearch(Z_pcl->at(i), 1, index, dist);
        l2_err += pow(dist[0], 2);
    } 
    return 1/l2_err;
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
    const geometry_msgs::Pose& p)
{
/*
    // Helper lambdas to calculate errors
    static auto l2_pos_err = [](
        const geometry_msgs::Pose& a, const geometry_msgs::Pose& b,
        double scalex, double scaley)
    {
        double dx = a.position.x - b.position.x;
        double dy = a.position.y - b.position.y;
        double dz = quat_to_yaw(a.orientation) - quat_to_yaw(b.orientation);
        if(dz > M_PI) dz -= 2*M_PI;
        if(dz <= -M_PI) dz += 2*M_PI;

        dx /= scalex;
        dy /= scaley;
        dz /= M_PI;

        return dx * dx + dy * dy + dz * dz;
    };
    
    // Order to check data
    static const std::vector<std::tuple<Dir, Dir, Dir>> movement_orders = {
        { Dir::PX, Dir::PY, Dir::PTH }, {Dir::PX, Dir::PY, Dir::NTH},
        { Dir::PX, Dir::NY, Dir::PTH }, {Dir::PX, Dir::NY, Dir::NTH},
        { Dir::NX, Dir::PY, Dir::PTH }, {Dir::NX, Dir::PY, Dir::NTH},
        { Dir::NX, Dir::NY, Dir::PTH }, {Dir::NX, Dir::NY, Dir::NTH}
    };

    double scalex = map_man_->get_width();
    double scaley = map_man_->get_height();
*/
    const auto Z_raw = cloud_gen_->get_raw_data(name_);

    // Transform the range data into our coordinate frame
    std::vector<geometry_msgs::Point> Z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Z_pcl(
        new pcl::PointCloud<pcl::PointXYZ>);
    for(const auto& z : Z_raw) {
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = z.range;
        ps.pose.orientation.w = 1;
        ps.header = z.header;
        try {
            tf2_.transform(ps, ps, base_link_frame_);
        }
        catch(const tf2::TransformException & ex) {
            ROS_WARN_STREAM(
                "move_in_dir caught exception: " << ex.what());
            continue;
        }

        double x = ps.pose.position.x;
        double y = ps.pose.position.y;
        double yaw = quat_to_yaw(p.orientation);

        ps.pose.position.x = p.position.x + x * cos(yaw) - y * sin(yaw);
        ps.pose.position.y = p.position.y + x * sin(yaw) + y * cos(yaw);

        Z.push_back(ps.pose.position);
        Z_pcl->push_back(pcl::PointXYZ(
            ps.pose.position.x, ps.pose.position.y, 0));
    }
    // Convert from ROS to Eigen
    Eigen::Affine3d eig_posed;
    Eigen::Affine3f eig_pose;
    Eigen::fromMsg(p, eig_posed);
    eig_pose = eig_posed.cast<float>();

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
        // Transform the scan to that point
        Eigen::Affine3f aff(Eigen::Translation3f(pt.x-z.x, pt.y-z.y, 0));
        pcl::transformPointCloud(*Z_pcl, Z_trans, aff);
        Eigen::Affine3f eig_pose_translated = aff * eig_pose;
        // To transform the cloud to the origin when rotating about the 
        // point
        Eigen::Affine3f ori_trans(Eigen::Translation3f(-pt.x, -pt.y, 0));
        for(unsigned int j = 0; j < Z_trans.size(); j++) {
            if(j == i) continue;
            auto z2 = Z_trans[j];
            // TODO: Find better method than this rotation crap.
            // Maybe take the closest point, calculate the distance, get
            // angle from trig and rotate to there? It's not perfect, but
            // it should perform well if we're not far off.
            // Find a rotation about the first point to place this point on
            // the map
            const double ROT = 2*M_PI / 30;
            for(double ang = 0; ang < 2*M_PI; ang += ROT) {
                Eigen::Affine3f rot;
                rot = ori_trans.inverse()
                    * Eigen::AngleAxisf(ang, Eigen::Vector3f::UnitZ()) 
                    * ori_trans;
                pcl::transformPointCloud(Z_trans, Z_rot, rot);
                Eigen::Affine3f eig_pose_rot = rot * eig_pose;

                // Now move along the vector between the points to
                // minimise distance from the original pose
                Eigen::Vector3f pt_vec(z.x - z2.x, z.y - z2.y, 0);
                Eigen::Vector3f pt = eig_pose_rot.translation();
                // pt + X * pt_vec = pos_vec
                Eigen::Vector3f new_pt = find_intersection(
                    pt_vec, pt, eig_pose.translation());
                Eigen::Translation3f trans(pt - eig_pose.translation());

                pcl::transformPointCloud(
                    Z_rot, Z_final, Eigen::Affine3f(trans));
                eig_pose_rot *= trans;

                double l2_err;
                for(const auto& z3 : Z_final) {
                    tree->nearestKSearch(z3, 1, index, dist);
                    l2_err += pow(dist[0], 2);
                }

                if(l2_err < best_err) {
                    best_err = l2_err;
                    best_pose = eig_pose_rot;
                }
            }
        }
    }
    
    eig_posed = best_pose.cast<double>();

    geometry_msgs::Pose ret = tf2::toMsg(eig_posed);

    return ret;

/*
    // Now pick the pose with the least error
    double best_err = std::numeric_limits<double>::max();
    auto& best_pose = poses[0];
    // Find normalising values
    std::vector<double> l2(poses.size());
    double norm_l2 = 0;
    double norm_weight = std::numeric_limits<double>::max();

    for(unsigned int i = 0; i < poses.size(); i++) {
        double err = l2_pos_err(p, poses[i], scalex, scaley);
        l2[i] = err;
        if(err > norm_l2)
            norm_l2 = err;
        if(weights[i] < norm_weight && weights[i] != 0)
            norm_weight = weights[i];
    }

    for(unsigned int i = 0; i < poses.size(); i++) {
        double err = l2[i] / norm_l2 + norm_weight / weights[i];
        if(err < best_err) {
            best_err = err;
            best_pose = poses[i];
        }
    }

    return best_pose;*/
}
