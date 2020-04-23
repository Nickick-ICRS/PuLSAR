#include "sensor_models/range_cloud_sensor_model.hpp"

#include "maths/useful_functions.hpp"

#include <cmath>

double RangeCloudSensorModel::lamshort_;
double RangeCloudSensorModel::sigmahit_;
double RangeCloudSensorModel::ztime_;
double RangeCloudSensorModel::zhit_;
double RangeCloudSensorModel::zshort_;
double RangeCloudSensorModel::zmax_;
double RangeCloudSensorModel::zrand_;

int   RangeCloudSensorModel::cycle_sensor_readings_;
float RangeCloudSensorModel::time_resolution_;

RangeCloudSensorModel::RangeCloudSensorModel(
    std::string map_frame, const std::shared_ptr<MapManager>& map_man)
    
    :tf_buffer_(ros::Duration(cycle_sensor_readings_)), tf2_(tf_buffer_),
    map_frame_(map_frame), map_man_(map_man)
{
    // ctor
}

RangeCloudSensorModel::~RangeCloudSensorModel() {
    // dtor
}

float RangeCloudSensorModel::model(
    const geometry_msgs::Pose& p, 
    const std::vector<sensor_msgs::Range>& c,
    std::string robot_name, std::string base_link_frame)
{
    float q = 1;
    ros::Time now = ros::Time::now();
    for(const auto& z : c) {
        // Temporary fix, but sometimes points don't have a frame_id???
        // Why ???
        if(z.header.frame_id == "")
            continue;
        // Transform the sensor pose into the estimated pose coordinate 
        // frame, for the closest pose in the history
        geometry_msgs::PointStamped pt2;
        geometry_msgs::PointStamped pt;
        pt2.header.frame_id = z.header.frame_id;
        pt2.header.stamp = z.header.stamp;

        // Flag to say whether this pose is relative to the new pose
        // estimate, or an older one in the past
        bool in_past = false;
        
        try {
            tf_buffer_.transform(pt2, pt, map_frame_);
        }
        catch(const tf2::ConnectivityException& ex) {
            ROS_WARN_STREAM(ex.what());
        }
        catch(const tf2::InvalidArgumentException& ex) {
            ROS_WARN_STREAM(ex.what());
        }
        catch(const tf2::TimeoutException& ex) {
            ROS_WARN_STREAM(ex.what());
        }
        catch(const tf2::TransformException& ex) {
            // If the transform doesn't exist yet, or requires extrapolation
            // into the future then we transform the point with the most
            // recent pose (i.e. the given one)
            
            // First try to transform into the base_link frame though
            try {
                tf_buffer_.transform(pt2, pt, base_link_frame);
            }
            catch(const tf2::TransformException& ex) {
                ROS_WARN_STREAM(ex.what());
                continue;
            }

            pt.point.x += p.position.x;
            pt.point.y += p.position.y;
            pt.point.z += p.position.z;
            double th = quat_to_yaw(p.orientation);
            pt.point.x = pt.point.x * cos(th) - pt.point.y * sin(th);
            pt.point.y = pt.point.x * sin(th) + pt.point.y * cos(th);
            pt.header.frame_id = map_frame_;
            in_past = false;
        }

        // Compute the ideal measurement via raycasting
        // If the point is old then we can't include the positions of other
        // robots, so use the simplified map
        double ideal_z;
        double ang;
        try {
            const auto trans = tf_buffer_.lookupTransform(
                base_link_frame, z.header.frame_id, ros::Time(0));
            ang = quat_to_yaw(trans.transform.rotation) 
                + quat_to_yaw(p.orientation);
        }
        catch(tf2::TransformException& ex) {
            ROS_WARN_STREAM(ex.what());
            continue;
        }
        if(in_past) {
            ideal_z = map_man_->cone_cast_plain_map(
                pt.point, ang, z.field_of_view);
        }
        // Otherwise we can consider positions of other robots within
        // the swarm
        else {
            ideal_z = map_man_->cone_cast_with_bots(
                pt.point, ang, z.field_of_view, robot_name);
        }
        // Now process how likely the point is based on how far it is from
        // an occupied area on the map
        float prob = zhit_*phit(z, ideal_z) + zshort_*pshort(z, ideal_z)
                   + zmax_*pmax(z) + zrand_*prand(z);
        prob /= zhit_+zshort_+zmax_+zrand_;

        // Update the total probability
        if(!std::isfinite(prob)) {
            ROS_ERROR_STREAM("nan");
            ROS_INFO_STREAM("prob: " << prob << " phit: " << phit(z, ideal_z)
                << " pshort " << pshort(z, ideal_z) << " pmax " << pmax(z)
                << " prand " << prand(z) << " z " << z.range << " iz " << ideal_z);
        }
            q *= prob;
    }
    return q;
}

float RangeCloudSensorModel::phit(
    const sensor_msgs::Range& z, double ideal_z)
{
    if(z.range > z.max_range || z.range < z.min_range) return 0;

    // Indefinite integral pre calculated via wolfram alpha
    double a = 0.5*erf(sqrt(0.5)*(z.max_range - ideal_z)/sigmahit_);
    double b = 0.5*erf(sqrt(0.5)*(0 - ideal_z)/sigmahit_);
    double eta = 1.0/(a - b);

    if(std::isinf(eta)) {
        // A large number
        eta = 1e50;
    }

    double sigmahit2 = pow(sigmahit_, 2);
    a = 1/sqrt(2*M_PI*sigmahit2);
    b = exp(-0.5*pow(z.range-ideal_z, 2)/sigmahit2);
    double n = a*b;

    return eta * n;
}

float RangeCloudSensorModel::pshort(
    const sensor_msgs::Range& z, double ideal_z)
{
    if(z.min_range <= z.range && z.range <= ideal_z) {
        double eta = 1/(1-exp(-lamshort_*ideal_z));
        return eta * lamshort_ * exp(-lamshort_*z.range);
    }
    return 0;
}

float RangeCloudSensorModel::pmax(const sensor_msgs::Range& z) {
    if(z.range >= z.max_range) return 1;
    return 0;
}

float RangeCloudSensorModel::prand(const sensor_msgs::Range& z) {
    if(z.range < z.min_range || z.range >= z.max_range) return 0;
    return 1 / (z.max_range - z.min_range);
}
