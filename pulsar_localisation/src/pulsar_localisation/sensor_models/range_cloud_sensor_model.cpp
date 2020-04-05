#include "sensor_models/range_cloud_sensor_model.hpp"

double RangeCloudSensorModel::lamshort_;
double RangeCloudSensorModel::sigmahit_;
double RangeCloudSensorModel::ztime_;
double RangeCloudSensorModel::zhit_;
double RangeCloudSensorModel::zshort_;
double RangeCloudSensorModel::zmax_;
double RangeCloudSensorModel::zrand_;

std::shared_ptr<MapManager> RangeCloudSensorModel::map_man_;
float RangeCloudSensorModel::history_length_;
float RangeCloudSensorModel::time_resolution_;

RangeCloudSensorModel::RangeCloudSensorModel(std::string map_frame)
    :tf_buffer_(ros::Duration(history_length_)), tf2_(tf_buffer_),
    map_frame_(map_frame)
{
    // ctor
}

RangeCloudSensorModel::~RangeCloudSensorModel() {
    // dtor
}

void RangeCloudSensorModel::add_pose_estimate(
    const geometry_msgs::PoseWithCovarianceStamped& p)
{
    // Store current time so we can remove any old time points
    ros::Time now = ros::Time::now();
    // Check if we have a pose estimate with a similar time slot
    int best_i = 0;
    float best_diff = 1e10;
    for(int i = 0; i < pose_estimates_.size(); i++) {
        if(pose_estimates_[i].header.stamp < 
           now - ros::Duration(history_length_))
        {
            pose_estimates_.erase(pose_estimates_.begin()+i);
            i--;
        }
        else {
            auto diff = p.header.stamp - pose_estimates_[i].header.stamp;
            float diff_s = diff.sec + diff.nsec / 1e9;
            if(fabsf(diff_s) < fabsf(best_diff)) {
                best_diff = diff_s;
                best_i = i;
            }
        }
    }

    // If the difference is very small then replace the closest element
    if(fabsf(best_diff) < time_resolution_) {
        pose_estimates_[best_i] = p;
    }
    // If the difference is negative then place the new measurement before
    // the closest element
    else if(best_diff < 0)
        pose_estimates_.insert(pose_estimates_.begin() + best_i, p);
    // Otherwise place the new measurement after the closest element
    else {
        if(best_i + 1 >= pose_estimates_.size())
            pose_estimates_.push_back(p);
        else
            pose_estimates_.insert(pose_estimates_.begin() + best_i + 1, p);
    }
}

float RangeCloudSensorModel::model(
    const geometry_msgs::Pose& p, 
    const std::vector<sensor_msgs::Range>& c,
    std::string robot_name, std::string base_link_frame)
{
    float q = 1;
    ros::Time now = ros::Time::now();
    for(const auto& z : c) {
        // Transform the point into the estimated pose coordinate frame, for
        // the closest pose in the history
        geometry_msgs::PointStamped pt;
        pt.header.frame_id = z.header.frame_id;
        pt.header.stamp = z.header.stamp;
        pt.point.x = z.range;

        // Flag to say whether this point is relative to the new pose
        // estimate, or an older one in the past
        bool in_past = true;
        
        try {
            tf_buffer_.transform(pt, pt, map_frame_);
        }
        catch(const tf2::TransformException* ex) {
            if(!dynamic_cast<const tf2::ExtrapolationException*>(ex) &&
               !dynamic_cast<const tf2::LookupException*>(ex))
            {
                ROS_WARN_STREAM(ex->what());
                continue;
            }
            // If the transform doesn't exist yet, or requires extrapolation
            // into the future then we transform the point with the most
            // recent pose (i.e. the given one)

            // First try to transform into the base_link frame though
            try {
                tf_buffer_.transform(pt, pt, base_link_frame);
            }
            catch(const tf2::TransformException& ex) {
                ROS_WARN_STREAM(ex.what());
                continue;
            }

            pt.point.x += p.position.x;
            pt.point.y += p.position.y;
            pt.point.z += p.position.z;
            double th = 2*acos(p.orientation.w);
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
            ang = 2*acos(trans.transform.rotation.w) 
                + 2*acos(p.orientation.w);
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

        // Older points are less reliable, so increase the probability of
        // them being 'correct' such that they have less of a negative
        // effect on the overall probability
        auto dur = now - z.header.stamp;
        double frac = (dur.sec + dur.nsec / 1e9) / history_length_;
        prob = ztime_ * frac + (1-ztime_) * prob;

        // Update the total probability
        q *= prob;
    }
    return 1.0;
}

float RangeCloudSensorModel::phit(
    const sensor_msgs::Range& z, double ideal_z)
{
    if(z.range > z.max_range || z.range < z.min_range) return 0;
    // Helper lambda
    static auto N = [this](double z, double ideal_z, double sigmahit) {
        double a = 1/sqrt(2*M_PI*sigmahit);
        double b = exp(-0.5*pow(z-ideal_z, 2)/sigmahit);
        return a*b;
    };
    
    // Simpson's rule in a helper lambda
    static auto integral = [this](
        const sensor_msgs::Range& z, double ideal_z, double sigmahit)
    {
        const int NUM_DIVISIONS = 2*10;
        double delta = z.max_range / (double)NUM_DIVISIONS;
        double area = 0;
        for(double zi = 0; zi < z.max_range; zi += 2 * delta) {
            area += N(zi, ideal_z, sigmahit)
                 + 4 * N(zi+delta, ideal_z, sigmahit)
                 + N(zi+2*delta, ideal_z, sigmahit);
        }
        area *= delta/3;
        return area;
    };

    double eta = 1/integral(z, ideal_z, sigmahit_);
    return eta * N(z.range, ideal_z, sigmahit_);
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
