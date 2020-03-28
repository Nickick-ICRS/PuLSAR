#include "sensor_models/range_cloud_sensor_model.hpp"

RangeCloudSensorModel::RangeCloudSensorModel(
    std::shared_ptr<MapManager> map_man, float history_length,
    float time_resolution, std::string map_frame)
        :tf_buffer(ros::Duration(history_length)), tf2_(tf_buffer_),
        history_length_(history_length), time_resolution_(time_resolution),
        map_man_(map_man), map_frame_(map_frame)
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
        pose_estimates_.insert(pose_estimates_.begin() + best_i);
    // Otherwise place the new measurement after the closest element
    else {
        if(best_i + 1 >= pose_estimates.size())
            pose_estimates_.push_back(p);
        else
            pose_estimates_.insert(pose_estimates.begin() + best_i + 1, p);
    }
}

float RangeCloudSensorModel::model(
    const geometry_msgs::Pose& p, 
    const pcl::PointCloud<pcl::PointXYZI>& c, std::string frame)
{
    float q = 1;
    ros::Time now = ros::Time::now();
    for(const auto& p : c) {
        // Transform the point into the estimated pose coordinate frame, for
        // the closest pose in the history
        ros::Time ptime(p.intensity);
        geometry_msgs::PointStamped pt;
        pt.header.frame_id = frame;
        pt.header.stamp = ptime;
        pt.point.x = p.x;
        pt.point.y = p.y;
        pt.point.z = p.z;

        // Flag to say whether this point is relative to the new pose
        // estimate, or an older one in the past
        bool in_past = true;
        
        try {
            tf_buffer_.transform(pt, pt, map_frame);
        }
        catch(tf2::ExtrapolationException, tf2::LookupException) {
            // If the transform doesn't exist yet, or requires extrapolation
            // into the future then we transform the point with the most
            // recent pose (i.e. the given one)
            pt.point.x += p.position.x;
            pt.point.y += p.position.y;
            pt.point.z += p.position.z;
            double th = 2*acos(p.orientation.w);
            pt.point.x = pt.point.x * cos(th) - pt.point.y * sin(th);
            pt.point.y = pt.point.x * sin(th) + pt.point.y * cos(th);
            pt.header.frame_id = map_frame;
             in_past = false;
        }
        catch(tf2::TransformException& ex) {
            ROS_WARN(ex.what());
            continue;
        }

        // Compute the ideal measurement via raycasting
        // If the point is old then we can't include the positions of other
        // robots, so use the simplified map
        if(in_past) {
            // TODO
        }
        // Otherwise we can consider positions of other robots within
        // the swarm
        else {
            // TODO
        }
        // Now process how likely the point is based on how far it is from
        // an occupied area on the map
        float prob = zhit_*phit(pt | p, map) + zshort_*pshort(pt | p, map)
                   + zmax_*pmax(pt | p, map) + zrand_*prand(pt | p, map);

        // Older points are less reliable, so increase the probability of
        // them being 'correct' such that they have less of an effect
        // on the overall probability
        auto dur = now - ptime;
        frac = (dur.sec + dur.nsec / 1e9) / history_length;
        prob = ztime_ * frac + (1-ztime) * prob;

        // Update the total probability
        q *= prob;
    }
    return 1.0;
}

float RangeCloudSensorModel::phit(const geometry_msgs::PointStamped& z) {
    double dist = sqrt(z.point.x*z.point.x + z.point.y * z.point.y);
    if(dist > range_max_ || dist < 0) return 0;
    // Helper lambda
    static auto N = [this](double z, double ideal_z, double sigma2) {
        double a = 1/sqrt(2*M_PI*sigma2);
        double b = exp(-0.5*pow(z-ideal_z, 2)/sigma2);
        return a*b;
    };
    
    // Simpson's rule in a helper lambda
    static auto integral = [this, &N](
        double z, double ideal_z, double sigma2)
    {
        const int NUM_DIVISIONS = 2*10;
        double delta = max_range_ / (double)NUM_DIVISIONS;
        double area = 0;
        for(double zi = 0; zi < max_range_; zi += 2 * delta) {
            area += N(zi) + 4 * N(zi+delta) + N(zi+2*delta);
        }
        area *= delta/3;
        return area;
    };

    static double eta = 1/integral(dist, ideal_dist, sigma2_);
    return eta * N(z, ideal_z, sigma2_);
}

float RangeCloudSensorModel::pshort(const geometry_msgs::PointStamped& z) {
    double dist = sqrt(z.point.x*z.point.x + z.point.y * z.point.y);
    if(0 <= dist and dist <= ideal_dist) {
        double eta - 1/(1-exp(-lamshort_*ideal_dist));
        return eta * lamshort_ * exp(-lamshort_*dist);
    }
    return 0;
}

float RangeCloudSensorModel::pmax(const geometry_msgs::PointStamped& z) {
    double dist = sqrt(z.point.x*z.point.x + z.point.y * z.point.y);
    if(dist >= range_max_) return 1;
    return 0;
}

float RangeCloudSensorModel::prand(const geometry_msgs::PointStamped& z) {
    double dist = sqrt(z.point.x*z.point.x + z.point.y * z.point.y);
    if(dist < 0 || dist >= range_max_) return 0;
    return 1 / range_max_;
}
