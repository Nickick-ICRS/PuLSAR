#include "maths/kmeans.hpp"

#include "maths/useful_functions.hpp"

#include <cmath>
#include <limits>

using DataSet = std::vector<geometry_msgs::Pose>;

KMeans::KMeans(const DataSet& data) 
    :data_(data), gen_(rd_()), dist_(0, data.size() - 1), scalex_(0), 
    scaley_(0), scaleyaw_(M_PI)
{
    // ctor
}

KMeans::~KMeans() {
    // dtor
}


std::vector<unsigned int> KMeans::run(unsigned int K, unsigned int num_its) 
{
    // Pick some centroids
    means_ = DataSet(K);
    for(auto& p : means_)
        p = data_[dist_(gen_)];
    
    return run(K, means_, num_its);
}

std::vector<unsigned int> KMeans::run(
    unsigned int K, DataSet& init, unsigned int num_its) 
{
    // Find scaling parameters
    for(const auto& p : data_) {
        double x, y;
        x = fabs(p.position.x);
        y = fabs(p.position.y);
        if(x > scalex_) scalex_ = x;
        if(y > scaley_) scaley_ = y;
    }

    means_ = init;
    
    std::vector<unsigned int> clusters(data_.size());
    for(unsigned int it = 0; it < num_its; it++) {
        for(unsigned int i = 0; i < data_.size(); i++) {
            double best_dist = std::numeric_limits<double>::max();
            unsigned int best_clust = 0;
            for(unsigned int c = 0; c < K; c++) {
                double dist = squared_l2(data_[i], means_[c]);
                if(dist < best_dist) {
                    best_dist = dist;
                    best_clust = c;
                }
            }
            clusters[i] = best_clust;
        }
    }

    DataSet new_means(K);
    std::vector<double> new_mean_yaws(K, 0);
    std::vector<unsigned int> counts(K, 0);
    for(unsigned int p = 0; p < data_.size(); p++) {
       const auto c = clusters[p];
       new_means[c].position.x += data_[p].position.x;
       new_means[c].position.y += data_[p].position.y;
       new_mean_yaws[c] += quat_to_yaw(data_[p].orientation);
       counts[c]++;
    }

    for(unsigned int c = 0; c < K; c++) {
        const auto count = std::max<unsigned int>(1, counts[c]);
        means_[c].position.x = new_means[c].position.x / count;
        means_[c].position.y = new_means[c].position.y / count;
        means_[c].orientation = yaw_to_quat(new_mean_yaws[c] / count);
    }
    return clusters;
}

double KMeans::squared_l2(
    const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
    double dx = (a.position.x - b.position.x) * scalex_;
    double dy = (a.position.y - b.position.y) * scaley_;
    double dyaw = quat_to_yaw(a.orientation) - quat_to_yaw(b.orientation);
    if(dyaw > M_PI) dyaw -= 2*M_PI;
    if(dyaw <= -M_PI) dyaw += 2*M_PI;
    dyaw *= scaleyaw_;
    return dx * dx + dy * dy + dyaw * dyaw;
}

std::map<unsigned int, geometry_msgs::Pose> KMeans::get_means() {
    std::map<unsigned int, geometry_msgs::Pose> means;
    int cluster = 0;
    for(const auto& mean : means_)
        means[cluster++] = mean;
    return means;
}
