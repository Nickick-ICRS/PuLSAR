#ifndef __KMEANS_HPP__
#define __KMEANS_HPP__

#include <vector>
#include <random>

#include <geometry_msgs/Pose.h>

class KMeans {
public:
    /**
     * Constructor with data. Just call run.
     *
     * @param data The data set to be clustered.
     */
    KMeans(const std::vector<geometry_msgs::Pose>& data);
    ~KMeans();

    /**
     * Run the KMeans clustering algorithm.
     *
     * @param K The number of clusters to find.
     *
     * @param init The initial centroids to use.
     *
     * @param num_its The number of iterations to perform.
     *
     * @return A vector containing the cluster for each pose in the given
     *         data set. Cluster numbers range from 0 to K.
     */
    std::vector<unsigned int> run(
        unsigned int K, std::vector<geometry_msgs::Pose>& init,
        unsigned int num_its=200);

    /**
     * Run the KMeans clustering algorithm.
     *
     * @param K The number of clusters to find.
     *
     * @param num_its The number of iterations to perform.
     *
     * @return A vector containing the cluster for each pose in the given
     *         data set. Cluster numbers range from 0 to K.
     */
    std::vector<unsigned int> run(
        unsigned int K, unsigned int num_its=200);

    /**
     * Gets the means of the clusters. Only returns valid results after
     * the run function has been run!
     *
     * @return Mean cluster positions.
     */
    std::map<unsigned int, geometry_msgs::Pose> get_means();
private:
    /**
     * Calculates the squared l2 distance between two points.
     *
     * @param a The first point.
     *
     * @param b The second point.
     *
     * @return The squared l2 distance.
     */
    double squared_l2(
        const geometry_msgs::Pose& a, const geometry_msgs::Pose& b);

    const std::vector<geometry_msgs::Pose>& data_;
    std::vector<geometry_msgs::Pose> means_;

    double scalex_;
    double scaley_;
    double scaleyaw_;

    std::random_device rd_;
    std::mt19937_64 gen_;
    std::uniform_int_distribution<unsigned int> dist_;
};

#endif // __KMEANS_HPP__
