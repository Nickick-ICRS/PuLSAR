#ifndef __USEFUL_FUNCTIONS_HPP__
#define __USEFUL_FUNCTIONS_HPP__

#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>

/**
 * Converts between a quaternion and yaw.
 *
 * @param q The (normalised) quaternion to convert.
 *
 * @return The yaw of the quaternion (in radians).
 */
double quat_to_yaw(const geometry_msgs::Quaternion& q);

/**
 * Converts between yaw and a quaternion.
 *
 * @param yaw The yaw to convert (in radians).
 *
 * @return A quaternion representation of the yaw.
 */
geometry_msgs::Quaternion yaw_to_quat(double yaw);

/**
 * @brief Calculate the probability of event a occuring.
 *
 * Calculates the probability of event a in the normal distribution with
 * variance b2 and zero mean. Taken from Probabilistic Robotics by Thrun 
 * et al.
 *
 * @param a The event that the probability of occuring is being calculated
 *          for.
 *
 * @param b2 The variance (stddev^2) of the normal distribution.
 *
 * @return p(a) in the normal distribution defined by b2.
 */
double prob_normal_distribution(double a, double b2);

/**
 * Samples a value from a zero mean normal distribution defined by b2.
 *
 * @param b2 The variance of the normal distribution.
 *
 * @return A random number from the distribution.
 */
double sample_normal_distribution(double b2);

/**
 * Calculates the average pose with covariance from a set of poses.
 *
 * @param poses The poses from which to calculate covariance.
 *
 * @return The average pose along with accompanying covariance.
 */
geometry_msgs::PoseWithCovariance calculate_pose_with_covariance(
    const std::vector<geometry_msgs::Pose>& poses);

#endif // __USEFUL_FUNCTIONS_HPP__
