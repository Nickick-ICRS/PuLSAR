#ifndef __USEFUL_FUNCTIONS_HPP__
#define __USEFUL_FUNCTIONS_HPP__

#include <geometry_msgs/Quaternion.h>

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

#endif // __USEFUL_FUNCTIONS_HPP__
