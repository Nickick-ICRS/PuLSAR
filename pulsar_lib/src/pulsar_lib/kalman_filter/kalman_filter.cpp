#include "kalman_filter/kalman_filter.hpp"
#include <ros/ros.h>

KalmanFilter::KalmanFilter(Eigen::Matrix5f Q_est, Eigen::Matrix5f P_0) {
    // H is an identity matrix at the top right and zero elsewhere
    H = Eigen::Matrix45f::Zero();
    H.topRightCorner(3, 3) = Eigen::Matrix3f::Identity();
    // F_k is mostly an identity matrix
    F_k = Eigen::Matrix5f::Identity();
    // L_k is an identity matrix
    L_k = Eigen::Matrix5f::Identity();

    // Initial state is assumed to be zero
    x_k = Eigen::Vector5f::Zero();
    u_k = Eigen::Vector5f::Zero();

    // Parameters
    P_k = P_0;
    Q_k = Q_est;
}

KalmanFilter::~KalmanFilter() {
    // dtor
}

void KalmanFilter::update(
    const float& dt, const Eigen::Vector4f& z_k,
    const Eigen::Matrix4f& R_k)
{
    ROS_WARN_STREAM("dt: " << dt << " z_k: " << z_k << " R_k: " << R_k);
    // Store previous states
    auto x_k_1 = x_k;
    auto P_k_1 = P_k;

    /* First the prediction step */

    // Update u and F
    make_u(dt, x_k_1);
    make_F(dt, x_k_1);

    // Predicted mean x of state k given k-1
    x_k = x_k_1 + u_k;
    // Predicted covariance of state k given k-1
    P_k = F_k * P_k_1 * F_k.transpose() + L_k * Q_k * L_k.transpose();

    ROS_WARN_STREAM("Predicted x_k: " << x_k);

    /* Now the measurement update step */

    // Calculate the measurement residual
    Eigen::Vector4f y_k = z_k - H * x_k;

    // Calculate the residual covariance
    Eigen::Matrix4f S_k = H * P_k * H.transpose() + R_k;

    // Calculate the Kalman gain
    // Because S_k is a 4x4 we can invert directly - if another measurement
    // is added we'll need to use decompositions e.g. PartialPivLU
    Eigen::Matrix54f K = P_k * H.transpose() * S_k.inverse();

    // Finally, update the state and covariance
    x_k = x_k + K * y_k;
    P_k = (Eigen::Matrix5f::Identity() - K * H) * P_k;
    ROS_ERROR_STREAM("Updated x_k: " << x_k);
}

void KalmanFilter::make_u(const float& dt, const Eigen::Vector5f& prev_x) {
    u_k[2] = prev_x[4]*dt;
    u_k[0] = prev_x[3] * cos((prev_x[2] + u_k[2]) / 2.f) * dt;
    u_k[1] = prev_x[3] * sin((prev_x[2] + u_k[2]) / 2.f) * dt;
}

void KalmanFilter::make_F(const float& dt, const Eigen::Vector5f& prev_x) {
    F_k(3, 0) = cos(prev_x[2]) * dt;
    F_k(3, 1) = sin(prev_x[2]) * dt;
    F_k(2, 0) = - prev_x[3] * F_k(3, 1);
    F_k(2, 1) = prev_x[3] * F_k(3, 0);
    F_k(4, 2) = dt;
}
