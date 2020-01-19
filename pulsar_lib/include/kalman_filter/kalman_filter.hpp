#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include <math.h>
#include <Eigen/Geometry>

// Define useful matrices and vectors that aren't defined in Eigen by default
namespace Eigen {
    typedef Matrix<float, 5, 1> Vector5f;
    typedef Matrix<float, 5, 5> Matrix5f;
    typedef Matrix<float, 4, 5> Matrix45f;
    typedef Matrix<float, 5, 4> Matrix54f;
}

class KalmanFilter {
public:
    // Q_est - an estimate of the noise in the state process 
    //         (needs to be tuned)
    // P_0 - initial covariance estimate
    KalmanFilter(
        Eigen::Matrix5f Q_est,
        Eigen::Matrix5f P_0=Eigen::Matrix5f::Identity() / 1e9);
    // dtor
    virtual ~KalmanFilter();
    // Update function
    // dt: delta time since last update cycle
    // z_k: measurements from sensors 
    //      [x_dot, theta_dot, x_ddot, theta]
    // R_l: covariance matrix of sensor measurements
    void update(
        const float& dt, const Eigen::Vector4f& z_k,
        const Eigen::Matrix4f& R_k);

    // Get the current state and covariance estimates
    const Eigen::Vector5f& get_state() { return x_k; };
    const Eigen::Matrix5f& get_covariance() { return P_k; };
protected:
    // Function to make the u vector
    void make_u(const float& dt, const Eigen::Vector5f& prev_x);
    // Function to make the F jacobian
    void make_F(const float& dt, const Eigen::Vector5f& prev_x);

    // State vector w.r.t k [x, y, theta, x_dot, theta_dot]
    Eigen::Vector5f x_k;
    // Theoretical model
    Eigen::Vector5f u_k;
    // Linearisation of the mapping from states to measurements w.r.t states
    Eigen::Matrix45f H;
    // Linearisation of predicted state function w.r.t states
    Eigen::Matrix5f F_k;
    // Linearisation of predicted state function w.r.t noise
    Eigen::Matrix5f L_k;
    // Covariance of the kth state estimate
    Eigen::Matrix5f P_k;
    // Covariance of noise in the filter
    Eigen::Matrix5f Q_k;
};

#endif // __KALMAN_FILTER_HPP__
