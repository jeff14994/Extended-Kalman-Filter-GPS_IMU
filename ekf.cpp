#include "ekf.h"

void ExtendedKalmanFilter::initialize(const Eigen::Vector3d& x, double xy_obs_noise_std, double yaw_rate_noise_std, double forward_velocity_noise_std, double initial_yaw_std){
    x_ = Eigen::Vector3d(x);


    P << xy_obs_noise_std * xy_obs_noise_std, 0, 0,
            0, xy_obs_noise_std * xy_obs_noise_std, 0,
            0, 0, initial_yaw_std * initial_yaw_std;

    P_ = Eigen::Matrix3d(P);

    Q << xy_obs_noise_std * xy_obs_noise_std, 0,
            0, xy_obs_noise_std * xy_obs_noise_std;

    R << forward_velocity_noise_std * forward_velocity_noise_std, 0, 0,
            0, forward_velocity_noise_std * forward_velocity_noise_std, 0,
            0, 0, yaw_rate_noise_std * yaw_rate_noise_std;
}

void ExtendedKalmanFilter::update(const Eigen::Vector2d& z) {
    Eigen::Matrix<double, 2, 3> H;
    H << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0;

    Eigen::Matrix<double,3,3> I;
    I<< 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;

    Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * (H * P_ * H.transpose() + Q).inverse();

    Eigen::Vector2d z_ = x_.head<2>();
    x_ = x_ + K * (z - z_);
    // P_ = P_ - K * H * P_;
    P_ = (I - (K * H)) * P_;
}

void ExtendedKalmanFilter::propagate(const Eigen::Vector2d& u, double dt) {
    double v = u[0];
    double omega = u[1];
    double theta = x_[2];
    double dtheta = omega * dt;

    double dx, dy;
    Eigen::Matrix3d G;

    if (std::abs(omega) < 1e-6) {
        // Straight-line motion (omega ≈ 0): avoid division by zero
        dx = v * cos(theta) * dt;
        dy = v * sin(theta) * dt;

        G << 1.0, 0.0, -v * sin(theta) * dt,
             0.0, 1.0,  v * cos(theta) * dt,
             0.0, 0.0, 1.0;
    } else {
        // Circular arc motion
        double r = v / omega;
        dx = -r * sin(theta) + r * sin(theta + dtheta);
        dy =  r * cos(theta) - r * cos(theta + dtheta);

        G << 1.0, 0.0, -r * cos(theta) + r * cos(theta + dtheta),
             0.0, 1.0, -r * sin(theta) + r * sin(theta + dtheta),
             0.0, 0.0, 1.0;
    }

    x_[0] += dx;
    x_[1] += dy;
    x_[2] += dtheta;

    P_ = G * P_ * G.transpose() + R;
}


//const Eigen::Vector3d& ExtendedKalmanFilter::getState() const {
//    return x_;
//}
//
//const Eigen::Matrix3d& ExtendedKalmanFilter::getCovariance() const {
//    return P_;
//}
