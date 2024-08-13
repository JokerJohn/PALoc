//
// Created by xchu on 21/3/2023.
//

#ifndef SRC_GRAVITY_KALMAN_FILTER_H
#define SRC_GRAVITY_KALMAN_FILTER_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

class GravityKalmanFilter {
public:
    VectorXd x;  // State vector
    MatrixXd P;  // Covariance matrix

    GravityKalmanFilter(int state_dim) {
        x = VectorXd::Zero(state_dim);
        P = MatrixXd::Identity(state_dim, state_dim);
    }

    void predict(const MatrixXd &F, const MatrixXd &Q) {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const VectorXd &z, const MatrixXd &H, const MatrixXd &R) {
        VectorXd y = z - H * x;
        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();
        x = x + K * y;
        MatrixXd I = MatrixXd::Identity(x.size(), x.size());
        P = (I - K * H) * P;
    }
};

// usage
//int main() {
//    // Simulate the accelerometer and gyroscope data from STIM300.
//    std::vector<Vector3d> accel_data = {/*...*/};
//    std::vector<Vector3d> gyro_data = {/*...*/};
//    double dt = 0.01; // Time step
//    // Initialize variables for the current orientation and gravity estimate.
//    Quaterniond q = Quaterniond::Identity();
//    KalmanFilter kalman_filter(3);
//    // Process the accelerometer and gyroscope data to estimate the gravity direction.
//    for (size_t i = 0; i < accel_data.size(); ++i) {
//        // Integrate the gyroscope data to update the current orientation.
//        Vector3d omega = gyro_data[i];
//        q = integrateGyroscope(q, omega, dt);
//        // Compensate for the gravity component in the accelerometer data.
//        Vector3d accel_body = accel_data[i];
//        Vector3d accel_global = q * accel_body - Vector3d(0, 0, -9.81);
//// Set up the Kalman filter matrices.
//        MatrixXd F = MatrixXd::Identity(3, 3);
//        MatrixXd H = MatrixXd::Identity(3, 3);
//        MatrixXd Q = MatrixXd::Identity(3, 3) * 1e-4; // Process noise covariance
//        MatrixXd R = MatrixXd::Identity(3, 3) * 1e-2; // Measurement noise covariance
//// Perform the Kalman filter prediction step.
//        kalman_filter.predict(F, Q);
//// Perform the Kalman filter update step using the compensated accelerometer data.
//        kalman_filter.update(accel_global, H, R);
//// Get the gravity estimate from the Kalman filter state.
//        Vector3d gravity_estimate = kalman_filter.x.normalized();
//        std::cout << "Gravity estimate: " << gravity_estimate.transpose() << std::endl;
//    }
//    return 0;
//}

#endif //SRC_GRAVITY_KALMAN_FILTER_H
