// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <eigen3/Eigen/Dense>
#include <functional>
#include "Eigen/src/Core/Matrix.h"

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() = default;

    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
    using VoidMatFunc = std::function<Eigen::MatrixXd()>;

    void LoadFormula(
            const VecVecFunc &f, const VecVecFunc &h, const VecMatFunc &j_f, const VecMatFunc &j_h,
            const VoidMatFunc &u_q, const VecMatFunc &u_r, const Eigen::MatrixXd &P_0);

    // Set the initial state
    void SetState(const Eigen::VectorXd &x0);

    Eigen::MatrixXd GetPPostMat();

    // Compute a predicted state
    Eigen::MatrixXd Predict();

    // Update the estimated state based on measurement
    Eigen::MatrixXd Update(const Eigen::VectorXd &z);

private:
    // Process nonlinear vector function
    VecVecFunc f_;
    // Observation nonlinear vector function
    VecVecFunc h_;
    // Jacobian of f()
    VecMatFunc jacobian_f_;
    Eigen::MatrixXd F_;
    // Jacobian of h()
    VecMatFunc jacobian_h_;
    Eigen::MatrixXd H_;
    // Process noise covariance matrix
    VoidMatFunc update_Q_;
    Eigen::MatrixXd Q_;
    // Measurement noise covariance matrix
    VecMatFunc update_R_;
    Eigen::MatrixXd R_;

    // Priori error estimate covariance matrix
    Eigen::MatrixXd P_pri_;
    // Posteriori error estimate covariance matrix
    Eigen::MatrixXd P_post_;

    // Kalman gain
    Eigen::MatrixXd K_;

    // System dimensions
    int n_;

    // N-size identity
    Eigen::MatrixXd I_;

    // Priori state
    Eigen::VectorXd x_pri_;
    // Posteriori state
    Eigen::VectorXd x_post_;
};

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
