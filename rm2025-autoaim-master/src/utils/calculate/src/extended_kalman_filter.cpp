// Copyright 2022 Chen Jun

#include <iostream>
#include "extended_kalman_filter.hpp"

void 
ExtendedKalmanFilter::LoadFormula(const ExtendedKalmanFilter::VecVecFunc &f, const ExtendedKalmanFilter::VecVecFunc &h,
                                  const ExtendedKalmanFilter::VecMatFunc &j_f,
                                  const ExtendedKalmanFilter::VecMatFunc &j_h,
                                  const ExtendedKalmanFilter::VoidMatFunc &u_q,
                                  const ExtendedKalmanFilter::VecMatFunc &u_r, const Eigen::MatrixXd &P_0) {
    f_ = f;
    h_ = h;
    jacobian_f_ = j_f;
    jacobian_h_ = j_h;
    update_Q_ = u_q;
    update_R_ = u_r;
    P_post_ = P_0;
    n_ = static_cast<int>(P_0.rows());
    I_ = Eigen::MatrixXd::Identity(n_, n_);
    x_pri_ = Eigen::VectorXd(n_);
    x_post_ = Eigen::VectorXd(n_);
}

void ExtendedKalmanFilter::SetState(const Eigen::VectorXd &x0) { x_post_ = x0; }

Eigen::MatrixXd ExtendedKalmanFilter::Predict() {
    F_ = jacobian_f_(x_post_), Q_ = update_Q_();

    x_pri_ = f_(x_post_);
    P_pri_ = F_ * P_post_ * F_.transpose() + Q_;

    // handle the case when there will be no measurement before the next predict
    x_post_ = x_pri_;
    P_post_ = P_pri_;

    return x_pri_;
}

Eigen::MatrixXd ExtendedKalmanFilter::Update(const Eigen::VectorXd &z) {
    H_ = jacobian_h_(x_pri_), R_ = update_R_(z);

    K_ = P_pri_ * H_.transpose() * (H_ * P_pri_ * H_.transpose() + R_).inverse();
    x_post_ = x_pri_ + K_ * (z - h_(x_pri_));
    P_post_ = (I_ - K_ * H_) * P_pri_;

    return x_post_;
}
Eigen::MatrixXd ExtendedKalmanFilter::GetPPostMat(){
    return P_post_;
}

