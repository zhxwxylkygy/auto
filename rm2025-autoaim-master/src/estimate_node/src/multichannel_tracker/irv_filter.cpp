// Copyright 2022 Chen Jun

#include "estimate_node/multichannel_tracker/irv_tracker/irv_filter.h"
#include <ceres/jet.h>

#include <cfloat>
#include <cmath>
#include <memory>
#include "Eigen/src/Core/Matrix.h"
#include "angles/angles.h"
#include "calculate/basic_calculate.h"
#include "vofa_bridge/vofa_bridge.h"


namespace MT::IRVT {
IRVFilter::IRVFilter(bool is_hero) {
    auto f = [this](const Eigen::VectorXd& x) {
        Eigen::VectorXd x_new = x;
        auto d_t = ekf_formula_server_.d_t;
        x_new(0) += x(1) * d_t;
        x_new(2) += x(3) * d_t;
        x_new(4) += x(5) * d_t;
        x_new(6) += x(7) * d_t;
        return x_new;
    };

    auto jacobian_f = [this](const Eigen::VectorXd&) {
        Eigen::MatrixXd f(9, 9);
        auto d_t = ekf_formula_server_.d_t;
        f << 1, d_t, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, d_t, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, d_t, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, d_t, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

        return f;
    };

    auto h = [](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(4);
        double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
        double xa = xc - r * cos(yaw);
        double ya = yc - r * sin(yaw);
        double za = x(4);
        double distance = std::sqrt(pow(xa, 2) + pow(ya, 2) + pow(za, 2));
        z(0) = distance;                  // a_distance
        z(1) = std::acos(za / distance);  // a_theta
        z(2) = atan2(ya, xa);             // a_phi
        z(3) = x(6);                      // yaw
        return z;
    };

    auto j_h = [this](const Eigen::VectorXd& x_vec) {
        Eigen::MatrixXd h(4, 9);  // Jacobian matrix
        ceres::Jet<double, 9> x_jet[9];
        for (int i = 0; i < 9; ++i) {
            x_jet[i].a = x_vec(i);
            x_jet[i].v[i] = 1;
        }

        ceres::Jet<double, 9> y_jet[4];
        ekf_h_cost_function_a(x_jet, y_jet);
        // double x[9] = {x_vec(0), x_vec(1), x_vec(2), x_vec(3), x_vec(4), x_vec(5), x_vec(6), x_vec(7), x_vec(8)};
        // double* x_blocks[1] = {x};
        // double residuals[4];
        // double jacobians[36] = {};
        // double* jacobians_blocks[1] = {jacobians};
        // h_cost_function_a->Evaluate(x_blocks, residuals, jacobians_blocks);
        // h << jacobians[0], jacobians[1], jacobians[2], jacobians[3], jacobians[4], jacobians[5], jacobians[6],
        //     jacobians[7], jacobians[8], jacobians[9], jacobians[10], jacobians[11], jacobians[12], jacobians[13],
        //     jacobians[14], jacobians[15], jacobians[16], jacobians[17], jacobians[18], jacobians[19], jacobians[20],
        //     jacobians[21], jacobians[22], jacobians[23], jacobians[24], jacobians[25], jacobians[26], jacobians[27],
        //     jacobians[28], jacobians[29], jacobians[30], jacobians[31], jacobians[32], jacobians[33], jacobians[34],
        //     jacobians[35];

        h = Eigen::Matrix<double, 4, 9>::Zero();
        for (int i = 0; i < 4; ++i) {
            h.block(i, 0, 1, 9) = y_jet[i].v.transpose();
        }
        return h;
    };
    if (is_hero) {
        ekf_formula_server_.q_xyz_2 = pow(param_.hero_q_xyz, 2);
        ekf_formula_server_.q_yaw_2 = pow(param_.hero_q_yaw, 2);
        ekf_formula_server_.q_r_2 = pow(param_.hero_q_r, 2);
    } else {
        ekf_formula_server_.q_xyz_2 = pow(param_.q_xyz, 2);
        ekf_formula_server_.q_yaw_2 = pow(param_.q_yaw, 2);
        ekf_formula_server_.q_r_2 = pow(param_.q_r, 2);
    }

    auto update_q = [this]() {
        Eigen::MatrixXd q(9, 9);
        auto d_t = ekf_formula_server_.d_t;
        double t = d_t, x = ekf_formula_server_.q_xyz_2, y = ekf_formula_server_.q_yaw_2, r = ekf_formula_server_.q_r_2;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
        double q_r = pow(t, 4) / 4 * r;

        //      xc          v_xc        yc      v_yc        za      v_za        yaw     v_yaw       r
        q << q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_x_x, q_x_vx, 0, 0, 0, 0,
            0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0, q_x_vx, q_vx_vx, 0,
            0, 0, 0, 0, 0, 0, 0, 0, q_y_y, q_y_vy, 0, 0, 0, 0, 0, 0, 0, q_y_vy, q_vy_vy, 0, 0, 0, 0, 0, 0, 0, 0, 0, q_r;

        return q;
    };
    if (is_hero) {
        ekf_formula_server_.r_distance_factor_2 = std::pow(param_.hero_r_distance_factor, 2);
        ekf_formula_server_.r_theta_2 = std::pow(param_.hero_r_theta, 2);
        ekf_formula_server_.r_phi_2 = std::pow(param_.hero_r_phi, 2);
        ekf_formula_server_.r_yaw_2 = std::pow(param_.hero_r_yaw, 2);
    } else {
        ekf_formula_server_.r_distance_factor_2 = std::pow(param_.r_distance_factor, 2);
        ekf_formula_server_.r_theta_2 = std::pow(param_.r_theta, 2);
        ekf_formula_server_.r_phi_2 = std::pow(param_.r_phi, 2);
        ekf_formula_server_.r_yaw_2 = std::pow(param_.r_yaw, 2);
    }
    auto update_r = [this](const Eigen::VectorXd& z) {
        Eigen::DiagonalMatrix<double, 4> r;
        r.diagonal() << ekf_formula_server_.r_distance_factor_2 * z[0] * z[0], ekf_formula_server_.r_theta_2,
            ekf_formula_server_.r_phi_2, ekf_formula_server_.r_yaw_2;
        return r;
    };
    // P - error estimate covariance matrix
    Eigen::DiagonalMatrix<double, 9> p_0;
    p_0.setIdentity();
    ekf_.LoadFormula(f, h, jacobian_f, j_h, update_q, update_r, p_0);

    h_func_ = h;
}

void IRVFilter::Init(const std::vector<std::shared_ptr<FP::ArmorInfo>>& input_armors,
                     const RM::TargetArmorNum input_target_armor_num) {
    if (input_armors.empty()) {
        return;
    }

    matched_armor_ = input_armors[0];

    InitEKF(matched_armor_);
    //        std::cout << "[armor_tracker]" << "Init EKF!" << std::endl;

    tracked_id_ = matched_armor_->armor_id;
    tracker_state_ = DETECTING;

    UpdateArmorsNum(input_target_armor_num);
}

void IRVFilter::Update(const std::vector<std::shared_ptr<FP::ArmorInfo>>& input_armors, double d_t) {
    lost_count_threshold_ = static_cast<int>(param_.lost_duration_threshold / d_t);
    ekf_formula_server_.d_t = d_t;
    // KF predict
    Eigen::VectorXd ekf_prediction = ekf_.Predict();
    //        std::cout << ekf_prediction << std::endl;

    //        v_4.Send((float)1, (float)2, (float)3, (float)4);

    //        v_4.Send((float) angles::to_degrees(ekf_prediction(6)),
    //                 (float) angles::to_degrees((float) ekf_prediction(7)), (float) ekf_prediction(0),
    //                 (float) ekf_prediction(2), (float) ekf_prediction(4),
    //                 (float) ekf_prediction(1),
    //                 (float) ekf_prediction(3), (float) ekf_prediction(5),
    //                 (float) ekf_prediction(8));

    //        Eigen::Vector3d dtp = {ekf_prediction(0), ekf_prediction(1), ekf_prediction(2)};
    //        Eigen::Vector3d xyz = DTP2XYZ(dtp);
    //        Eigen::Vector4d xyz_yaw = {xyz(0), xyz(1), xyz(2), ekf_prediction(3)};

    //        Eigen::Vector3d ekf_prediction_xyz =
    //        ekf_prediction = DTP2XYZ(ekf_prediction);

    bool matched = false;

    target_state = ekf_prediction;

    if (!input_armors.empty()) {
        int same_id_armors_count = 0;
        auto predicted_position = GetArmorPositionFromState(ekf_prediction);
        auto min_position_diff = DBL_MAX;
        auto yaw_diff = DBL_MAX;

        // chenjun 的最小距离�?�甲板匹�? 为了保证原生代码完整�? 故没有进行重�?
        for (const auto& armor : input_armors) {
            if (armor->armor_id == tracked_id_) {
                same_id_armors_count++;

                Eigen::Vector3d position_vec(armor->geographic_coord_pose.x, armor->geographic_coord_pose.y,
                                             armor->geographic_coord_pose.z);
                double position_diff = (predicted_position - position_vec).norm();

                if (position_diff < min_position_diff) {
                    min_position_diff = position_diff;
                    yaw_diff = abs(angles::normalize_angle(armor->geographic_coord_yaw) - ekf_prediction(6));
                    matched_armor_ = armor;
                }
            }
        }
        //            v_4.Send((float)angles::to_degrees(tracked_armor.yaw));

        //            info_position_diff = min_position_diff;
        //            info_yaw_diff = yaw_diff;

        //            v_4.Send((float)min_position_diff, (float)yaw_diff);

        if (min_position_diff < param_.max_match_distance_diff && yaw_diff < param_.max_match_yaw_diff) {
            // if (true) {
            matched = true;
            Eigen::Vector3d p_vec = {matched_armor_->geographic_coord_pose.x, matched_armor_->geographic_coord_pose.y,
                                     matched_armor_->geographic_coord_pose.z};

            double measured_yaw = SerializeArmorYawAngle(matched_armor_->geographic_coord_yaw);

            auto dtp = XYZ2DTP(p_vec);

            Eigen::VectorXd z_pre = h_func_(ekf_prediction);

            auto after_process_theta = get_closest(dtp.y(), z_pre(1), 2 * M_PI);
            auto after_process_phi = get_closest(dtp.z(), z_pre(2), 2 * M_PI);

            auto measurement = Eigen::Vector4d(dtp.x(), after_process_theta, after_process_phi, measured_yaw);

            target_state = ekf_.Update(measurement);

        } else if (yaw_diff > param_.max_match_yaw_diff && min_position_diff < param_.max_match_distance_diff) {
            HandleArmorJump(matched_armor_);
        } else {
            std::cout << "[armor_tracker]"
                      << "No matched armor found!" << std::endl;
        }
    }

    // vpie::VofaBridge::Get().SendOnce(last_armor_yaw_, last_phi_);

    Eigen::MatrixXd p = ekf_.GetPPostMat();
    auto p_d = p.diagonal();
    // std::cout << p.diagonal() << std::endl;
    // v_14.Send(p_d(0), p_d(1), p_d(2), p_d(3), p_d(4), p_d(5), p_d(6), p_d(7), p_d(8));
    double p_xyz = std::sqrt(p_d(1) + p_d(3) + p_d(5));
    // v_14.Send(p_xyz, p_d(7), p_d(8));

    int score = GetKalmanScore(p);
    // v_14.Send(score);
    // std::cout << score << std::endl;

    // Prevent radius from spreading
    if (target_state(8) < 0.12) {
        target_state(8) = 0.12;
        ekf_.SetState(target_state);
    } else if (target_state(8) > 0.4) {
        target_state(8) = 0.4;
        ekf_.SetState(target_state);
    }
    target_state(5) = 0;
    ekf_.SetState(target_state);

    // Tracking state machine
    if (tracker_state_ == DETECTING) {
        if (matched) {
            detect_count_++;
            if (detect_count_ > param_.tracking_count_threshold) {
                detect_count_ = 0;
                tracker_state_ = TRACKING;
            }
        } else {
            detect_count_ = 0;
            tracker_state_ = LOST;
        }
    } else if (tracker_state_ == TRACKING) {
        if (!matched) {
            tracker_state_ = TEMP_LOST;
            lost_count_++;
        }
    } else if (tracker_state_ == TEMP_LOST) {
        if (!matched) {
            lost_count_++;
            if (lost_count_ > lost_count_threshold_) {
                lost_count_ = 0;
                tracker_state_ = LOST;
            }
        } else {
            tracker_state_ = TRACKING;
            lost_count_ = 0;
        }
    }
}

int IRVFilter::GetKalmanScore(Eigen::MatrixXd p_post) {
    const double p_v_xyz_coeff = 3.0;
    const double p_v_yaw_coeff = 3.0;
    const double p_r_coeff = 3.0;
    const double ground_noise = 10.;

    auto p_diagonal = p_post.diagonal();
    double p_v_xyz = std::sqrt(p_diagonal(1) + p_diagonal(2) + p_diagonal(5));
    double p_v_yaw = std::sqrt(p_diagonal(7));
    double p_r = std::sqrt(p_diagonal(8));

    double score = 100 - (p_v_xyz * p_v_xyz_coeff + p_v_yaw * p_v_yaw_coeff + p_r * p_r_coeff - ground_noise);
    score = LimitNumber(score, 0., 100.);
    return static_cast<int>(score);
}

void IRVFilter::InitEKF(const std::shared_ptr<FP::ArmorInfo>& input_armor) {
    double xa = input_armor->geographic_coord_pose.x;
    double ya = input_armor->geographic_coord_pose.y;
    double za = input_armor->geographic_coord_pose.z;
    Eigen::Vector3d p_vec = {xa, ya, za};
    last_armor_yaw_ = input_armor->geographic_coord_yaw;
    auto dtp = XYZ2DTP(p_vec);
    last_phi_ = dtp.z();
    // Set initial position at 0.2m behind the target_classify
    target_state = Eigen::VectorXd::Zero(9);
    double r = 0.26;
    double xc = xa + r * cos(input_armor->geographic_coord_yaw);
    double yc = ya + r * sin(input_armor->geographic_coord_yaw);
    d_z_ = 0, another_r_ = r;
    target_state << xc, 0, yc, 0, za, 0, input_armor->geographic_coord_yaw, 0, r;

    ekf_.SetState(target_state);
}

void IRVFilter::UpdateArmorsNum(const RM::TargetArmorNum input_target_armor_num) {
    if (!(input_target_armor_num == RM::TargetArmorNum::TWO || input_target_armor_num == RM::TargetArmorNum::THREE ||
          input_target_armor_num == RM::TargetArmorNum::FOUR)) {
        std::cout << "MT:RVT: wrong input armors target_classify armor num" << std::endl;
        std::terminate();
    }
    target_armor_num_ = input_target_armor_num;
}

void IRVFilter::HandleArmorJump(const std::shared_ptr<FP::ArmorInfo>& input_armor) {
    //        UpdateArmorsNum(input_target_armor_num);
    // double yaw = SerializeArmorYawAngle(input_armor->geographic_coord_yaw);
    last_armor_yaw_ = input_armor->geographic_coord_yaw;
    target_state(6) = input_armor->geographic_coord_yaw;
    // Only 4 armors has 2 radius and height
    if (target_armor_num_ == RM::TargetArmorNum::FOUR) {
        d_z_ = target_state(4) - input_armor->geographic_coord_pose.z;
        target_state(4) = input_armor->geographic_coord_pose.z;
        std::swap(target_state(8), another_r_);
    }
    std::cout << "[armor_tracker]"
              << "Armor jump!" << std::endl;

    // todo add debug switcher

    // If position difference is larger than max_match_distance_,
    // take this case as the ekf diverged, reset the state

    // 其实�?能就�?陈军为了防�?�卡尔曼发散搞得限幅

    Eigen::Vector3d current_p(input_armor->geographic_coord_pose.x, input_armor->geographic_coord_pose.y,
                              input_armor->geographic_coord_pose.z);
    Eigen::Vector3d infer_p = GetArmorPositionFromState(target_state);
    if ((current_p - infer_p).norm() > param_.max_match_distance_diff) {
        // if(false){
        double r = target_state(8);
        target_state(0) = input_armor->geographic_coord_pose.x + r * cos(input_armor->geographic_coord_yaw);  // xc
        target_state(1) = 0;                                                                                  // vxc
        target_state(2) = input_armor->geographic_coord_pose.y + r * sin(input_armor->geographic_coord_yaw);  // yc
        target_state(3) = 0;                                                                                  // vyc
        target_state(4) = input_armor->geographic_coord_pose.z;                                               // za
        target_state(5) = 0;                                                                                  // vza
        std::cout << "[armor_tracker]"
                  << "Reset State!" << std::endl;
        // todo add debug switcher
    }
    // getchar();

    ekf_.SetState(target_state);
}

double IRVFilter::SerializeArmorYawAngle(double yaw) {
    yaw = last_armor_yaw_ + angles::shortest_angular_distance(last_armor_yaw_, yaw);
    last_armor_yaw_ = yaw;
    return yaw;
}

double IRVFilter::SerializePhiAngle(double phi) {
    phi = last_phi_ + angles::shortest_angular_distance(last_phi_, phi);
    last_phi_ = phi;
    return phi;
}

Eigen::Vector3d IRVFilter::GetArmorPositionFromState(const Eigen::VectorXd& x) {
    // Calculate predicted position of the current armor
    double xc = x(0), yc = x(2), za = x(4);
    double yaw = x(6), r = x(8);
    double xa = xc - r * cos(yaw);
    double ya = yc - r * sin(yaw);

    return {xa, ya, za};
}

}  // namespace MT::IRVT
