//
// Created by wpie on 24-1-21.
//

#include "estimate_node/multichannel_tracker/power_rune_tracker/slpr_filter.h"
#include <cassert>
#include <memory>


namespace MT::PT {
SLPRFilter::SLPRFilter() {
    auto f = [this](const Eigen::VectorXd& x) {
        Eigen::VectorXd x_new = x;
        auto d_t = ekf_formula_server_.d_t;
        auto t_sum = ekf_formula_server_.t_sum;
        double last_roll = x(0);
        double last_v_roll = x(1);
        double last_func_a = x(2);
        double last_func_omega = x(3);
        double last_func_phi = x(4);
        double last_centre_x = x(5);
        double last_centre_y = x(6);
        double last_centre_z = x(7);
        x_new(0) = is_clockwise_ ? last_roll - d_t * last_v_roll : last_roll + d_t * last_v_roll;
        x_new(1) = last_func_a * std::sin(last_func_omega * (t_sum + last_func_phi)) + (2.090 - last_func_a);
        x_new(2) = last_func_a;
        x_new(3) = last_func_omega;
        x_new(4) = last_func_phi;
        x_new(5) = last_centre_x;
        x_new(6) = last_centre_y;
        x_new(7) = last_centre_z;
        return x_new;
    };

    auto jacobian_f = [this]([[maybe_unused]] const Eigen::VectorXd& x) {
        Eigen::MatrixXd output(8, 8);
        auto d_t = ekf_formula_server_.d_t;
        double ceres_x[8] = {x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7)};
        double* x_blocks[1] = {ceres_x};
        double residuals[8];
        double jacobian[64] = {};
        double* jacobian_blocks[1] = {jacobian};
        f_cost_function->Evaluate(x_blocks, residuals, jacobian_blocks);
        // clang-format off
        //@formatter:off
        output << 
            jacobian[0], jacobian[1], jacobian[2], jacobian[3], jacobian[4], jacobian[5], jacobian[6], jacobian[7],
            jacobian[8], jacobian[9], jacobian[10], jacobian[11], jacobian[12], jacobian[13], jacobian[14], jacobian[15],
            jacobian[16], jacobian[17], jacobian[18], jacobian[19], jacobian[20], jacobian[21], jacobian[22], jacobian[23],
            jacobian[24], jacobian[25], jacobian[26], jacobian[27], jacobian[28], jacobian[29], jacobian[30], jacobian[31],
            jacobian[32], jacobian[33], jacobian[34], jacobian[35], jacobian[36], jacobian[37], jacobian[38], jacobian[39],
            jacobian[40], jacobian[41], jacobian[42], jacobian[43], jacobian[44], jacobian[45], jacobian[46], jacobian[47],
            jacobian[48], jacobian[49], jacobian[50], jacobian[51], jacobian[52], jacobian[53], jacobian[54], jacobian[55],
            jacobian[56], jacobian[57], jacobian[58], jacobian[59], jacobian[60], jacobian[61], jacobian[62], jacobian[63];
        //@formatter:on
        // clang-format on

        return output;
    };

    auto h = [](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(4);
        double roll = x(0);
        double centre_x = x(5);
        double centre_y = x(6);
        double centre_z = x(7);

        cv::Point3d centre(centre_x, centre_y, centre_z);
        z(0) = roll;
        z(1) = norm(centre);
        z(2) = std::acos(centre_z / norm(centre));
        z(3) = atan2(centre_y, centre_x);

        return z;
    };

    auto jacobian_h = [this](const Eigen::VectorXd& x) {
        Eigen::MatrixXd output(4, 8);
        double ceres_x[8] = {x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7)};
        double* x_blocks[1] = {ceres_x};
        double residuals[4];
        double jacobian[32] = {};
        double* jacobian_blocks[1] = {jacobian};
        h_cost_function->Evaluate(x_blocks, residuals, jacobian_blocks);

        // clang-format off
        //@formatter:off
        output << 
            jacobian[0], jacobian[1], jacobian[2], jacobian[3], 
            jacobian[4], jacobian[5], jacobian[6], jacobian[7], 
            jacobian[8], jacobian[9], jacobian[10], jacobian[11], 
            jacobian[12], jacobian[13], jacobian[14],jacobian[15], 
            jacobian[16], jacobian[17], jacobian[18], jacobian[19],
            jacobian[20], jacobian[21], jacobian[22], jacobian[23],
            jacobian[24], jacobian[25], jacobian[26], jacobian[27],
            jacobian[28], jacobian[29], jacobian[30], jacobian[31];
        //@formatter:on
        // clang-format on

        return output;
    };

    ekf_formula_server_.q_roll_2 = pow(param_.q_roll, 2);
    ekf_formula_server_.q_v_roll_2 = pow(param_.q_v_roll, 2);
    ekf_formula_server_.q_func_a_2 = pow(param_.q_func_a, 2);
    ekf_formula_server_.q_func_omega_2 = pow(param_.q_func_omega, 2);
    ekf_formula_server_.q_func_phi_2 = pow(param_.q_func_phi, 2);
    ekf_formula_server_.q_centre_xyz_2 = pow(param_.q_centre_xyz, 2);
    ekf_formula_server_.q_roll_time_v_roll = param_.q_roll * param_.q_v_roll;
    auto update_q = [this]() {
        Eigen::MatrixXd q(8, 8);
        auto d_t = ekf_formula_server_.d_t;
        // clang-format off
        //@formatter:off
        q << 
            pow(d_t, 4) / 4 * ekf_formula_server_.q_roll_2, pow(d_t, 3) / 2 * ekf_formula_server_.q_roll_time_v_roll, 0, 0, 0, 0, 0, 0,
            pow(d_t, 3) / 2 * ekf_formula_server_.q_roll_time_v_roll, d_t * ekf_formula_server_.q_v_roll_2, 0, 0, 0, 0, 0, 0,
            0, 0, ekf_formula_server_.q_func_a_2, 0, 0, 0, 0, 0,
            0, 0, 0, ekf_formula_server_.q_func_omega_2, 0, 0, 0, 0,
            0, 0, 0, 0, ekf_formula_server_.q_func_phi_2, 0, 0, 0,
            0, 0, 0, 0, 0, ekf_formula_server_.q_centre_xyz_2, 0, 0, 
            0, 0, 0, 0, 0, 0, ekf_formula_server_.q_centre_xyz_2, 0,
            0, 0, 0, 0, 0, 0, 0, ekf_formula_server_.q_centre_xyz_2;

        //@formatter:on
        //clang-format on

        return q;
    };

    ekf_formula_server_.r_roll_2 = pow(param_.r_roll, 2);
    ekf_formula_server_.r_centre_distance_factor_2 = pow(param_.r_centre_distance, 2);
    ekf_formula_server_.r_centre_theta_2 = pow(param_.r_centre_theta, 2);
    ekf_formula_server_.r_centre_phi_2 = pow(param_.r_centre_phi, 2);

    auto update_r = [this](const Eigen::VectorXd& z) {
        Eigen::DiagonalMatrix<double, 4> r;
        r.diagonal() << ekf_formula_server_.r_roll_2,
            ekf_formula_server_.r_centre_distance_factor_2 * z[1] * z[1], ekf_formula_server_.r_centre_theta_2,
            ekf_formula_server_.r_centre_phi_2;
        return r;
    };
    // P - error estimate covariance matrix
    Eigen::DiagonalMatrix<double, 8> p_0;
    p_0.setIdentity();
    ekf_.LoadFormula(f, h, jacobian_f, jacobian_h, update_q, update_r, p_0);
}

void SLPRFilter::Init(const std::shared_ptr<FP::FanInfo>& input_fan) {
    last_dir_judge_roll_ = 0;
    last_measure_power_rune_roll_ = 0;
    last_measure_r_symbol_phi_ = 0;

    ekf_formula_server_.t_sum = 0;

    tracked_fan_ = input_fan;

    InitEKF(tracked_fan_);

    auto bull_pose = tracked_fan_->geographic_coord_light_surface_points_infer_bull_pose;
    auto r_symbol_pose = tracked_fan_->geographic_coord_r_symbol_pose;
    std::tie(last_dir_judge_roll_, std::ignore) = GetRollAndRadiusFromPose(bull_pose, r_symbol_pose);

    filter_state_ = FilterState::DETECTING;
}

std::tuple<Eigen::VectorXd, bool> SLPRFilter::Update(const std::vector<std::shared_ptr<FP::FanInfo>>& input_fans,
                                                     double d_t) {
    //todo �?改接�? bool clockwise
    lost_count_threshold_ = static_cast<int>(param_.lost_duration_threshold / d_t);
    ekf_formula_server_.d_t = d_t;
    ekf_formula_server_.t_sum += d_t;

    Eigen::VectorXd target_pri_state = ekf_.Predict();
    // std::cout << target_pri_state << std::endl;
    // v_13.Send((float)target_pri_state(0), (float)target_pri_state(1), (float)target_pri_state(2), (float)target_pri_state(3), (float)target_pri_state(4),
    //                          (float)target_pri_state(5), (float)target_pri_state(6), (float)target_pri_state(7));


    bool matched = false;
    Eigen::VectorXd output_state(9);

    auto target_fan_iter =
        std::find_if(input_fans.begin(), input_fans.end(), [](auto& input_fan) { return input_fan->is_target_fan; });

    if (target_fan_iter != input_fans.end()) {
    // if(false){
        auto predicted_roll = target_pri_state(0);
        double now_roll;

        //            std::cout << "before" << (*target_fan_iter)->geographic_coord_r_symbol_pose << std::endl;
        std::tie(now_roll, std::ignore) =
            GetRollAndRadiusFromPose((*target_fan_iter)->geographic_coord_light_surface_points_infer_bull_pose,
                                     (*target_fan_iter)->geographic_coord_r_symbol_pose);

        auto roll_dev_x_hat_to_now = angles::shortest_angular_distance(predicted_roll, now_roll);
        if (std::abs(roll_dev_x_hat_to_now) <= param_.max_match_roll_dev) {
        // if(true) {
            tracked_fan_ = *target_fan_iter;
            matched = true;

            Eigen::VectorXd measurement(4);

            auto serialized_energy_roll = SerializeAngle(now_roll, last_measure_power_rune_roll_);
            auto r_symbol_eigen_pose = Eigen::Vector3d(tracked_fan_->geographic_coord_r_symbol_pose.x,
                                                       tracked_fan_->geographic_coord_r_symbol_pose.y,
                                                       tracked_fan_->geographic_coord_r_symbol_pose.z);
            auto r_symbol_eigen_dtp = XYZ2DTP(r_symbol_eigen_pose);
            r_symbol_eigen_dtp.z() = SerializeAngle(r_symbol_eigen_dtp.z(), last_measure_r_symbol_phi_);
            measurement << serialized_energy_roll, r_symbol_eigen_dtp;

            Eigen::VectorXd target_post_state = ekf_.Update(measurement);
            
            EKFStateLimit(target_post_state);
            output_state << target_post_state, ekf_formula_server_.t_sum;

        } else {
            HandleFanJump(*target_fan_iter, target_pri_state);
            EKFStateLimit(target_pri_state);
            output_state << target_pri_state, ekf_formula_server_.t_sum;
        }
    } else {
        EKFStateLimit(target_pri_state);
        output_state << target_pri_state, ekf_formula_server_.t_sum;
    }

    // Tracking state machine
    if (filter_state_ == FilterState::DETECTING) {
        if (matched) {
            detect_count_++;
            if (detect_count_ > param_.tracking_count_threshold) {
                detect_count_ = 0;
                filter_state_ = FilterState::TRACKING;
            }
        } else {
            detect_count_ = 0;
            filter_state_ = FilterState::LOST;
        }
    } else if (filter_state_ == FilterState::TRACKING) {
        if (!matched) {
            filter_state_ = FilterState::TEMP_LOST;
            lost_count_++;
        }
    } else if (filter_state_ == FilterState::TEMP_LOST) {
        if (!matched) {
            lost_count_++;
            if (lost_count_ > lost_count_threshold_) {
                lost_count_ = 0;
                filter_state_ = FilterState::LOST;
            }
        } else {
            filter_state_ = FilterState::TRACKING;
            lost_count_ = 0;
        }
    }

    return std::make_tuple(output_state, is_clockwise_);
}

void SLPRFilter::InitEKF(const std::shared_ptr<FP::FanInfo>& input_armor) {
    auto bull_pose = input_armor->geographic_coord_light_surface_points_infer_bull_pose;
    auto r_symbol_pose = input_armor->geographic_coord_r_symbol_pose;

    Eigen::VectorXd target_state = Eigen::VectorXd::Zero(8);

    double radius, roll;
    std::tie(roll, radius) = GetRollAndRadiusFromPose(bull_pose, r_symbol_pose);

    double func_a_mid = (0.780 + 1.045) / 2;
    double func_omega_mid = (1.884 + 2.0) / 2;
    // double func_phi_mid = (0 + M_PI) / 2;
    double func_phi_mid = 0.;
    double v_roll_mid = (1.310 + 3.135) / 2; //1.130 3.135分别为�?�则范围内能量机关转速的最小值和最大�?

    target_state << roll, v_roll_mid, func_a_mid, func_omega_mid, func_phi_mid, r_symbol_pose.x, r_symbol_pose.y, r_symbol_pose.z;

    ekf_.SetState(target_state);
}

void SLPRFilter::HandleFanJump(const std::shared_ptr<FP::FanInfo>& input_fan, Eigen::VectorXd& target_pri_state) {
    double roll;
    std::tie(roll, std::ignore) = GetRollAndRadiusFromPose(
        input_fan->geographic_coord_light_surface_points_infer_bull_pose, input_fan->geographic_coord_r_symbol_pose);

    target_pri_state(0) = roll;
    last_measure_power_rune_roll_ = 0;
    std::cout << "power rune filter: fan jump" << std::endl;

    ekf_.SetState(target_pri_state);

    // If position difference is larger than max_match_distance_,
    // take this case as the ekf diverged, reset the state

    // 其实�?能就�?陈军为了防�?�卡尔曼发散搞得限幅
    //        auto p = a.position;
    //        Eigen::Vector3d current_p(p.x(), p.y(), p.z());
    //        Eigen::Vector3d infer_p = GetArmorPositionFromState(target_state);
    //        if ((current_p - infer_p).norm() > param_.max_match_distance_diff) {
    //            double r = target_state(8);
    //            target_state(0) = p.x() + r * cos(yaw);  // xc
    //            target_state(1) = 0;                   // vxc
    //            target_state(2) = p.y() + r * sin(yaw);  // yc
    //            target_state(3) = 0;                   // vyc
    //            target_state(4) = p.z();                 // za
    //            target_state(5) = 0;                   // vza
    //            std::cout << "[armor_tracker]" << "Reset State!" << std::endl;
    //        }
}

void SLPRFilter::EKFStateLimit(Eigen::VectorXd &target_non_finite_state) {
        auto &state = target_non_finite_state;
        // state(1) = LimitNumber(state(1), 1.130, 3.135);
        state(2) = LimitNumber(state(2), 0.780, 1.045);
        state(3) = LimitNumber(state(3), 1.884, 2.);
        state(4) = LimitNumber(state(4), -M_PI, M_PI);

        ekf_.SetState(target_non_finite_state);
    }
void SLPRFilter::DirJudge(const std::vector<std::shared_ptr<FP::FanInfo>>& input_fans, const double input_d_t) {
    auto target_fan_iter =
        std::find_if(input_fans.begin(), input_fans.end(), [](auto& input_fan) { return input_fan->is_target_fan; });
    if (target_fan_iter == input_fans.end()) {
        dir_judge_deque_.push_back(DirModeUnit::UNCERTAIN);
        if (static_cast<int>(dir_judge_deque_.size()) > param_.dir_judge_deque_size)
            dir_judge_deque_.pop_front();
        return;
    }
    const auto& fan = *target_fan_iter;
    auto bull_pose = fan->geographic_coord_light_surface_points_infer_bull_pose;
    auto r_symbol_pose = fan->geographic_coord_r_symbol_pose;
    double now_roll;
    std::tie(now_roll, std::ignore) = GetRollAndRadiusFromPose(bull_pose, r_symbol_pose);

    double angle_dev = angles::shortest_angular_distance(last_dir_judge_roll_, now_roll);
    last_dir_judge_roll_ = now_roll;
    if (std::abs(angle_dev) / input_d_t <= param_.dir_judge_min_v_roll_degree) {
        dir_judge_deque_.push_back(DirModeUnit::UNCERTAIN);
        if (static_cast<int>(dir_judge_deque_.size()) > param_.dir_judge_deque_size)
            dir_judge_deque_.pop_front();
    } else {
        dir_judge_deque_.push_back(angle_dev >= 0 ? DirModeUnit::ANTICLOCKWISE : DirModeUnit::CLOCKWISE);
        if (static_cast<int>(dir_judge_deque_.size()) > param_.dir_judge_deque_size)
            dir_judge_deque_.pop_front();
    }

    int anticlockwise_count =
        static_cast<int>(std::count(dir_judge_deque_.begin(), dir_judge_deque_.end(), DirModeUnit::ANTICLOCKWISE));
    int clockwise_count =
        static_cast<int>(std::count(dir_judge_deque_.begin(), dir_judge_deque_.end(), DirModeUnit::CLOCKWISE));
    if (anticlockwise_count > param_.dir_judge_valid_value_count_threshold) {
        dir_judge_deque_.clear();
        is_clockwise_ = false;
        Init(*target_fan_iter);
    }
    if (clockwise_count > param_.dir_judge_valid_value_count_threshold) {
        dir_judge_deque_.clear();
        is_clockwise_ = true;
        Init(*target_fan_iter);
    }
}


bool SLPRFilter::EKFHCostFunction::operator()(const double* const x, double* residual) const {
    double roll = x[0];
    double centre_x = x[5];
    double centre_y = x[6];
    double centre_z = x[7];

    cv::Point3d centre(centre_x, centre_y, centre_z);
    residual[0] = roll;
    residual[1] = norm(centre);
    residual[2] = std::acos(centre_z / norm(centre));
    residual[3] = atan2(centre_y, centre_x);
    return true;
}
bool SLPRFilter::EKFFCostFunction::operator()(const double* const x, double* residual) const {
        auto d_t = class_ptr_->ekf_formula_server_.d_t;
        auto t_sum = class_ptr_->ekf_formula_server_.t_sum;
        double last_roll = x[0];
        double last_v_roll = x[1];
        double last_func_a = x[2];
        double last_func_omega = x[3];
        double last_func_phi = x[4];
        double last_centre_x = x[5];
        double last_centre_y = x[6];
        double last_centre_z = x[7];
        residual[0] = class_ptr_->is_clockwise_ ? last_roll - d_t * last_v_roll : last_roll + d_t * last_v_roll;
        residual[1] = last_func_a * std::sin(last_func_omega * t_sum + last_func_phi) + (2.090 - last_func_a);
        residual[2] = last_func_a;
        residual[3] = last_func_omega;
        residual[4] = last_func_phi;
        residual[5] = last_centre_x;
        residual[6] = last_centre_y;
        residual[7] = last_centre_z;
    return true;
}


cv::Point3d SLPRFilter::GetBullFromRollAndRadius(const double& input_roll,
                                                 const double& input_radius,
                                                 const cv::Point3d& r_symbol_pose) {
    double roll = input_roll, radius = input_radius;

    cv::Point3d geographic_coord_symbol_pose(r_symbol_pose);
    cv::Point2d power_rune_plane_bull_vec(cos(roll) * radius, sin(roll) * radius);
    cv::Point2d geographic_coord_r_symbol_xoy_vec(r_symbol_pose.x, r_symbol_pose.y);
    cv::Point3d power_rune_plane_basis_x_unit_vec_in_geographic_coord(  // 向右旋转90�? 并作为三维基底向量的x y
        cos(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.x - sin(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.y,
        sin(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.x + cos(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.y, 0);
    power_rune_plane_basis_x_unit_vec_in_geographic_coord /=
        norm(power_rune_plane_basis_x_unit_vec_in_geographic_coord);

    cv::Point3d power_rune_plane_basis_y_unit_vec_in_geographic_coord(0, 0, 1);

    auto geographic_coord_bull_pose =
        geographic_coord_symbol_pose +
        power_rune_plane_basis_x_unit_vec_in_geographic_coord * power_rune_plane_bull_vec.x +
        power_rune_plane_basis_y_unit_vec_in_geographic_coord * power_rune_plane_bull_vec.y;
    return geographic_coord_bull_pose;
}

std::tuple<double, double> SLPRFilter::GetRollAndRadiusFromPose(const cv::Point3d& bull_pose,
                                                                const cv::Point3d& r_symbol_pose) {
    cv::Point3d geographic_coord_r_symbol_to_bull_vec = bull_pose - r_symbol_pose;
    cv::Point2d geographic_coord_r_symbol_xoy_vec(r_symbol_pose.x, r_symbol_pose.y);
    cv::Point3d power_rune_plane_basis_x_unit_vec_in_geographic_coord(  // 向右旋转90�? 并作为三维基底向量的x y
        cos(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.x - sin(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.y,
        sin(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.x + cos(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.y, 0);
    power_rune_plane_basis_x_unit_vec_in_geographic_coord /=
        norm(power_rune_plane_basis_x_unit_vec_in_geographic_coord);
    cv::Point3d power_rune_plane_basis_y_unit_vec_in_geographic_coord(0, 0, 1);
    cv::Point2d power_rune_plane_bull_vec(
        geographic_coord_r_symbol_to_bull_vec.dot(power_rune_plane_basis_x_unit_vec_in_geographic_coord),
        geographic_coord_r_symbol_to_bull_vec.dot(power_rune_plane_basis_y_unit_vec_in_geographic_coord));
    double roll = atan2(power_rune_plane_bull_vec.y, power_rune_plane_bull_vec.x);
    double radius = norm(power_rune_plane_bull_vec);
    return std::make_tuple(roll, radius);
}

SLPRFilter::FilterState SLPRFilter::GetFilterState() {
    return filter_state_;
}

void SLPRFilter::SetFilterState(const FilterState input_state) {
    filter_state_ = input_state;
}

double SLPRFilter::SerializeAngle(const double input_now_angle, double& last_angle) {
    last_angle += angles::shortest_angular_distance(last_angle, input_now_angle);
    auto output_angle = last_angle;
    return output_angle;
}

}  // namespace MT::PT
