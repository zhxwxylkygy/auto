//
// Created by wpie on 24-1-21.
//

#include "estimate_node/kinematics_solver/slpr_algo/slpr_algo.h"
#include "calculate/basic_calculate.h"
using namespace KS;

TargetSolution SLPRAlgo::Run(const std::shared_ptr<MT::ITargetInfo>& input_target, cv::Point3d g_precise_tf_offset) {
    auto power_rune_target = std::static_pointer_cast<MT::PowerRuneTargetInfo>(input_target);
    auto info = get<MT::PowerRuneTargetInfo::SLPR>(power_rune_target->motion_info);
    auto bull_pose = GetBullPose(info.bull_roll, 0.60, info.geographic_coord_r_symbol_pose);

    auto horizon_distance = GetDistance(bull_pose.x, bull_pose.y);
    auto tc_result_opt = trajectory_computer_.GetSolution(horizon_distance, bull_pose.z);
    if (!tc_result_opt.has_value()) {
        std::cerr << "ks: invalid trajectory computer result" << std::endl;
        std::terminate();
    }

    auto yaw_t_predict = tc_result_opt.value().fly_time + param_.gimbal_yaw_machine_time + info.t_sum;
    auto pitch_t_predict = tc_result_opt.value().fly_time + param_.gimbal_pitch_machine_time + info.t_sum;
    auto t_now = info.t_sum;

    // auto yaw_delta_roll =
    //     (-info.func_a / info.func_omega * cos(info.func_omega * yaw_t_predict + info.func_phi) +
    //      (2.090 - info.func_a) * yaw_t_predict) -
    //     (-info.func_a / info.func_omega * cos(info.func_omega * t_now + info.func_phi) + (2.090 - info.func_a) *
    //     t_now);

    // auto pitch_delta_roll =
    //     (-info.func_a / info.func_omega * cos(info.func_omega * pitch_t_predict + info.func_phi) +
    //      (2.090 - info.func_a) * pitch_t_predict) -
    //     (-info.func_a / info.func_omega * cos(info.func_omega * t_now + info.func_phi) + (2.090 - info.func_a) *
    //     t_now);

    auto yaw_delta_roll = (-info.func_a / info.func_omega * cos(info.func_omega * (yaw_t_predict + info.func_phi)) +
                           (2.090 - info.func_a) * (yaw_t_predict + info.func_phi)) -
                          (-info.func_a / info.func_omega * cos(info.func_omega * (t_now + info.func_phi)) +
                           (2.090 - info.func_a) * (t_now + info.func_phi));

    auto pitch_delta_roll = (-info.func_a / info.func_omega * cos(info.func_omega * (pitch_t_predict + info.func_phi)) +
                           (2.090 - info.func_a) * (pitch_t_predict + info.func_phi)) -
                          (-info.func_a / info.func_omega * cos(info.func_omega * (t_now + info.func_phi)) +
                           (2.090 - info.func_a) * (t_now + info.func_phi));

    yaw_delta_roll = angles::normalize_angle(yaw_delta_roll);
    pitch_delta_roll = angles::normalize_angle(pitch_delta_roll);
    if (info.is_clockwise) {
        yaw_delta_roll *= -1;
        pitch_delta_roll *= -1;
    }

    auto yaw_predicted_roll = yaw_delta_roll + info.bull_roll;
    auto pitch_predicted_roll = pitch_delta_roll + info.bull_roll;

    auto yaw_predicted_bull_pose = GetBullPose(yaw_predicted_roll, 0.60, info.geographic_coord_r_symbol_pose);
    auto pitch_predicted_bull_pose = GetBullPose(pitch_predicted_roll, 0.60, info.geographic_coord_r_symbol_pose);

    auto pitch_predicted_tc_result_opt =
        trajectory_computer_.GetSolution(GetXOYNorm(pitch_predicted_bull_pose), yaw_predicted_bull_pose.z);
    if (!pitch_predicted_tc_result_opt.has_value()) {
        std::cerr << "ks: invalid trajectory computer result" << std::endl;
        std::terminate();
    }
    TargetSolution output{GetXOYYaw(yaw_predicted_bull_pose), -pitch_predicted_tc_result_opt.value().angle, true};

    return output;
}
SLPRAlgo::SLPRAlgo(Trajectory& input) : IAlgorithm(input) {
    if (param_.bullet_type == RM::BulletType::D_17MM)
        trajectory_computer_.Init(Trajectory::Mode::BULLET_17MM_SECOND_ORDER_APPROX, param_.bullet_speed);
    else
        trajectory_computer_.Init(Trajectory::Mode::BULLET_42MM_SECOND_ORDER_APPROX, param_.bullet_speed);
}

cv::Point3d SLPRAlgo::GetBullPose(const double input_roll, const double input_radius, cv::Point3d input_centre) {
    double roll = input_roll, radius = input_radius;

    cv::Point3d geographic_coord_symbol_pose(input_centre);
    cv::Point2d power_rune_plane_bull_vec(cos(roll) * radius, sin(roll) * radius);
    cv::Point2d geographic_coord_r_symbol_xoy_vec(input_centre.x, input_centre.y);
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