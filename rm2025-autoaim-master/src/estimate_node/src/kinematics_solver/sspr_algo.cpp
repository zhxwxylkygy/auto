//
// Created by wpie on 24-1-21.
//

#include "estimate_node/kinematics_solver/sspr_algo/sspr_algo.h"
#include "calculate/basic_calculate.h"
using namespace KS;

TargetSolution SSPRAlgo::Run(const std::shared_ptr<MT::ITargetInfo>& input_target, cv::Point3d g_precise_tf_offset) {
    auto power_rune_target = std::static_pointer_cast<MT::PowerRuneTargetInfo>(input_target);
    auto info = get<MT::PowerRuneTargetInfo::SSPR>(power_rune_target->motion_info);
    auto bull_pose = GetBullPose(info.bull_roll, 0.610, info.geographic_coord_r_symbol_pose);

    auto horizon_distance = GetDistance(bull_pose.x, bull_pose.y);
    auto tc_result_opt = trajectory_computer_.GetSolution(horizon_distance, bull_pose.z);
    if (!tc_result_opt.has_value()) {
        std::cerr << "ks: invalid trajectory computer result" << std::endl;
        std::terminate();
    }
    auto yaw_t_sum = tc_result_opt.value().fly_time + param_.gimbal_yaw_machine_time;
    auto pitch_t_sum = tc_result_opt.value().fly_time + param_.gimbal_pitch_machine_time;

    auto yaw_delta_roll = info.bull_abs_v_roll * yaw_t_sum;
    auto pitch_delta_roll = info.bull_abs_v_roll * pitch_t_sum;

    auto yaw_predicted_roll =
        angles::normalize_angle(info.is_clockwise ? info.bull_roll - yaw_delta_roll : info.bull_roll + yaw_delta_roll);
    auto pitch_predicted_roll = angles::normalize_angle(info.is_clockwise ? info.bull_roll - pitch_delta_roll
                                                                          : info.bull_roll + pitch_delta_roll);

    auto yaw_predicted_bull_pose = GetBullPose(yaw_predicted_roll, 0.60, info.geographic_coord_r_symbol_pose);
    auto pitch_predicted_bull_pose = GetBullPose(pitch_predicted_roll, 0.60, info.geographic_coord_r_symbol_pose);

    auto pitch_predicted_tc_result_opt =
        trajectory_computer_.GetSolution(GetXOYNorm(pitch_predicted_bull_pose), pitch_predicted_bull_pose.z);
    if (!pitch_predicted_tc_result_opt.has_value()) {
        std::cerr << "ks: invalid trajectory computer result" << std::endl;
        std::terminate();
    }
    TargetSolution output{GetXOYYaw(yaw_predicted_bull_pose), -pitch_predicted_tc_result_opt.value().angle, true};
    
    return output;
}
SSPRAlgo::SSPRAlgo(Trajectory& input) : IAlgorithm(input) {
    if (param_.bullet_type == RM::BulletType::D_17MM)
        trajectory_computer_.Init(Trajectory::Mode::BULLET_17MM_SECOND_ORDER_APPROX, param_.bullet_speed);
    else
        trajectory_computer_.Init(Trajectory::Mode::BULLET_42MM_SECOND_ORDER_APPROX, param_.bullet_speed);
}

cv::Point3d SSPRAlgo::GetBullPose(const double input_roll, const double input_radius, cv::Point3d input_centre) {
    double roll = input_roll, radius = input_radius;

    cv::Point3d geographic_coord_r_symbol_pose(input_centre);
    cv::Point2d power_rune_plane_bull_vec(cos(roll) * radius, sin(roll) * radius);
    cv::Point2d geographic_coord_r_symbol_xoy_vec(input_centre.x, input_centre.y);
    cv::Point3d power_rune_plane_basis_x_unit_vec_in_geographic_coord(  // 向右旋转90�? 并作为三维基底向量的x y
        cos(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.x - sin(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.y,
        sin(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.x + cos(-M_PI / 2) * geographic_coord_r_symbol_xoy_vec.y, 0);
    power_rune_plane_basis_x_unit_vec_in_geographic_coord /=
        norm(power_rune_plane_basis_x_unit_vec_in_geographic_coord);

    cv::Point3d power_rune_plane_basis_y_unit_vec_in_geographic_coord(0, 0, 1);

    auto geographic_coord_bull_pose =
        geographic_coord_r_symbol_pose +
        power_rune_plane_basis_x_unit_vec_in_geographic_coord * power_rune_plane_bull_vec.x +
        power_rune_plane_basis_y_unit_vec_in_geographic_coord * power_rune_plane_bull_vec.y;
    return geographic_coord_bull_pose;
}