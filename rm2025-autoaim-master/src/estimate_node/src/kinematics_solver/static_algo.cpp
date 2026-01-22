//
// Created by wpie on 23-11-2.
//

#include "estimate_node/kinematics_solver/static_algo/static_algo.h"
#include <exception>
#include "calculate/basic_calculate.h"
using namespace KS;

inline cv::Point3d g_precise_tf_offset;

TargetSolution StaticAlgo::Run(const std::shared_ptr<MT::ITargetInfo>& input_target, cv::Point3d g_precise_tf_offset) {
    auto static_target = std::static_pointer_cast<MT::StaticTargetInfo>(input_target);
    if (param_.advanced_transform)
        static_target->c_position + g_precise_tf_offset;
    double horizon_distance = GetDistance(static_target->c_position.x, static_target->c_position.y);
    double fly_time = trajectory_computer_.GetSolution(horizon_distance, static_target->c_position.z)->fly_time;
    auto predicted_p = static_target->c_position + static_target->c_velocity * fly_time;
    double predicted_horizon_distance = GetDistance(predicted_p.x, predicted_p.y);

    TargetSolution output{};
    output.enable_shoot = true;
    output.yaw = GetYaw(static_target->c_position.x, static_target->c_position.y);
    output.pitch = -trajectory_computer_.GetSolution(predicted_horizon_distance, predicted_p.z)->angle;
    return output;
}
StaticAlgo::StaticAlgo(Trajectory& input) : IAlgorithm(input) {
    if (param_.bullet_type == RM::BulletType::D_17MM)
        trajectory_computer_.Init(Trajectory::Mode::BULLET_17MM_SECOND_ORDER_APPROX, param_.bullet_speed);
    else
        trajectory_computer_.Init(Trajectory::Mode::BULLET_42MM_SECOND_ORDER_APPROX, param_.bullet_speed);
}