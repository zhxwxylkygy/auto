//
// Created by wpie on 24-1-21.
//

#pragma once
#include <optional>
#include "angles/angles.h"
#include "calculate/trajectory_calculate.h"
#include "estimate_node/interface/i_algorithm.h"
#include "estimate_node/interface/interface.h"
#include "param_loader.hpp"

namespace KS {
class SSPRAlgo : public IAlgorithm {
   public:
    SSPRAlgo() = delete;

    explicit SSPRAlgo(Trajectory& input);

    [[nodiscard]] TargetSolution Run(const std::shared_ptr<MT::ITargetInfo>& input_target,
                                     cv::Point3d g_precise_tf_offset) override;

    [[nodiscard]] static cv::Point3d GetBullPose(double input_roll, double input_radius, cv::Point3d input_centre);

   private:
    struct Param {
        const RM::BulletType bullet_type =
            static_cast<RM::BulletType>(ParamLoader::GetInstance().GetParam<int>("KS", "BULLET_TYPE"));
        const double bullet_speed = ParamLoader::GetInstance().GetParam<double>("KS", "BULLET_SPEED");
        const double gimbal_yaw_machine_time =
            ParamLoader::GetInstance().GetParam<double>("KS", "GIMBAL_YAW_MACHINE_TIME_MS") / 1000.0;
        const double gimbal_pitch_machine_time =
            ParamLoader::GetInstance().GetParam<double>("KS", "GIMBAL_PITCH_MACHINE_TIME_MS") / 1000.0;
    } param_;
};
}  // namespace KS
