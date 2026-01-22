//
// Created by wpie on 23-11-2.
//

#pragma once
#include <optional>
#include "angles/angles.h"
#include "calculate/trajectory_calculate.h"
#include "estimate_node/interface/i_algorithm.h"
#include "estimate_node/interface/interface.h"
#include "param_loader.hpp"

namespace KS {
class StaticAlgo : public IAlgorithm {
   public:
    StaticAlgo() = delete;

    explicit StaticAlgo(Trajectory& input);

    [[nodiscard]] TargetSolution Run(const std::shared_ptr<MT::ITargetInfo>& input_target,
                                     cv::Point3d g_precise_tf_offset) override;

   private:
    struct Param {
        const RM::BulletType bullet_type =
            static_cast<RM::BulletType>(ParamLoader::GetInstance().GetParam<int>("KS", "BULLET_TYPE"));
        const double bullet_speed = ParamLoader::GetInstance().GetParam<double>("KS", "BULLET_SPEED");
        const bool advanced_transform = ParamLoader::GetInstance().GetParam<bool>("ROBOT", "ADVANCED_TRANSFORM");
    } param_;
};
}  // namespace KS
