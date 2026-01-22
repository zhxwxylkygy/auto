//
// Created by wpie on 23-10-30.
//

#pragma once
#include <functional>
#include <estimate_node/interface/interface.h>
#include <calculate/trajectory_calculate.h>

namespace KS {

class IAlgorithm {
   public:
    IAlgorithm() = delete;
    explicit IAlgorithm(Trajectory& trajectory) : trajectory_computer_(std::ref(trajectory)) {}

    [[nodiscard]] virtual TargetSolution Run(const std::shared_ptr<MT::ITargetInfo>& input_target, cv::Point3d g_precise_tf_offset) = 0;

    virtual ~IAlgorithm() = default;

   protected:
    Trajectory& trajectory_computer_;
};
}  // namespace KS
