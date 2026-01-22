//
// Created by wpie on 23-10-29.
//

#pragma once

#include <cmath>

#include <calculate/trajectory_calculate.h>
#include "angles/angles.h"

#include <estimate_node/interface/i_algorithm.h>
#include <estimate_node/interface/mcu.h>
#include "rv_algo/rv_algo.h"
#include "static_algo/static_algo.h"

namespace KS {

class KinematicsSolver {
   public:
    KinematicsSolver() = default;

    [[nodiscard]] TargetSolution Run(
        const std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_target_pair,
        const MCU::Orders& input_order_data,
        cv::Point3d p);

   private:
    struct Param {
        const int power_rune_mode_judge_remaining_time_threshold_s = 210;
    } param_;

    enum class PowerRuneMode { LARGE_POWER_RUNE, SMALL_POWER_RUNE, INVALID } power_rune_mode_ = PowerRuneMode::INVALID;

    void ModeUpdate(RM::TargetClassify input_key, const MCU::Orders& input_order_data);

    Trajectory trajectory_computer_{};

    std::pair<RM::TargetClassify, std::unique_ptr<IAlgorithm>> algorithm_pair_{
        RM::TargetClassify::BASE_DART, std::make_unique<StaticAlgo>(trajectory_computer_)};
};
}  // namespace KS
