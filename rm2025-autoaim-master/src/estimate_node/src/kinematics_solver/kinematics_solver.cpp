//
// Created by wpie on 23-10-29.
//

#include "estimate_node/kinematics_solver/kinematics_solver.h"
#include <cmath>
#include <functional>
#include <opencv2/core/types.hpp>
#include "estimate_node/interface/rm_interface.h"
#include "estimate_node/kinematics_solver/slpr_algo/slpr_algo.h"
#include "estimate_node/kinematics_solver//sspr_algo/sspr_algo.h"

using namespace KS;

TargetSolution KinematicsSolver::Run(
    const std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_target_pair,
    const MCU::Orders& input_order_data, 
    cv::Point3d p) {
    ModeUpdate(input_target_pair.first, input_order_data);
    return algorithm_pair_.second->Run(input_target_pair.second, p);
}

void KinematicsSolver::ModeUpdate(RM::TargetClassify input_key, const MCU::Orders& input_order_data) {
    if (input_key != algorithm_pair_.first) {
        if (input_key == RM::TargetClassify::POWER_RUNE) {
            algorithm_pair_.first = input_key;
            if (input_order_data.remaining_time_s >= param_.power_rune_mode_judge_remaining_time_threshold_s) {
                power_rune_mode_ = PowerRuneMode::SMALL_POWER_RUNE;
                algorithm_pair_.second = std::make_unique<SSPRAlgo>(trajectory_computer_);

            } else {
                power_rune_mode_ = PowerRuneMode::LARGE_POWER_RUNE;
                algorithm_pair_.second = std::make_unique<SLPRAlgo>(trajectory_computer_);
            }
        } else if (input_key == RM::TargetClassify::BASE_DART || input_key == RM::TargetClassify::BASE_BOTTOM ||
                   input_key == RM::TargetClassify::BASE_TOP || input_key == RM::TargetClassify::OUTPOST_DART) {
            algorithm_pair_.second = std::make_unique<StaticAlgo>(trajectory_computer_);
            algorithm_pair_.first = input_key;
        } else {
            algorithm_pair_.second = std::make_unique<KS::RVA::RVAlgo>(trajectory_computer_);
            algorithm_pair_.first = input_key;
        }

        algorithm_pair_.first = input_key;
    }

    if (algorithm_pair_.first == RM::TargetClassify::POWER_RUNE) {
        if (input_order_data.remaining_time_s >= param_.power_rune_mode_judge_remaining_time_threshold_s) {
            if (power_rune_mode_ != PowerRuneMode::SMALL_POWER_RUNE) {
                power_rune_mode_ = PowerRuneMode::SMALL_POWER_RUNE;
                algorithm_pair_.second = std::make_unique<SSPRAlgo>(trajectory_computer_);
            }
        } else {
            if (power_rune_mode_ != PowerRuneMode::LARGE_POWER_RUNE) {
                power_rune_mode_ = PowerRuneMode::LARGE_POWER_RUNE;
                algorithm_pair_.second = std::make_unique<SLPRAlgo>(trajectory_computer_);
            }
        }
    }
}
