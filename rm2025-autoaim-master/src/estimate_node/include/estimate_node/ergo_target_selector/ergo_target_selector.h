//
// Created by wpie on 23-10-26.
//

#pragma once

#include <angles/angles.h>
#include <calculate/trajectory_calculate.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <array>
#include <optional>
#include <variant>
#include <vector>
#include "estimate_node/interface/mcu.h"
#include "opencv2/opencv.hpp"

namespace ETS {

class ErgoTargetSelector {
   public:
    ErgoTargetSelector() = default;

    std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> Run(
        const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_targets,
        const MCU::Orders& input_order_data,
        const double& imu_yaw);

   private:
    enum class GameMode { RMUC, RMUL };

    struct Param {
        const GameMode game_mode = GameMode::RMUC;
        const int auto_mode_low_hp_threshold = 50;
        const double auto_mode_low_hp_add_weighting = 3.;
        const double auto_mode_inverse_func_a = 3.;
        const double auto_mode_target_switch_threshold = 1.;
    } param_;
    std::vector<RM::TargetClassify> target_keys_;

    std::map<RM::TargetClassify, double> target_weighting_map_;

    std::optional<RM::TargetClassify> last_target_key_opt_ = std::nullopt;

    void ExcludeTargets(std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>> input_targets,
                        MCU::Orders input_order_data);

    std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> ManualModeSelect(
        std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>> input_targets,
        MCU::Orders input_order_data,
        const double& imu_yaw);

    std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> AutoModeSelect(
        const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_targets,
        MCU::Orders input_order_data);

    std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> HeroModeSelect(
        const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_targets);
};
}  // namespace ETS
