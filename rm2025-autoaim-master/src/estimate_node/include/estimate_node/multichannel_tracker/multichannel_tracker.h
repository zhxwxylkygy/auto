//
// Created by wpie on 23-11-2.
//

#pragma once

#include <array>
#include <iostream>
#include <vector>
#include "estimate_node/interface/i_tracker.h"
#include "estimate_node/interface/interface.h"
#include "power_rune_tracker/power_rune_tracker.h"

#include "irv_tracker/irv_tracker.h"
#include "static_tracker/static_tracker.h"
#include "static_tracker/triaxial_velocity_filter.h"
#include "estimate_node/interface/mcu.h"

namespace MT {
class MultichannelTracker {
   public:
    MultichannelTracker();

    void Run(std::map<RM::TargetClassify, std::shared_ptr<ITargetInfo>>& input_targets,
             std::vector<std::shared_ptr<FP::IFeature>> input_features,
             const MCU::Orders& input_order_data);

    static void InitTargetVec(std::map<RM::TargetClassify, std::shared_ptr<ITargetInfo>>& input_targets);

   private:
    static void Classified(std::map<RM::TargetClassify, std::shared_ptr<ITargetInfo>>& input_targets,
                           std::vector<std::shared_ptr<FP::IFeature>> input_features,
                           const MCU::Orders& input_order_data);

    void Track(std::map<RM::TargetClassify, std::shared_ptr<ITargetInfo>>& input_targets,
               const MCU::Orders& input_order_data);

    struct Param {
    } param_;

    std::map<RM::TargetClassify, std::unique_ptr<MT::ITracker>> trackers_{};
};
}  // namespace MT
// namespace MT
