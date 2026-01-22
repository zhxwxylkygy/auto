//
// Created by wpie on 23-11-5.
//

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "estimate_node/interface/i_tracker.h"
#include "param_loader.hpp"
#include "slpr_filter.h"
#include "sspr_filter.h"

namespace MT::PT {
class PowerRuneTracker : public ITracker {
   public:
    void Run(std::shared_ptr<MT::ITargetInfo>& input_targets, const MCU::Orders& input_order_data) final;

   private:
    struct Param {
        const int power_rune_mode_judge_remaining_time_threshold_s = 210;
    } param_;

    double last_time_;
    // todo 注意此变量的数学概念有效范围
    
    std::unique_ptr<PT::SSPRFilter> sspr_filter_ = std::make_unique<PT::SSPRFilter>();

    std::unique_ptr<PT::SLPRFilter> slpr_filter_ = std::make_unique<PT::SLPRFilter>();
};

}  // namespace MT::PT
