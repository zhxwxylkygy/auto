// Copyright 2022 Chen Jun

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "estimate_node/interface/i_tracker.h"
#include "irv_filter.h"
#include "param_loader.hpp"

namespace MT::IRVT {
class IRVTracker : public ITracker {
   public:
    IRVTracker() = delete;

    IRVTracker(bool is_hero);

    void Run(std::shared_ptr<MT::ITargetInfo>& input_target, const MCU::Orders& input_order_data) final;

   private:
    struct Param {
    } param_;

    std::unique_ptr<IRVFilter> irv_filter_;

    double last_time_;
};
}  // namespace MT::IRVT
