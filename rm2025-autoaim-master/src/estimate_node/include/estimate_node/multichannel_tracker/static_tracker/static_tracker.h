#pragma once

#include <memory>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "estimate_node/interface/i_tracker.h"
#include "triaxial_velocity_filter.h"

namespace MT::ST {
class StaticTracker : public ITracker {
   public:
    StaticTracker();

    void Run(std::shared_ptr<MT::ITargetInfo>& input_target, const MCU::Orders& input_order_data) final;

   private:
    struct Param {
    } param_;

    double last_time_;

    std::unique_ptr<TriaxialVelocityFilter> tracker_;
};
}  // namespace MT::ST
