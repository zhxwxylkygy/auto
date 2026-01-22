//
// Created by wpie on 23-11-2.
//

#pragma once
#include <array>
#include <iostream>
#include "estimate_node/interface/interface.h"
#include <estimate_node/interface/mcu.h>

namespace MT {
class ITracker {
   public:
    virtual void Run(std::shared_ptr<MT::ITargetInfo>& input_target,const MCU::Orders& input_order_data) = 0;
    virtual ~ITracker() = default;
};
}  // namespace MT
