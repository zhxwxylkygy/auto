
#include "estimate_node/multichannel_tracker/static_tracker/static_tracker.h"
#include "eigen3/Eigen/Eigen"

// STD
#include <memory>
#include <vector>

#include "angles/angles.h"

namespace MT::ST {
StaticTracker::StaticTracker() {
    tracker_ = std::make_unique<TriaxialVelocityFilter>();
}

void StaticTracker::Run(std::shared_ptr<MT::ITargetInfo>& input_target,
                        [[maybe_unused]] const MCU::Orders& input_order_data) {
    auto input_static_target = std::static_pointer_cast<StaticTargetInfo>(input_target);
    std::vector<FP::ArmorInfo> armors;
    std::for_each(input_static_target->armors_vec.begin(), input_static_target->armors_vec.end(),
                  [&armors, this](std::shared_ptr<FP::ArmorInfo>& input_ptr) {
                      FP::ArmorInfo output = *input_ptr;

                      armors.push_back(output);
                  });

    auto p = dynamic_cast<StaticTargetInfo*>(input_target.get());

    auto time = input_target->GetMyTimestamp();  // std::chrono::steady_clock::now();

    if (last_time_ == 0)
        last_time_ = time;
    auto dt = time - last_time_;
    last_time_ = time;

    // Update tracker
    if (tracker_->tracker_state_ == TriaxialVelocityFilter::LOST) {
        tracker_->Init(armors);
        input_target->target_available = false;
    } else {
        tracker_->Update(armors, dt);

        if (tracker_->tracker_state_ == TriaxialVelocityFilter::DETECTING) {
            input_target->target_available = false;
        } else if (tracker_->tracker_state_ == TriaxialVelocityFilter::TRACKING ||
                   tracker_->tracker_state_ == TriaxialVelocityFilter::TEMP_LOST) {
            p->target_available = true;
            // Fill target_classify message
            const auto& state = tracker_->target_state_;
            p->armor_id = tracker_->tracked_id_;
            p->c_position.x = state(0);
            p->c_velocity.x = state(1);
            p->c_position.y = state(2);
            p->c_velocity.y = state(3);
            p->c_position.z = state(4);
            p->c_velocity.z = state(5);

            //                std::cout << p->c_position << " " << p->c_velocity << std::endl;

            p->target_ref_coord = p->c_position;
        }
    }
}
}  // namespace MT::ST