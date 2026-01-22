// Copyright 2022 Chen Jun
#include "estimate_node/multichannel_tracker/irv_tracker/irv_tracker.h"
#include <vofa_bridge/vofa_bridge.h>
#include "eigen3/Eigen/Eigen"

// STD
#include <memory>
#include <vector>

#include "angles/angles.h"

using namespace MT::IRVT;

IRVTracker::IRVTracker(bool is_hero) {
    irv_filter_ = std::make_unique<IRVFilter>(is_hero);
}

void IRVTracker::Run(std::shared_ptr<MT::ITargetInfo>& input_target,
                     [[maybe_unused]] const MCU::Orders& input_order_data) {
    auto input_rv_target = std::static_pointer_cast<RVTargetInfo>(input_target);
    //    std::for_each(input_rv_target->armors_vec.begin(), input_rv_target->armors_vec.end(),
    //                  [& armors, & input_rv_target](auto &input) {
    //                      RVT::RVTArmor armor;
    //                      armor.position.setX(input->geographic_coord_pose.x);
    //                      armor.position.setY(input->geographic_coord_pose.y);
    //                      armor.position.setZ(input->geographic_coord_pose.z);
    //                      armor.target_armor_num = input_rv_target->target_armor_num;
    //                      armor.armor_id = input->armor_id;
    //                      armor.yaw = input->geographic_coord_yaw;
    //                      armors.push_back(armor);
    //                  });

    auto time = input_target->GetMyTimestamp();  // std::chrono::steady_clock::now();
    if (last_time_ == 0)
        last_time_ = time;
    auto dt = time - last_time_;  // TODO : test here
    last_time_ = time;

    if (irv_filter_->tracker_state_ == IRVFilter::LOST) {
        irv_filter_->Init(input_rv_target->armors_vec, input_rv_target->target_armor_num);
        input_target->target_available = false;
        // std::cout << "lost" << std::endl;
    } else {
        irv_filter_->Update(input_rv_target->armors_vec, dt);

        if (irv_filter_->tracker_state_ == IRVFilter::DETECTING) {
            input_target->target_available = false;
            // std::cout << "detecting" << std::endl;
        } else if (irv_filter_->tracker_state_ == IRVFilter::TRACKING ||
                   irv_filter_->tracker_state_ == IRVFilter::TEMP_LOST) {
            // std::cout << "tracking" << std::endl;
            input_rv_target->target_available = true;

            const auto& state = irv_filter_->target_state;
            input_rv_target->armor_id = irv_filter_->tracked_id_;
            input_rv_target->c_position.x = state(0);
            input_rv_target->c_velocity.x = state(1);
            input_rv_target->c_position.y = state(2);
            input_rv_target->c_velocity.y = state(3);
            input_rv_target->c_position.z = state(4);
            input_rv_target->c_velocity.z = state(5);
            input_rv_target->yaw = state(6);
            input_rv_target->v_yaw = state(7);
            input_rv_target->r_1 = state(8);
            input_rv_target->r_2 = irv_filter_->another_r_;
            input_rv_target->d_z = irv_filter_->d_z_;
            //std::cout<< "distance" << sqrt(input_rv_target->c_position.x * input_rv_target->c_position.x + state(2) * state(2) + state(4) * state(4)) << std::endl;
            input_rv_target->target_ref_coord = input_rv_target->c_position;
            //vpie::VofaBridge::Get().SendOnce(state(0), state(2), state(4));
            //            vofaHelper.Send((float) angles::to_degrees(input_rv_target->yaw),
            //                            angles::to_degrees((float) input_rv_target->v_yaw), (float)
            //                            input_rv_target->c_position.x, (float) input_rv_target->c_position.y, (float)
            //                            input_rv_target->c_position.z, (float) input_rv_target->c_velocity.x, (float)
            //                            input_rv_target->c_velocity.y, (float) input_rv_target->c_velocity.z, (float)
            //                            input_rv_target->r_1);
            //                vofaHelper.Send((float) p->c_position.x, (float) p->c_position.y, (float)
            //                p->c_position.z); vofaHelper.Send((float) p->c_velocity.x, (float) p->c_velocity.y,
            //                (float) p->c_velocity.z); std::cout << *p << std::endl;
        }
    }
}
