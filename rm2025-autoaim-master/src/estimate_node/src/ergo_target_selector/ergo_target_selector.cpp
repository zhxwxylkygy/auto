//
// Created by wpie on 23-10-26.
//

#include "estimate_node/ergo_target_selector/ergo_target_selector.h"
#include <utility>
#include <vector>
#include "estimate_node/interface/rm_interface.h"
#include "calculate/basic_calculate.h"
#include "estimate_node/interface/mcu.h"

using namespace ETS;

std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> ErgoTargetSelector::Run(
    const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_targets,
    const MCU::Orders& input_order_data, 
    const double& imu_yaw) {
    target_keys_.clear();
    for (auto& i : input_targets) {
        target_keys_.push_back(i.first);
    }
    
    ExcludeTargets(input_targets, input_order_data);
    if (input_order_data.target_priority_rule == MCU::AutoaimReceiveInfo::TargetPriorityRule::ONLY_HERO) {
        return HeroModeSelect(input_targets);
    } else if (input_order_data.target_priority_rule == MCU::AutoaimReceiveInfo::TargetPriorityRule::MANUAL) {
        return ManualModeSelect(input_targets, input_order_data, imu_yaw);
    } else {  // input_mcu_data.target_priority_rule == MCU::ReceiveInfo::TargetPriorityRule::AUTO
        return AutoModeSelect(input_targets, input_order_data);
    }
}

void ErgoTargetSelector::ExcludeTargets(
    const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>> input_targets,
    MCU::Orders input_order_data) {
    auto i = std::remove_if(
        target_keys_.begin(), target_keys_.end(), [&input_order_data, &input_targets](auto& target_key) -> bool {
            if (!input_targets.find(target_key)->second->target_available)
                return true;
            if (input_order_data.target_type == MCU::AutoaimReceiveInfo::TargetType::BUILDING) {
                return !(target_key == RM::TargetClassify::BASE_TOP || target_key == RM::TargetClassify::BASE_BOTTOM ||
                         target_key == RM::TargetClassify::BASE_DART ||
                         target_key == RM::TargetClassify::OUTPOST_SPIN ||
                         target_key == RM::TargetClassify::OUTPOST_DART);
            } else if (input_order_data.target_type == MCU::AutoaimReceiveInfo::TargetType::ROBOT) {
                return !(target_key == RM::TargetClassify::SENTRY || target_key == RM::TargetClassify::HERO_1 ||
                         target_key == RM::TargetClassify::ENGINEER_2 || target_key == RM::TargetClassify::STANDARD_3 ||
                         target_key == RM::TargetClassify::STANDARD_4 || target_key == RM::TargetClassify::STANDARD_5);
            } else if (input_order_data.target_type == MCU::AutoaimReceiveInfo::TargetType::POWER_RUNE) {
                return target_key != RM::TargetClassify::POWER_RUNE;
            } else {  // (input_data.target_type == MCU::ReceiveInfo::TargetType::POWER_RUNE)
                return target_key == RM::TargetClassify::POWER_RUNE;
            }
        });
    target_keys_.erase(i, target_keys_.end());
}

std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> ErgoTargetSelector::ManualModeSelect(
    const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>> input_targets,
    MCU::Orders input_order_data, 
    const double& imu_yaw) {
    std::function<bool(RM::TargetClassify&, RM::TargetClassify&)> min_yaw_orientation_f;
    min_yaw_orientation_f = [&input_order_data, &input_targets, imu_yaw](RM::TargetClassify& a, RM::TargetClassify& b) {
        double a_dev = fabs(angles::shortest_angular_distance(
            imu_yaw, atan2(input_targets.find(a)->second->target_ref_coord.y,
                                             input_targets.find(a)->second->target_ref_coord.x)));
        double b_dev = fabs(angles::shortest_angular_distance(
            imu_yaw, atan2(input_targets.find(b)->second->target_ref_coord.y,
                                             input_targets.find(b)->second->target_ref_coord.x)));
        return a_dev < b_dev;
        return true; 
    };
    if (target_keys_.empty()) {
        last_target_key_opt_ = std::nullopt;
        return std::nullopt;
    }
    if (!last_target_key_opt_.has_value()) {
        auto output_key_ptr = std::min_element(target_keys_.begin(), target_keys_.end(), min_yaw_orientation_f);
        last_target_key_opt_ = *output_key_ptr;
        return *input_targets.find(*output_key_ptr);
    } else {
        auto last_target_key_iter = std::find(target_keys_.begin(), target_keys_.end(), last_target_key_opt_.value());
        if (last_target_key_iter == target_keys_.end()) {
            auto output_ptr = std::min_element(target_keys_.begin(), target_keys_.end(), min_yaw_orientation_f);
            last_target_key_opt_ = *output_ptr;
            return *input_targets.find(*output_ptr);
        }

        if (input_order_data.switch_mode == MCU::AutoaimReceiveInfo::SwitchMode::NONE) {
            return *input_targets.find(*last_target_key_iter);
        } else if (input_order_data.switch_mode == MCU::AutoaimReceiveInfo::SwitchMode::RIGHT) {
            double last_target_yaw = atan2(input_targets.find((*last_target_key_iter))->second->target_ref_coord.y,
                                           input_targets.find((*last_target_key_iter))->second->target_ref_coord.x);
            auto aft_remove_end_target_keys_iter = std::remove_if(
                target_keys_.begin(), target_keys_.end(),
                [&last_target_yaw, &input_targets](RM::TargetClassify& input) {
                    double input_target_yaw = atan2(input_targets.find(input)->second->target_ref_coord.y,
                                                    input_targets.find(input)->second->target_ref_coord.x);
                    return angles::shortest_angular_distance(last_target_yaw, input_target_yaw) >=
                               -angles::from_degrees(1.5) ||
                           angles::shortest_angular_distance(last_target_yaw, input_target_yaw) <= -M_PI / 2;
                });
            if (aft_remove_end_target_keys_iter == target_keys_.begin())
                return *input_targets.find(*last_target_key_iter);
            auto output_ptr =
                std::min_element(target_keys_.begin(), aft_remove_end_target_keys_iter, min_yaw_orientation_f);
            last_target_key_opt_ = *output_ptr;
            return *input_targets.find(*output_ptr);
        } else if (input_order_data.switch_mode == MCU::AutoaimReceiveInfo::SwitchMode::LEFT) {
            double last_target_yaw = atan2(input_targets.find(*last_target_key_iter)->second->target_ref_coord.y,
                                           input_targets.find(*last_target_key_iter)->second->target_ref_coord.x);
            auto aft_remove_end_target_keys_iter = std::remove_if(
                target_keys_.begin(), target_keys_.end(),
                [&last_target_yaw, &input_targets](RM::TargetClassify& input) {
                    double input_target_yaw = atan2(input_targets.find(input)->second->target_ref_coord.y,
                                                    input_targets.find(input)->second->target_ref_coord.x);
                    return angles::shortest_angular_distance(last_target_yaw, input_target_yaw) <=
                               angles::from_degrees(1.5) ||
                           angles::shortest_angular_distance(last_target_yaw, input_target_yaw) >= M_PI / 2;
                });
            if (aft_remove_end_target_keys_iter == target_keys_.begin())
                return *input_targets.find(*last_target_key_iter);
            auto output_ptr =
                std::min_element(target_keys_.begin(), aft_remove_end_target_keys_iter, min_yaw_orientation_f);
            last_target_key_opt_ = *output_ptr;
            return *input_targets.find(*output_ptr);
        } else {  // input_mcu_data.switch_mode == MCU::ReceiveInfo::SwitchMode::RESET
            auto output_ptr = std::min_element(target_keys_.begin(), target_keys_.end(), min_yaw_orientation_f);
            last_target_key_opt_ = *output_ptr;
            return *input_targets.find(*output_ptr);
        }
    }
}

std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> ErgoTargetSelector::AutoModeSelect(
    const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_targets,
    MCU::Orders input_order_data) {
    target_weighting_map_.clear();

    std::for_each(target_keys_.begin(), target_keys_.end(), [this, &input_order_data, &input_targets](auto& input_key) {
        // 非防守模式屏蔽工程
        auto arid = input_key;
        arid;
        if (input_order_data.sentry_behavior_mode != MCU::AutoaimReceiveInfo::SentryBehaviorMode::DEFEND &&
            input_key == RM::TargetClassify::ENGINEER_2)
            return;

        // 如果地方前哨战血量不为0 屏蔽哨兵 25赛季移除
        // if (input_order_data.enemy_hp[RM::TargetClassify::SENTRY] != 0 && input_key == RM::TargetClassify::SENTRY)
        //     return; 

        int target_hp;
        switch (input_key) {
            case RM::TargetClassify::BASE_BOTTOM:
            case RM::TargetClassify::BASE_DART:
            case RM::TargetClassify::BASE_TOP: {
                target_hp = input_order_data.enemy_hp[RM::TargetClassify::BASE_DART];
                break;
            }
            case RM::TargetClassify::OUTPOST_DART:
            case RM::TargetClassify::OUTPOST_SPIN: {
                target_hp = input_order_data.enemy_hp[RM::TargetClassify::OUTPOST_DART];
                break;
            }
            case RM::TargetClassify::HERO_1: {
                target_hp = input_order_data.enemy_hp[RM::TargetClassify::HERO_1];
                break;
            }
            case RM::TargetClassify::ENGINEER_2: {
                target_hp = input_order_data.enemy_hp[RM::TargetClassify::ENGINEER_2];
                break;
            }
            case RM::TargetClassify::STANDARD_3: {
                target_hp = input_order_data.enemy_hp[RM::TargetClassify::STANDARD_3];
                break;
            }
            case RM::TargetClassify::STANDARD_4: {
                target_hp = input_order_data.enemy_hp[RM::TargetClassify::STANDARD_4];
                break;
            }
            case RM::TargetClassify::STANDARD_5: {
                target_hp = input_order_data.enemy_hp[RM::TargetClassify::STANDARD_5];
                break;
            }
            case RM::TargetClassify::SENTRY: {
                target_hp = input_order_data.enemy_hp[RM::TargetClassify::SENTRY];
                break;
            }
            case RM::TargetClassify::POWER_RUNE: {
                target_hp = 1;
                break;
            }
        }
        cv::Point3d ref_pose = input_targets.find(input_key)->second->target_ref_coord;
        auto distance = GetDistance(ref_pose.x, ref_pose.y, ref_pose.z);
        auto weighting = param_.auto_mode_inverse_func_a / distance;
        if (target_hp <= param_.auto_mode_low_hp_threshold)
            weighting += param_.auto_mode_low_hp_add_weighting;
        this->target_weighting_map_.insert(std::make_pair(input_key, weighting));
    });

    auto max_weighting_pair_iter = std::max_element(target_weighting_map_.begin(), target_weighting_map_.end(),
                                                    [](auto& a, auto& b) { return a.second < b.second; });
    if (max_weighting_pair_iter == target_weighting_map_.end()) {
        last_target_key_opt_ = std::nullopt;
        return std::nullopt;
    }

    if (last_target_key_opt_.has_value()) {
        if (last_target_key_opt_.value() == max_weighting_pair_iter->first) {
            return *input_targets.find(max_weighting_pair_iter->first);
        } else {
            if (target_weighting_map_.contains(last_target_key_opt_.value())) {
                if (max_weighting_pair_iter->second - target_weighting_map_.at(last_target_key_opt_.value()) >
                    param_.auto_mode_target_switch_threshold) {
                    last_target_key_opt_ = max_weighting_pair_iter->first;
                    return *input_targets.find(max_weighting_pair_iter->first);
                } else {
                    return *input_targets.find(last_target_key_opt_.value());
                }

            } else {
                last_target_key_opt_ = max_weighting_pair_iter->first;
                return *input_targets.find(max_weighting_pair_iter->first);
            }
        }
    } else {
        last_target_key_opt_ = max_weighting_pair_iter->first;
        return *input_targets.find(max_weighting_pair_iter->first);
    }
}

std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> ErgoTargetSelector::HeroModeSelect(
    const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_targets) {
    auto hero_iter = std::find(target_keys_.begin(), target_keys_.end(), RM::TargetClassify::HERO_1);
    if (hero_iter == target_keys_.end()) {
        last_target_key_opt_ = std::nullopt;
        return std::nullopt;
    }

    else
        return *input_targets.find(*hero_iter);
}