//
// Created by wpie on 23-11-5.
//

#include "estimate_node/multichannel_tracker/power_rune_tracker/power_rune_tracker.h"
#include "estimate_node/multichannel_tracker/power_rune_tracker/slpr_filter.h"
#include "vofa_bridge/vofa_bridge.h"

using namespace MT;
using namespace MT::PT;

void PowerRuneTracker::Run(std::shared_ptr<MT::ITargetInfo>& input_target,
                           const MCU::Orders& input_order_data) {
    auto input_power_rune_target = std::static_pointer_cast<PowerRuneTargetInfo>(input_target);
    auto time = input_target->GetMyTimestamp();  // std::chrono::steady_clock::now();
    if(last_time_ == 0)
        last_time_ = time;
    auto d_t = time - last_time_;  // TODO : test here
    last_time_ = time;

    // 该情境下不涉及到卡尔曼的持续更新需�?
    if (input_order_data.remaining_time_s >= param_.power_rune_mode_judge_remaining_time_threshold_s) {
        if (sspr_filter_->GetFilterState() == SSPRFilter::FilterState::LOST) {
            sspr_filter_->SetFilterState(SSPRFilter::FilterState::DIR_JUDGE);
            input_target->target_available = false;
        } else {
            if (sspr_filter_->GetFilterState() == SSPRFilter::FilterState::DIR_JUDGE) {
                sspr_filter_->DirJudge(input_power_rune_target->fans_vec, d_t);

            } else {
                auto [state, is_clockwise] = sspr_filter_->Update(input_power_rune_target->fans_vec, d_t);
                input_target->target_available = false;

                if (sspr_filter_->GetFilterState() == SSPRFilter::FilterState::DETECTING) {
                    input_target->target_available = false;
                } else if (sspr_filter_->GetFilterState() == SSPRFilter::FilterState::TRACKING ||
                           sspr_filter_->GetFilterState() == SSPRFilter::FilterState::TEMP_LOST) {
                    input_power_rune_target->target_available = true;

                    PowerRuneTargetInfo::SSPR info;
                    info.bull_roll = state(0);
                    info.bull_abs_v_roll = state(1);
                    info.geographic_coord_r_symbol_pose.x = state(2);
                    info.geographic_coord_r_symbol_pose.y = state(3);
                    info.geographic_coord_r_symbol_pose.z = state(4);
                    info.is_clockwise = is_clockwise;
                    input_power_rune_target->motion_info = info;
                    input_power_rune_target->target_ref_coord = info.geographic_coord_r_symbol_pose;
                    vpie::VofaBridge::Get().SendOnce((float)state(0), (float)state(1), (float)state(2), (float)state(3), (float)state(4));
                }
            }
        }

        //        if (small_power_rune_filter_->GetFilterState() == SmallPowerRuneFilter::FilterState::LOST) {
        //            small_power_rune_filter_->SetFilterState(SmallPowerRuneFilter::FilterState::DIR_JUDGE);
        //            input_target->target_available = false;
        //        } else {
        //            if (small_power_rune_filter_->GetFilterState() == SmallPowerRuneFilter::FilterState::DIR_JUDGE) {
        //                small_power_rune_filter_->DirJudge(input_power_rune_target->fans_vec, d_t);
        //
        //            } else {
        //                auto [state, is_clockwise] =
        //                small_power_rune_filter_->Update(input_power_rune_target->fans_vec, d_t);
        //                input_target->target_available = false;
        //
        //                if (small_power_rune_filter_->GetFilterState() ==
        //                SmallPowerRuneFilter::FilterState::DETECTING) {
        //                    input_target->target_available = false;
        //                } else if (
        //                        small_power_rune_filter_->GetFilterState() ==
        //                        SmallPowerRuneFilter::FilterState::TRACKING ||
        //                        small_power_rune_filter_->GetFilterState() ==
        //                        SmallPowerRuneFilter::FilterState::TEMP_LOST) {
        //                    input_power_rune_target->target_available = true;
        //
        //                    PowerRuneTargetInfo::SmallPowerRune info;
        //                    info.bull_roll = state(0);
        //                    info.bull_abs_v_roll = state(1);
        //                    info.radius = state(2);
        //                    info.geographic_coord_r_symbol_pose.x = state(3);
        //                    info.geographic_coord_r_symbol_pose.y = state(4);
        //                    info.geographic_coord_r_symbol_pose.z = state(5);
        //                    info.is_clockwise = is_clockwise;
        //                    input_power_rune_target->motion_info = info;
        //                    input_power_rune_target->target_ref_coord = info.geographic_coord_r_symbol_pose;
        //                    v_8.Send((float) state(0), (float) state(1), (float) state(2), (float) state(3), (float)
        //                    state(4),
        //                             (float) state(5));
        //                }
        //            }
        //        }
    } else {
        // 大能量机�?

        //     if (large_power_rune_filter_->GetFilterState() == LargePowerRuneFilter::FilterState::LOST) {
        //         large_power_rune_filter_->SetFilterState(LargePowerRuneFilter::FilterState::DIR_JUDGE);
        //         input_target->target_available = false;
        //     } else {
        //         if (large_power_rune_filter_->GetFilterState() == LargePowerRuneFilter::FilterState::DIR_JUDGE) {
        //             large_power_rune_filter_->DirJudge(input_power_rune_target->fans_vec, d_t);
        //             large_power_rune_filter_->Init(input_power_rune_target->fans_vec);
        //         } else {
        //             auto [state, is_clockwise] = large_power_rune_filter_->Update(input_power_rune_target->fans_vec,
        //             d_t); input_target->target_available = false;

        //             if (large_power_rune_filter_->GetFilterState() == LargePowerRuneFilter::FilterState::DETECTING) {
        //                 input_target->target_available = false;
        //             } else if (large_power_rune_filter_->GetFilterState() ==
        //             LargePowerRuneFilter::FilterState::TRACKING ||
        //                        large_power_rune_filter_->GetFilterState() ==
        //                        LargePowerRuneFilter::FilterState::TEMP_LOST) {
        //                 input_power_rune_target->target_available = true;

        //                 PowerRuneTargetInfo::LargePowerRune info;
        //                 info.bull_roll = state(0);
        //                 info.bull_abs_v_roll = state(1);
        //                 info.func_a = state(2);
        //                 info.func_omega = state(3);
        //                 info.func_phi = state(4);
        //                 info.radius = state(5);
        //                 info.geographic_coord_r_symbol_pose.x = state(6);
        //                 info.geographic_coord_r_symbol_pose.y = state(7);
        //                 info.geographic_coord_r_symbol_pose.z = state(8);
        //                 info.is_clockwise = is_clockwise;
        //                 input_power_rune_target->motion_info = info;
        //                 input_power_rune_target->target_ref_coord = info.geographic_coord_r_symbol_pose;
        //             }
        //         }
        //     }
        // }

        if (slpr_filter_->GetFilterState() == SLPRFilter::FilterState::LOST) {
            slpr_filter_->SetFilterState(SLPRFilter::FilterState::DIR_JUDGE);
            input_target->target_available = false;
        } else {
            if (slpr_filter_->GetFilterState() == SLPRFilter::FilterState::DIR_JUDGE) {
                slpr_filter_->DirJudge(input_power_rune_target->fans_vec, d_t);
            } else {
                auto [state, is_clockwise] = slpr_filter_->Update(input_power_rune_target->fans_vec, d_t);
                input_target->target_available = false;

                if (slpr_filter_->GetFilterState() == SLPRFilter::FilterState::DETECTING) {
                    input_target->target_available = false;
                } else if (slpr_filter_->GetFilterState() == SLPRFilter::FilterState::TRACKING ||
                           slpr_filter_->GetFilterState() == SLPRFilter::FilterState::TEMP_LOST) {
                    input_power_rune_target->target_available = true;

                    PowerRuneTargetInfo::SLPR info;
                    info.bull_roll = state(0);
                    info.bull_abs_v_roll = state(1);
                    info.func_a = state(2);
                    info.func_omega = state(3);
                    info.func_phi = state(4);
                    info.geographic_coord_r_symbol_pose.x = state(5);
                    info.geographic_coord_r_symbol_pose.y = state(6);
                    info.geographic_coord_r_symbol_pose.z = state(7);
                    info.t_sum = state(8);

                    info.is_clockwise = is_clockwise;
                    input_power_rune_target->motion_info = info;
                    input_power_rune_target->target_ref_coord = info.geographic_coord_r_symbol_pose;

                    // vpie::VofaBridge::Get().SendOnce((float)state(0), (float)state(1), (float)state(2), (float)state(3), (float)state(4),
                    //          (float)state(5), (float)state(6), (float)state(7), (float)state(8));
                    // std::cout << state(0) << std::endl;
                }
            }
        }
    }
}
