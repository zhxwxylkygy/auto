//
// Created by wpie on 23-11-2.
//

#include "estimate_node/multichannel_tracker/multichannel_tracker.h"
#include <execution>
#include <utility>
#include "estimate_node/interface/rm_interface.h"

using namespace MT;

MultichannelTracker::MultichannelTracker() {
    trackers_.emplace(RM::TargetClassify::BASE_DART, std::make_unique<ST::StaticTracker>());
    trackers_.emplace(RM::TargetClassify::BASE_BOTTOM, std::make_unique<ST::StaticTracker>());
    trackers_.emplace(RM::TargetClassify::BASE_TOP, std::make_unique<ST::StaticTracker>());
    trackers_.emplace(RM::TargetClassify::OUTPOST_DART, std::make_unique<ST::StaticTracker>());
    trackers_.emplace(RM::TargetClassify::OUTPOST_SPIN, std::make_unique<IRVT::IRVTracker>(false));
    trackers_.emplace(RM::TargetClassify::HERO_1, std::make_unique<IRVT::IRVTracker>(true));
    trackers_.emplace(RM::TargetClassify::ENGINEER_2, std::make_unique<IRVT::IRVTracker>(false));
    trackers_.emplace(RM::TargetClassify::STANDARD_3, std::make_unique<IRVT::IRVTracker>(false));
    trackers_.emplace(RM::TargetClassify::STANDARD_4, std::make_unique<IRVT::IRVTracker>(false));
    trackers_.emplace(RM::TargetClassify::STANDARD_5, std::make_unique<IRVT::IRVTracker>(false));
    trackers_.emplace(RM::TargetClassify::SENTRY, std::make_unique<IRVT::IRVTracker>(false));
    trackers_.emplace(RM::TargetClassify::POWER_RUNE, std::make_unique<MT::PT::PowerRuneTracker>());
}

void MultichannelTracker::Run(std::map<RM::TargetClassify, std::shared_ptr<ITargetInfo>>& input_targets,
                              std::vector<std::shared_ptr<FP::IFeature>> input_features,
                              const MCU::Orders& input_order_data) {
    Classified(input_targets, std::move(input_features), input_order_data);
    Track(input_targets, input_order_data);
}

void MultichannelTracker::Classified(std::map<RM::TargetClassify, std::shared_ptr<ITargetInfo>>& input_targets,
                                     std::vector<std::shared_ptr<FP::IFeature>> input_features,
                                     const MCU::Orders& input_order_data) {
    auto base_dart = std::static_pointer_cast<MT::StaticTargetInfo>(input_targets.at(RM::TargetClassify::BASE_DART));
    auto base_bottom =
        std::static_pointer_cast<MT::StaticTargetInfo>(input_targets.at(RM::TargetClassify::BASE_BOTTOM));
    auto base_top = std::static_pointer_cast<MT::StaticTargetInfo>(input_targets.at(RM::TargetClassify::BASE_TOP));
    auto outpost_dart =
        std::static_pointer_cast<MT::StaticTargetInfo>(input_targets.at(RM::TargetClassify::OUTPOST_DART));
    auto outpost_spin =
        std::static_pointer_cast<MT::StaticTargetInfo>(input_targets.at(RM::TargetClassify::OUTPOST_SPIN));
    auto hero_1 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::HERO_1));
    auto engineer_2 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::ENGINEER_2));
    auto standard_3 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::STANDARD_3));
    auto standard_4 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::STANDARD_4));
    auto standard_5 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::STANDARD_5));
    auto sentry = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::SENTRY));
    auto power_rune =
        std::static_pointer_cast<MT::PowerRuneTargetInfo>(input_targets.at(RM::TargetClassify::POWER_RUNE));

    // 清空上一次内容，释放指针
    std::for_each(input_targets.begin(), input_targets.end(), [](auto input_pair) {
        if (input_pair.first == RM::TargetClassify::BASE_BOTTOM || input_pair.first == RM::TargetClassify::BASE_DART ||
            input_pair.first == RM::TargetClassify::BASE_TOP || input_pair.first == RM::TargetClassify::OUTPOST_DART) {
            std::static_pointer_cast<StaticTargetInfo>(input_pair.second)->armors_vec.clear();
        } else if (input_pair.first == RM::TargetClassify::POWER_RUNE) {
            std::static_pointer_cast<PowerRuneTargetInfo>(input_pair.second)->fans_vec.clear();
        } else {
            std::static_pointer_cast<RVTargetInfo>(input_pair.second)->armors_vec.clear();
        }
    });

    // 分类装甲板
    std::for_each(input_features.begin(), input_features.end(),
                  [&base_dart, &base_bottom, &base_top, &outpost_dart, &outpost_spin, &hero_1, &engineer_2, &standard_3,
                   &standard_4, &standard_5, &sentry, &power_rune,
                   &input_order_data](std::shared_ptr<FP::IFeature>& input_feature) {
                      if (auto armor = std::dynamic_pointer_cast<FP::ArmorInfo>(input_feature); armor != nullptr) {
                          
                          switch (armor->armor_id) {
                              case RM::ArmorId::BASE:
                                  if (armor->armor_size == RM::ArmorSize::LARGE) {
                                      base_bottom->armors_vec.push_back(armor);
                                  } else if (armor->armor_size == RM::ArmorSize::SMALL) {
                                      base_top->armors_vec.push_back(armor);
                                  }
                                  break;
                              case RM::ArmorId::OUTPOST:
                                  outpost_spin->armors_vec.push_back(armor);
                                  break;

                              case RM::ArmorId::SENTRY:
                                  sentry->armors_vec.push_back(armor);
                                  break;
                              case RM::ArmorId::ONE:
                                  hero_1->armors_vec.push_back(armor);
                                  break;
                              case RM::ArmorId::TWO:
                                  engineer_2->armors_vec.push_back(armor);
                                  break;
                              case RM::ArmorId::THREE:
                                  if (armor->armor_size != RM::ArmorSize::INVALID) {
                                      standard_3->armors_vec.push_back(armor);
                                  } else {
                                      std::cout << "MT: classified fail" << std::endl;
                                      std::terminate();
                                  }
                                  break;
                              case RM::ArmorId::FOUR:
                                  if (armor->armor_size != RM::ArmorSize::INVALID) {
                                      standard_4->armors_vec.push_back(armor);
                                  } else {
                                      std::cout << "MT: classified fail" << std::endl;
                                      std::terminate();
                                  }
                                  break;
                              case RM::ArmorId::FIVE:
                                  if (armor->armor_size != RM::ArmorSize::INVALID) {
                                      standard_5->armors_vec.push_back(armor);
                                  } else {
                                      std::cout << "MT: classified fail" << std::endl;
                                      std::terminate();
                                  }
                                  break;
                              case RM::ArmorId::INVALID:
                                  if (armor->is_dart_armor) {
                                      if (input_order_data.enemy_hp.at(RM::TargetClassify::OUTPOST_DART) > 0) {
                                          outpost_dart->armors_vec.push_back(armor);
                                      } else {
                                          base_dart->armors_vec.push_back(armor);
                                      }
                                  }
                                  break;
                          }
                      } else if (auto fan = std::dynamic_pointer_cast<FP::FanInfo>(input_feature); fan != nullptr) {
                          power_rune->fans_vec.push_back(fan);
                      }
                  });

    std::for_each(input_targets.begin(), input_targets.end(), [](auto& input_pair) {
        if (input_pair.first == RM::TargetClassify::BASE_BOTTOM || input_pair.first == RM::TargetClassify::BASE_DART ||
            input_pair.first == RM::TargetClassify::BASE_TOP || input_pair.first == RM::TargetClassify::OUTPOST_DART) {
            auto static_target = std::static_pointer_cast<StaticTargetInfo>(input_pair.second);
            if (!static_target->armors_vec.empty()) {
                static_target->armor_size = static_target->armors_vec.at(0)->armor_size;
            } else {
                static_target->armor_size = RM::ArmorSize::INVALID;
            }
        } else if (input_pair.first == RM::TargetClassify::POWER_RUNE) {
        } else {
            auto rv_target = std::static_pointer_cast<RVTargetInfo>(input_pair.second);
            if (!rv_target->armors_vec.empty()) {
                rv_target->armor_size = rv_target->armors_vec.at(0)->armor_size;
            } else {
                rv_target->armor_size = RM::ArmorSize::INVALID;
            }
        }
    });

    // 步兵构型判定
    if (!standard_3->armors_vec.empty())
        standard_3->target_armor_num =
            standard_3->armor_size == RM::ArmorSize::LARGE ? RM::TargetArmorNum::TWO : RM::TargetArmorNum::FOUR;

    if (!standard_4->armors_vec.empty())
        standard_4->target_armor_num =
            standard_4->armor_size == RM::ArmorSize::LARGE ? RM::TargetArmorNum::TWO : RM::TargetArmorNum::FOUR;
    if (!standard_5->armors_vec.empty())
        standard_5->target_armor_num =
            standard_5->armor_size == RM::ArmorSize::LARGE ? RM::TargetArmorNum::TWO : RM::TargetArmorNum::FOUR;
}

void MultichannelTracker::Track(std::map<RM::TargetClassify, std::shared_ptr<ITargetInfo>>& input_targets,
                                const MCU::Orders& input_order_data) {
    //    std::execution::par_unseq
    std::for_each(std::execution::par_unseq, input_targets.begin(), input_targets.end(),
                  [this, &input_order_data](auto& input_target_pair) {
                      trackers_.at(input_target_pair.first)->Run(input_target_pair.second, input_order_data);
                  });
}

void MultichannelTracker::InitTargetVec(std::map<RM::TargetClassify, std::shared_ptr<ITargetInfo>>& input_targets) {
    auto base_dart = std::static_pointer_cast<MT::StaticTargetInfo>(input_targets.at(RM::TargetClassify::BASE_DART));
    base_dart->armor_id = RM::ArmorId::INVALID;
    base_dart->target_armor_num = RM::TargetArmorNum::ONE;

    auto base_bottom =
        std::static_pointer_cast<MT::StaticTargetInfo>(input_targets.at(RM::TargetClassify::BASE_BOTTOM));
    base_bottom->armor_id = RM::ArmorId::BASE;
    base_bottom->target_armor_num = RM::TargetArmorNum::ONE;

    auto base_top = std::static_pointer_cast<MT::StaticTargetInfo>(input_targets.at(RM::TargetClassify::BASE_TOP));
    base_top->armor_id = RM::ArmorId::BASE;
    base_top->target_armor_num = RM::TargetArmorNum::ONE;

    auto outpost_dart =
        std::static_pointer_cast<MT::StaticTargetInfo>(input_targets.at(RM::TargetClassify::OUTPOST_DART));
    outpost_dart->armor_id = RM::ArmorId::INVALID;
    outpost_dart->target_armor_num = RM::TargetArmorNum::ONE;

    auto outpost_spin = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::OUTPOST_SPIN));
    outpost_spin->armor_id = RM::ArmorId::OUTPOST;
    outpost_spin->target_armor_num = RM::TargetArmorNum::THREE;

    auto hero_1 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::HERO_1));
    hero_1->armor_id = RM::ArmorId::ONE;
    hero_1->target_armor_num = RM::TargetArmorNum::FOUR;

    auto engineer_2 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::ENGINEER_2));
    engineer_2->armor_id = RM::ArmorId::TWO;
    engineer_2->target_armor_num = RM::TargetArmorNum::FOUR;

    auto standard_3 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::STANDARD_3));
    standard_3->armor_id = RM::ArmorId::THREE;
    standard_3->target_armor_num = RM::TargetArmorNum::UNCERTAIN;

    auto standard_4 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::STANDARD_4));
    standard_4->armor_id = RM::ArmorId::FOUR;
    standard_4->target_armor_num = RM::TargetArmorNum::UNCERTAIN;

    auto standard_5 = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::STANDARD_5));
    standard_5->armor_id = RM::ArmorId::FIVE;
    standard_5->target_armor_num = RM::TargetArmorNum::UNCERTAIN;

    auto sentry = std::static_pointer_cast<MT::RVTargetInfo>(input_targets.at(RM::TargetClassify::SENTRY));
    sentry->armor_id = RM::ArmorId::SENTRY;
    sentry->target_armor_num = RM::TargetArmorNum::FOUR;

    auto power_rune =
        std::static_pointer_cast<MT::PowerRuneTargetInfo>(input_targets.at(RM::TargetClassify::POWER_RUNE));
}
