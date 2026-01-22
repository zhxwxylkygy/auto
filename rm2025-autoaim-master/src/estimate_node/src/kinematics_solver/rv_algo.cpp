//
// Created by wpie on 23-11-2.
//

#include "estimate_node/kinematics_solver/rv_algo/rv_algo.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include "angles/angles.h"
#include "calculate/basic_calculate.h"
#include "estimate_node/interface/interface.h"
#include "estimate_node/interface/rm_interface.h"
#include "vofa_bridge/vofa_bridge.h"

using namespace KS::RVA;

RVAlgo::RVAlgo(Trajectory& input) : IAlgorithm(input) {
    if (param_.bullet_type == RM::BulletType::D_17MM)
        trajectory_computer_.Init(Trajectory::Mode::BULLET_17MM_SECOND_ORDER_APPROX, param_.bullet_speed);
    else
        trajectory_computer_.Init(Trajectory::Mode::BULLET_42MM_SECOND_ORDER_APPROX, param_.bullet_speed);
}

Target::Armor ArmorSelector::Run(const std::vector<Target::Armor>& armors, bool is_clockwise_rotate, Method method) {
    if (method == Method::MIN_DISTANCE) {
        return *std::min_element(armors.begin(), armors.end(), [](Target::Armor a, Target::Armor b) {
            double d_a = cv::norm(a.armor_in_chassis.p);
            double d_b = cv::norm(b.armor_in_chassis.p);
            return d_a < d_b;
        });

    } else if (method == Method::MIN_RELATIVE) {
        auto i = std::min_element(armors.begin(), armors.end(), [](Target::Armor a, Target::Armor b) {
            return fabs(angles::shortest_angular_distance(a.chassis_in_armor.yaw, angles::from_degrees(0))) <
                   fabs(angles::shortest_angular_distance(b.chassis_in_armor.yaw, angles::from_degrees(0)));
        });
        return *i;

    } else {  // method == Method::PREDICTED_MIN_OFFSET_RELATIVE
        auto i = std::min_element(
            armors.begin(), armors.end(), [this, &is_clockwise_rotate](Target::Armor a, Target::Armor b) {
                return fabs(angles::shortest_angular_distance(
                           a.chassis_in_armor.yaw,
                           angles::from_degrees(is_clockwise_rotate ? -param_.switch_offset_angle
                                                                    : param_.switch_offset_angle))) <
                       fabs(angles::shortest_angular_distance(
                           b.chassis_in_armor.yaw,
                           angles::from_degrees(is_clockwise_rotate ? -param_.switch_offset_angle
                                                                    : param_.switch_offset_angle)));
            });
        return *i;
    }
}

Target RVAlgo::LoadTargetInfo(const std::shared_ptr<MT::RVTargetInfo>& input_target) {
    Target target_info;
    if (input_target->target_armor_num == RM::TargetArmorNum::FOUR) {
        target_info.armors.resize(4);
    } else if (input_target->target_armor_num == RM::TargetArmorNum::THREE) {
        target_info.armors.resize(3);
    } else if (input_target->target_armor_num == RM::TargetArmorNum::TWO) {
        target_info.armors.resize(2);
    } else {
        std::cout << "RVTrackerAlgo: wrong target_classify armor num!" << std::endl;
        std::terminate();
    }

    target_info.target_in_chassis.p = input_target->c_position;
    target_info.target_in_chassis.v = input_target->c_velocity;
    target_info.v_autoroatation = input_target->v_yaw;
    if (input_target->armor_id == RM::ArmorId::OUTPOST && param_.normalized_outpost_angular_velocity) {
        if (input_target->v_yaw >= 1.5)
            target_info.v_autoroatation = 2.5;
        else if (input_target->v_yaw <= -1.5)
            target_info.v_autoroatation = -2.5;
    }

    std::cout << target_info.v_autoroatation << std::endl;
    target_info.armor_r1 = input_target->r_1;
    target_info.armor_r2 = input_target->r_2;
    target_info.d_z = input_target->d_z;
    target_info.armor_size = input_target->armor_size;

    auto main_armor_orientation = angles::normalize_angle(input_target->yaw + M_PI);

    if (input_target->target_armor_num == RM::TargetArmorNum::FOUR) {
        for (auto i = target_info.armors.begin(); i != target_info.armors.end(); i++) {
            i->armor_in_target.yaw =
                main_armor_orientation +
                angles::from_degrees(90) * static_cast<int>(std::distance(target_info.armors.begin(), i));
            i->armor_in_target.yaw = angles::normalize_angle(i->armor_in_target.yaw);
            if (std::distance(target_info.armors.begin(), i) == 1 ||
                std::distance(target_info.armors.begin(), i) == 3) {
                auto result = GetVec(target_info.armor_r2, i->armor_in_target.yaw);
                i->armor_in_chassis.p.x = target_info.target_in_chassis.p.x + result.x;
                i->armor_in_chassis.p.y = target_info.target_in_chassis.p.y + result.y;
                i->armor_in_chassis.p.z = target_info.target_in_chassis.p.z + target_info.d_z;
            } else {
                auto result = GetVec(target_info.armor_r1, i->armor_in_target.yaw);
                i->armor_in_chassis.p.x = target_info.target_in_chassis.p.x + result.x;
                i->armor_in_chassis.p.y = target_info.target_in_chassis.p.y + result.y;
                i->armor_in_chassis.p.z = target_info.target_in_chassis.p.z;
            }
        }
    } else if (input_target->target_armor_num == RM::TargetArmorNum::THREE) {
        for (auto i = target_info.armors.begin(); i != target_info.armors.end(); i++) {
            i->armor_in_target.yaw =
                main_armor_orientation +
                angles::from_degrees(120) * static_cast<int>(std::distance(target_info.armors.begin(), i));
            i->armor_in_target.yaw = angles::normalize_angle(i->armor_in_target.yaw);
            auto result = GetVec(target_info.armor_r2, i->armor_in_target.yaw);
            i->armor_in_chassis.p.x = target_info.target_in_chassis.p.x + result.x;
            i->armor_in_chassis.p.y = target_info.target_in_chassis.p.y + result.y;
            i->armor_in_chassis.p.z = target_info.target_in_chassis.p.z;
        }
    } else if (input_target->target_armor_num == RM::TargetArmorNum::TWO) {
        for (auto i = target_info.armors.begin(); i != target_info.armors.end(); i++) {
            i->armor_in_target.yaw =
                main_armor_orientation +
                angles::from_degrees(180) * static_cast<int>(std::distance(target_info.armors.begin(), i));
            i->armor_in_target.yaw = angles::normalize_angle(i->armor_in_target.yaw);
            auto result = GetVec(target_info.armor_r2, i->armor_in_target.yaw);
            i->armor_in_chassis.p.x = target_info.target_in_chassis.p.x + result.x;
            i->armor_in_chassis.p.y = target_info.target_in_chassis.p.y + result.y;
            i->armor_in_chassis.p.z = target_info.target_in_chassis.p.z;
        }
    } else {
        std::cout << "RVTrackerAlgo: wrong target_classify armor num!" << std::endl;
        std::terminate();
    }
    auto& v = vpie::VofaBridge::Get();

    if (target_info.armors.size() == 4) {
        v.SendOnce(
            (float)target_info.armors.at(0).armor_in_chassis.p.x, (float)target_info.armors.at(0).armor_in_chassis.p.y,
            (float)target_info.armors.at(1).armor_in_chassis.p.x, (float)target_info.armors.at(1).armor_in_chassis.p.y,
            (float)target_info.armors.at(2).armor_in_chassis.p.x, (float)target_info.armors.at(2).armor_in_chassis.p.y,
            (float)target_info.armors.at(3).armor_in_chassis.p.x, (float)target_info.armors.at(3).armor_in_chassis.p.y,
            (float)target_info.target_in_chassis.p.x, (float)target_info.target_in_chassis.p.y);
    } else if (target_info.armors.size() == 3) {
        v.SendOnce(
            (float)target_info.armors.at(0).armor_in_chassis.p.x, (float)target_info.armors.at(0).armor_in_chassis.p.y,
            (float)target_info.armors.at(1).armor_in_chassis.p.x, (float)target_info.armors.at(1).armor_in_chassis.p.y,
            (float)target_info.armors.at(2).armor_in_chassis.p.x, (float)target_info.armors.at(2).armor_in_chassis.p.y,
            (float)target_info.target_in_chassis.p.x, (float)target_info.target_in_chassis.p.y);

    } else if (target_info.armors.size() == 2) {
        v.SendOnce(
            (float)target_info.armors.at(0).armor_in_chassis.p.x, (float)target_info.armors.at(0).armor_in_chassis.p.y,
            (float)target_info.armors.at(1).armor_in_chassis.p.x, (float)target_info.armors.at(1).armor_in_chassis.p.y,
            (float)target_info.target_in_chassis.p.x, (float)target_info.target_in_chassis.p.y);
    }
    return target_info;
}

Target Target::PredictTime(std::chrono::milliseconds duration_time_ms) {
    double d_t = static_cast<double>(duration_time_ms.count()) / 1000.;

    auto predicted_target = this->CloneConstantInfo();

    predicted_target.target_in_chassis.p = this->target_in_chassis.p + this->target_in_chassis.v * d_t;

    for (int i = 0; i < static_cast<int>(this->armors.size()); i++) {
        predicted_target.armors.at(i).armor_in_target.yaw =
            this->armors.at(i).armor_in_target.yaw + this->v_autoroatation * d_t;
    }

    if (this->armors.size() == 4) {
        for (int i = 0; i < static_cast<int>(this->armors.size()); i++) {
            if (i == 1 || i == 3) {
                auto result = GetVec(predicted_target.armor_r2, predicted_target.armors.at(i).armor_in_target.yaw);
                predicted_target.armors.at(i).armor_in_chassis.p.x = predicted_target.target_in_chassis.p.x + result.x;
                predicted_target.armors.at(i).armor_in_chassis.p.y = predicted_target.target_in_chassis.p.y + result.y;
                predicted_target.armors.at(i).armor_in_chassis.p.z =
                    predicted_target.target_in_chassis.p.z + predicted_target.d_z;
            } else {
                auto result = GetVec(predicted_target.armor_r1, predicted_target.armors.at(i).armor_in_target.yaw);
                predicted_target.armors.at(i).armor_in_chassis.p.x = predicted_target.target_in_chassis.p.x + result.x;
                predicted_target.armors.at(i).armor_in_chassis.p.y = predicted_target.target_in_chassis.p.y + result.y;
                predicted_target.armors.at(i).armor_in_chassis.p.z = predicted_target.target_in_chassis.p.z;
            }
        }
    } else if (this->armors.size() == 3 || this->armors.size() == 2) {
        for (int i = 0; i < static_cast<int>(this->armors.size()); i++) {
            auto result = GetVec(predicted_target.armor_r1, predicted_target.armors.at(i).armor_in_target.yaw);
            predicted_target.armors.at(i).armor_in_chassis.p.x = predicted_target.target_in_chassis.p.x + result.x;
            predicted_target.armors.at(i).armor_in_chassis.p.y = predicted_target.target_in_chassis.p.y + result.y;
            predicted_target.armors.at(i).armor_in_chassis.p.z = predicted_target.target_in_chassis.p.z;
        }
    } else {
        std::cout << "RVTrackerAlgo: wrong target_classify armor num!" << std::endl;
        std::terminate();
    }

    for (auto& armor : predicted_target.armors) {
        auto angle = armor.armor_in_target.yaw;

        auto result = TransformPoint(cv::Point2d(0, 0),
                                     cv::Point2d(armor.armor_in_chassis.p.x, armor.armor_in_chassis.p.y), -angle);
        armor.chassis_in_armor.yaw = GetYaw(result.x, result.y);
    }

    auto& v = vpie::VofaBridge::Get();

    // if (predicted_target.armors.size() == 4) {
    //     v.SendOnce(
    //         (float)predicted_target.armors.at(0).armor_in_chassis.p.x,
    //         (float)predicted_target.armors.at(0).armor_in_chassis.p.y,
    //         (float)predicted_target.armors.at(1).armor_in_chassis.p.x,
    //         (float)predicted_target.armors.at(1).armor_in_chassis.p.y,
    //         (float)predicted_target.armors.at(2).armor_in_chassis.p.x,
    //         (float)predicted_target.armors.at(2).armor_in_chassis.p.y,
    //         (float)predicted_target.armors.at(3).armor_in_chassis.p.x,
    //         (float)predicted_target.armors.at(3).armor_in_chassis.p.y, (float)predicted_target.target_in_chassis.p.x,
    //         (float)predicted_target.target_in_chassis.p.y);
    // } else if (predicted_target.armors.size() == 3) {
    //     v.SendOnce(
    //         (float)predicted_target.armors.at(0).armor_in_chassis.p.x,
    //         (float)predicted_target.armors.at(0).armor_in_chassis.p.y,
    //         (float)predicted_target.armors.at(1).armor_in_chassis.p.x,
    //         (float)predicted_target.armors.at(1).armor_in_chassis.p.y,
    //         (float)predicted_target.armors.at(2).armor_in_chassis.p.x,
    //         (float)predicted_target.armors.at(2).armor_in_chassis.p.y, (float)predicted_target.target_in_chassis.p.x,
    //         (float)predicted_target.target_in_chassis.p.y);

    // } else if (predicted_target.armors.size() == 2) {
    //     v.SendOnce(
    //         (float)predicted_target.armors.at(0).armor_in_chassis.p.x,
    //         (float)predicted_target.armors.at(0).armor_in_chassis.p.y,
    //         (float)predicted_target.armors.at(1).armor_in_chassis.p.x,
    //         (float)predicted_target.armors.at(1).armor_in_chassis.p.y, (float)predicted_target.target_in_chassis.p.x,
    //         (float)predicted_target.target_in_chassis.p.y);
    // }
    return predicted_target;
}

KS::TargetSolution RVAlgo::Run(const std::shared_ptr<MT::ITargetInfo>& input_target, cv::Point3d g_precise_tf_offset) {
    auto rv_target = std::static_pointer_cast<MT::RVTargetInfo>(input_target);
    mode_ = mode_switcher_.Run(mode_, rv_target);

    auto now_target = LoadTargetInfo(rv_target);

    if (mode_ == Mode::CLOSE) {
        auto armor =
            armor_selector_.Run(now_target.armors, now_target.v_autoroatation < 0, ArmorSelector::Method::MIN_DISTANCE);
        auto tra = trajectory_computer_.GetSolution(
            armor.armor_in_chassis.p + (param_.advanced_transform ? g_precise_tf_offset : cv::Point3d(0, 0, 0)));
        TargetSolution sol{GetXOYYaw(armor.armor_in_chassis.p +
                                     (param_.advanced_transform ? g_precise_tf_offset : cv::Point3d(0, 0, 0))),
                           -tra->angle, true};

        shoot_status_ = true;
        return sol;

    } else if (mode_ == Mode::NORMAL) {
        auto tra_to_now_target = trajectory_computer_.GetSolution(now_target.target_in_chassis.p);
        auto pre_target = now_target.PredictTime(std::chrono::milliseconds(
            static_cast<int>(tra_to_now_target->fly_time * 1000. + param_.gimbal_yaw_machine_time_ms)));

        auto armor =
            armor_selector_.Run(pre_target.armors, now_target.v_autoroatation < 0, ArmorSelector::Method::MIN_RELATIVE);

        auto tra_to_predicted_armor = trajectory_computer_.GetSolution(
            armor.armor_in_chassis.p + (param_.advanced_transform ? g_precise_tf_offset : cv::Point3d(0, 0, 0)));
        TargetSolution sol{GetXOYYaw(armor.armor_in_chassis.p +
                                     (param_.advanced_transform ? g_precise_tf_offset : cv::Point3d(0, 0, 0))),
                           -tra_to_predicted_armor->angle, true};

        shoot_status_ = true;
        return sol;

    } else if (mode_ == Mode::ROTATE) {
        auto tra_to_now_target = trajectory_computer_.GetSolution(now_target.target_in_chassis.p);
        auto pre_gimbal_time_target = now_target.PredictTime(std::chrono::milliseconds(
            static_cast<int>(tra_to_now_target->fly_time * 1000. + param_.gimbal_yaw_machine_time_ms)));

        auto pre_enable_shoot_time_target = now_target.PredictTime(std::chrono::milliseconds(
            static_cast<int>(tra_to_now_target->fly_time * 1000. + param_.enable_shoot_machine_time_ms)));

        auto pre_disable_shoot_time_target = now_target.PredictTime(std::chrono::milliseconds(
            static_cast<int>(tra_to_now_target->fly_time * 1000. + param_.disable_shoot_machine_time_ms)));

        auto pre_gimbal_time_armor = armor_selector_.Run(pre_gimbal_time_target.armors, now_target.v_autoroatation < 0,
                                                         ArmorSelector::Method::MIN_OFFSET_RELATIVE);

        auto pre_enable_shoot_time_armor =
            armor_selector_.Run(pre_enable_shoot_time_target.armors, now_target.v_autoroatation < 0,
                                ArmorSelector::Method::MIN_OFFSET_RELATIVE);

        auto pre_disable_shoot_time_armor =
            armor_selector_.Run(pre_disable_shoot_time_target.armors, now_target.v_autoroatation < 0,
                                ArmorSelector::Method::MIN_OFFSET_RELATIVE);

        auto asymmetry_angle_offset = now_target.v_autoroatation *
                                      (tra_to_now_target->fly_time + param_.gimbal_yaw_machine_time_ms / 1000. +
                                       param_.enable_shoot_machine_time_ms / 1000.) *
                                      param_.enable_shoot_relative_angle_asymmetry_coefficient;
        if (now_target.armors.size() == 4) {
            if (shoot_status_) {
                if (std::abs(pre_disable_shoot_time_armor.chassis_in_armor.yaw) >=
                    (pre_gimbal_time_target.armor_size == RM::ArmorSize::LARGE
                         ? param_.large_4_armor_rotate_enable_shoot_relative_angle
                         : param_.small_4_armor_rotate_enable_shoot_relative_angle))
                    shoot_status_ = false;
            } else {
                if (std::abs(pre_enable_shoot_time_armor.chassis_in_armor.yaw) <=
                    (pre_gimbal_time_target.armor_size == RM::ArmorSize::LARGE
                         ? param_.large_4_armor_rotate_enable_shoot_relative_angle - asymmetry_angle_offset
                         : param_.small_4_armor_rotate_enable_shoot_relative_angle - asymmetry_angle_offset))
                    shoot_status_ = true;
            }
        } else if (now_target.armors.size() == 3) {
            if (shoot_status_) {
                if (std::abs(pre_disable_shoot_time_armor.chassis_in_armor.yaw) >=
                    (param_.small_3_armor_rotate_enable_shoot_relative_angle ))
                    shoot_status_ = false;
            } else {
                if (std::abs(pre_enable_shoot_time_armor.chassis_in_armor.yaw) <=
                    (param_.small_4_armor_rotate_enable_shoot_relative_angle - asymmetry_angle_offset))
                    shoot_status_ = true;
            }
        } else {  // == 2
            if (shoot_status_) {
                if (std::abs(pre_disable_shoot_time_armor.chassis_in_armor.yaw) >=
                    (param_.large_2_armor_rotate_enable_shoot_relative_angle))
                    shoot_status_ = false;
            } else {
                if (std::abs(pre_enable_shoot_time_armor.chassis_in_armor.yaw) <=
                    (param_.large_2_armor_rotate_enable_shoot_relative_angle - asymmetry_angle_offset))
                    shoot_status_ = true;
            }
        }

        auto tra_to_predicted_armor =
            trajectory_computer_.GetSolution(pre_gimbal_time_armor.armor_in_chassis.p +
                                             (param_.advanced_transform ? g_precise_tf_offset : cv::Point3d(0, 0, 0)));
        TargetSolution sol{GetXOYYaw(pre_gimbal_time_armor.armor_in_chassis.p +
                                     (param_.advanced_transform ? g_precise_tf_offset : cv::Point3d(0, 0, 0))),
                           -tra_to_predicted_armor->angle, shoot_status_};
        return sol;

    } else {  // mode_ == Mode::SPEEDY_ROTATE
        auto tra_to_now_target = trajectory_computer_.GetSolution(now_target.target_in_chassis.p);
        auto pre_gimbal_time_target = now_target.PredictTime(std::chrono::milliseconds(
            static_cast<int>(tra_to_now_target->fly_time * 1000. + param_.gimbal_yaw_machine_time_ms)));

        auto pre_enable_shoot_time_target = now_target.PredictTime(std::chrono::milliseconds(
            static_cast<int>(tra_to_now_target->fly_time * 1000. + param_.enable_shoot_machine_time_ms)));

        auto pre_disable_shoot_time_target = now_target.PredictTime(std::chrono::milliseconds(
            static_cast<int>(tra_to_now_target->fly_time * 1000. + param_.disable_shoot_machine_time_ms)));

        auto pre_gimbal_time_armor = armor_selector_.Run(pre_gimbal_time_target.armors, now_target.v_autoroatation < 0,
                                                         ArmorSelector::Method::MIN_OFFSET_RELATIVE);

        auto pre_enable_shoot_time_armor =
            armor_selector_.Run(pre_enable_shoot_time_target.armors, now_target.v_autoroatation < 0,
                                ArmorSelector::Method::MIN_OFFSET_RELATIVE);

        auto pre_disable_shoot_time_armor =
            armor_selector_.Run(pre_disable_shoot_time_target.armors, now_target.v_autoroatation < 0,
                                ArmorSelector::Method::MIN_OFFSET_RELATIVE);

        if (now_target.armors.size() == 4) {
            if (shoot_status_) {
                if (std::abs(pre_disable_shoot_time_armor.chassis_in_armor.yaw) >=
                    (pre_gimbal_time_target.armor_size == RM::ArmorSize::LARGE
                         ? param_.large_4_armor_speedy_rotate_enable_shoot_relative_angle
                         : param_.small_4_armor_speedy_rotate_enable_shoot_relative_angle))
                    shoot_status_ = false;
            } else {
                if (std::abs(pre_enable_shoot_time_armor.chassis_in_armor.yaw) <=
                    (pre_gimbal_time_target.armor_size == RM::ArmorSize::LARGE
                         ? param_.large_4_armor_speedy_rotate_enable_shoot_relative_angle
                         : param_.small_4_armor_speedy_rotate_enable_shoot_relative_angle))
                    shoot_status_ = true;
            }
        } else if (now_target.armors.size() == 3) {
            if (shoot_status_) {
                if (std::abs(pre_disable_shoot_time_armor.chassis_in_armor.yaw) >=
                    (param_.small_3_armor_speedy_rotate_enable_shoot_relative_angle))
                    shoot_status_ = false;
            } else {
                if (std::abs(pre_enable_shoot_time_armor.chassis_in_armor.yaw) <=
                    (param_.small_3_armor_speedy_rotate_enable_shoot_relative_angle))
                    shoot_status_ = true;
            }
        } else {  // == 2
            if (shoot_status_) {
                if (std::abs(pre_disable_shoot_time_armor.chassis_in_armor.yaw) >=
                    (param_.large_2_armor_speedy_rotate_enable_shoot_relative_angle))
                    shoot_status_ = false;
            } else {
                if (std::abs(pre_enable_shoot_time_armor.chassis_in_armor.yaw) <=
                    (param_.large_2_armor_speedy_rotate_enable_shoot_relative_angle))
                    shoot_status_ = true;
            }
        }

        auto tra_to_predicted_armor =
            trajectory_computer_.GetSolution(pre_gimbal_time_armor.armor_in_chassis.p +
                                             (param_.advanced_transform ? g_precise_tf_offset : cv::Point3d(0, 0, 0)));
        TargetSolution sol{GetXOYYaw(pre_gimbal_time_target.target_in_chassis.p +
                                     (param_.advanced_transform ? g_precise_tf_offset : cv::Point3d(0, 0, 0))),
                           -tra_to_predicted_armor->angle, shoot_status_};
        return sol;
    }
}

Mode ModeSwitcher::Run(Mode last_mode, const std::shared_ptr<MT::RVTargetInfo>& rv_target) {
    const double& enable_distance = param_.enable_close_mode_distance_threshold;
    const double& disable_distance = param_.disable_close_mode_distance_threshold;
    const double& enable_v_yaw = param_.enable_rotate_mode_v_yaw_threshold;
    const double& disable_v_yaw = param_.disable_rotate_mode_v_yaw_threshold;
    const double& speedy_enable_v_yaw = param_.enable_speedy_rotate_mode_v_yaw_threshold;
    const double& speedy_disable_v_yaw = param_.disable_speedy_rotate_mode_v_yaw_threshold;

    double distance = GetDistance(rv_target->c_position.x, rv_target->c_position.y, rv_target->c_position.z);

    if (last_mode == Mode::NORMAL) {
        if (distance <= enable_distance) {
            return Mode::CLOSE;
        } else if (fabs(rv_target->v_yaw) >= disable_v_yaw) {
            return Mode::ROTATE;
        } else {
            return Mode::NORMAL;
        }
    } else if (last_mode == Mode::ROTATE) {
        if (fabs(rv_target->v_yaw) >= speedy_enable_v_yaw) {
            return Mode::SPEEDY_ROTATE;
        }
        if (distance <= enable_distance && fabs(rv_target->v_yaw) <= disable_v_yaw) {
            return Mode::CLOSE;
        } else if (distance >= enable_distance && fabs(rv_target->v_yaw) <= disable_v_yaw) {
            return Mode::NORMAL;
        } else {
            return Mode::ROTATE;
        }
    } else if (last_mode == Mode::CLOSE) {
        if (distance >= disable_distance && fabs(rv_target->v_yaw) >= enable_v_yaw) {
            return Mode::ROTATE;
        } else if (distance >= disable_distance && fabs(rv_target->v_yaw) <= enable_v_yaw) {
            return Mode::NORMAL;
        } else {
            return Mode::CLOSE;
        }
    } else {  // last_mode == Mode::SPEEDY_ROTATE
        if (fabs(rv_target->v_yaw) <= speedy_disable_v_yaw) {
            return Mode::ROTATE;
        } else {
            return Mode::SPEEDY_ROTATE;
        }
    }
}