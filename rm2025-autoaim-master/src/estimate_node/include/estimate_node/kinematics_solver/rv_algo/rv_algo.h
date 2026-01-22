//
// Created by wpie on 23-11-2.
//

#pragma once
#include <chrono>
#include <vector>
#include "angles/angles.h"
#include "calculate/trajectory_calculate.h"
#include "estimate_node/interface/i_algorithm.h"
#include "estimate_node/interface/interface.h"
#include "estimate_node/interface/rm_interface.h"
#include "param_loader.hpp"

namespace KS::RVA {

enum class Mode { CLOSE, NORMAL, ROTATE, SPEEDY_ROTATE };

struct Target {
    struct Armor {
        struct ArmorInTarget {
            double yaw;
        };
        struct ArmorInChassis {
            cv::Point3d p;
        };
        struct ChassisInArmor {
            double yaw;
        };

        ArmorInTarget armor_in_target{};
        ArmorInChassis armor_in_chassis{};
        ChassisInArmor chassis_in_armor{};
    };

    struct TargetInChassis {
        cv::Point3d p;
        cv::Point3d v;
    };

    TargetInChassis target_in_chassis{};

    std::vector<Armor> armors{};

    double v_autoroatation{};
    double armor_r1{};
    double armor_r2{};
    double d_z{};
    RM::ArmorSize armor_size{};

    void CopyConstantInfoTo(Target& t) const {
        t.armor_r1 = armor_r1;
        t.armor_r2 = armor_r2;
        t.d_z = d_z;
        t.v_autoroatation = v_autoroatation;
        t.target_in_chassis.v = target_in_chassis.v;
        t.armors.resize(armors.size());
        t.armor_size = armor_size;
    }

    [[nodiscard]] Target CloneConstantInfo() const {
        Target t;
        this->CopyConstantInfoTo(t);
        return t;
    }

    Target PredictTime(std::chrono::milliseconds duration_time_ms);
};

class ArmorSelector {
   public:
    enum class Method { MIN_DISTANCE, MIN_OFFSET_RELATIVE, MIN_RELATIVE };

    [[nodiscard]] Target::Armor Run(const std::vector<Target::Armor>& armors, bool is_clockwise_rotate, Method method);

   private:
    struct Param {
        // const double armor4_relative_angle_max_limit = angles::from_degrees(
        //     ParamLoader::GetInstance().GetParam<double>("KS","RV_ALGO.ARMOR4_RELATIVE_ANGLE_MAX_LIMIT_DEGREE"));
        // const double armor4_window_phase_angle_max_limit = angles::from_degrees(
        //     ParamLoader::GetInstance().GetParam<double>("KS","RV_ALGO.ARMOR4_WINDOW_PHASE_ANGLE_MAX_LIMIT_DEGREE"));
        // const double armor4_matching_max_yaw = angles::from_degrees(
        //     ParamLoader::GetInstance().GetParam<double>("KS","RV_ALGO.ARMOR4_MATCHING_MAX_YAW_DEGREE"));

        // const double armor3_relative_angle_max_limit = angles::from_degrees(
        //     ParamLoader::GetInstance().GetParam<double>("KS","RV_ALGO.ARMOR3_RELATIVE_ANGLE_MAX_LIMIT_DEGREE"));
        // const double armor3_window_phase_angle_max_limit = angles::from_degrees(
        //     ParamLoader::GetInstance().GetParam<double>("KS","RV_ALGO.ARMOR3_WINDOW_PHASE_ANGLE_MAX_LIMIT_DEGREE"));
        // const double armor3_matching_max_yaw = angles::from_degrees(
        //     ParamLoader::GetInstance().GetParam<double>("KS","RV_ALGO.ARMOR3_MATCHING_MAX_YAW_DEGREE"));

        // const double armor2_relative_angle_max_limit = angles::from_degrees(
        //     ParamLoader::GetInstance().GetParam<double>("KS","RV_ALGO.ARMOR2_RELATIVE_ANGLE_MAX_LIMIT_DEGREE"));
        // const double armor2_window_phase_angle_max_limit = angles::from_degrees(
        //     ParamLoader::GetInstance().GetParam<double>("KS","RV_ALGO.ARMOR2_WINDOW_PHASE_ANGLE_MAX_LIMIT_DEGREE"));
        // const double armor2_matching_max_yaw = angles::from_degrees(
        //     ParamLoader::GetInstance().GetParam<double>("KS","RV_ALGO.ARMOR2_MATCHING_MAX_YAW_DEGREE"));

        const double switch_offset_angle = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("KS", "RV_ALGO", "SWITCH_OFFSET_ANGLE_DEGREE"));

    } param_;
};

class ModeSwitcher {
   public:
    Mode Run(Mode last_mode, const std::shared_ptr<MT::RVTargetInfo>& rv_target);

   private:
    struct Param {
        const double enable_close_mode_distance_threshold =
            ParamLoader::GetInstance().GetParam<double>("KS", "RV_ALGO", "ENABLE_CLOSE_MODE_DISTANCE_THRESHOLD");
        const double disable_close_mode_distance_threshold =
            ParamLoader::GetInstance().GetParam<double>("KS", "RV_ALGO", "DISABLE_CLOSE_MODE_DISTANCE_THRESHOLD");
        const double enable_rotate_mode_v_yaw_threshold = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("KS", "RV_ALGO", "ENABLE_ROTATE_MODE_V_YAW_THRESHOLD_DEGREE"));
        const double disable_rotate_mode_v_yaw_threshold = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("KS", "RV_ALGO", "DISABLE_ROTATE_MODE_V_YAW_THRESHOLD_DEGREE"));
        const double enable_speedy_rotate_mode_v_yaw_threshold = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("KS",
                                                        "RV_ALGO",
                                                        "ENABLE_SPEEDY_ROTATE_MODE_V_YAW_THRESHOLD_DEGREE"));
        const double disable_speedy_rotate_mode_v_yaw_threshold = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("KS",
                                                        "RV_ALGO",
                                                        "DISABLE_SPEEDY_ROTATE_MODE_V_YAW_THRESHOLD_DEGREE"));

    } param_;
};

class RVAlgo : public IAlgorithm {
   public:
    RVAlgo() = delete;

    explicit RVAlgo(Trajectory& input);

    [[nodiscard]] TargetSolution Run(const std::shared_ptr<MT::ITargetInfo>& input_target,
                                     cv::Point3d g_precise_tf_offset) override;

   private:
    Target LoadTargetInfo(const std::shared_ptr<MT::RVTargetInfo>& input_target);

    Target GetTargetAfterTime(const Target& now_target_info, std::chrono::milliseconds time_ms);

    Mode mode_{};

    bool shoot_status_ = false;

    struct Param {
        const double bullet_speed = ParamLoader::GetInstance().GetParam<double>("KS", "BULLET_SPEED");
        const double gimbal_yaw_machine_time_ms =
            ParamLoader::GetInstance().GetParam<double>("KS", "GIMBAL_YAW_MACHINE_TIME_MS");
        const double gimbal_pitch_machine_time_ms =
            ParamLoader::GetInstance().GetParam<double>("KS", "GIMBAL_PITCH_MACHINE_TIME_MS");
        const double enable_shoot_machine_time_ms =
            ParamLoader::GetInstance().GetParam<double>("KS", "ENABLE_SHOOT_MACHINE_TIME_MS");
        const double disable_shoot_machine_time_ms =
            ParamLoader::GetInstance().GetParam<double>("KS", "DISABLE_SHOOT_MACHINE_TIME_MS");

        const RM::BulletType bullet_type =
            static_cast<RM::BulletType>(ParamLoader::GetInstance().GetParam<int>("KS", "BULLET_TYPE"));

        // const double small_4_armor_speedy_rotate_enable_shoot_relative_angle = angles::from_degrees(18);
        // const double small_4_armor_rotate_enable_shoot_relative_angle = angles::from_degrees(22);
        // const double large_4_armor_speedy_rotate_enable_shoot_relative_angle = angles::from_degrees(28);
        // const double large_4_armor_rotate_enable_shoot_relative_angle = angles::from_degrees(40);

        // const double small_3_armor_speedy_rotate_enable_shoot_relative_angle = angles::from_degrees(18);
        // const double small_3_armor_rotate_enable_shoot_relative_angle = angles::from_degrees(30);

        // const double large_2_armor_speedy_rotate_enable_shoot_relative_angle = angles::from_degrees(28);
        // const double large_2_armor_rotate_enable_shoot_relative_angle = angles::from_degrees(60);

        const double small_4_armor_speedy_rotate_enable_shoot_relative_angle =
            angles::from_degrees(ParamLoader::GetInstance().GetParam<double>(
                "KS",
                "RV_ALGO",
                "SMALL_4_ARMOR_SPEEDY_ROTATE_ENABLE_SHOOT_RELATIVE_ANGLE_DEGREE"));
        const double small_4_armor_rotate_enable_shoot_relative_angle = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("KS",
                                                        "RV_ALGO",
                                                        "SMALL_4_ARMOR_ROTATE_ENABLE_SHOOT_RELATIVE_ANGLE_DEGREE"));

        const double large_4_armor_speedy_rotate_enable_shoot_relative_angle =
            angles::from_degrees(ParamLoader::GetInstance().GetParam<double>(
                "KS",
                "RV_ALGO",
                "LARGE_4_ARMOR_SPEEDY_ROTATE_ENABLE_SHOOT_RELATIVE_ANGLE_DEGREE"));
        const double large_4_armor_rotate_enable_shoot_relative_angle = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("KS",
                                                        "RV_ALGO",
                                                        "LARGE_4_ARMOR_ROTATE_ENABLE_SHOOT_RELATIVE_ANGLE_DEGREE"));

        const double small_3_armor_speedy_rotate_enable_shoot_relative_angle =
            angles::from_degrees(ParamLoader::GetInstance().GetParam<double>(
                "KS",
                "RV_ALGO",
                "SMALL_3_ARMOR_SPEEDY_ROTATE_ENABLE_SHOOT_RELATIVE_ANGLE_DEGREE"));
        const double small_3_armor_rotate_enable_shoot_relative_angle = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("KS",
                                                        "RV_ALGO",
                                                        "SMALL_3_ARMOR_ROTATE_ENABLE_SHOOT_RELATIVE_ANGLE_DEGREE"));

        const double large_2_armor_speedy_rotate_enable_shoot_relative_angle =
            angles::from_degrees(ParamLoader::GetInstance().GetParam<double>(
                "KS",
                "RV_ALGO",
                "LARGE_2_ARMOR_SPEEDY_ROTATE_ENABLE_SHOOT_RELATIVE_ANGLE_DEGREE"));
        const double large_2_armor_rotate_enable_shoot_relative_angle = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("KS",
                                                        "RV_ALGO",
                                                        "LARGE_2_ARMOR_ROTATE_ENABLE_SHOOT_RELATIVE_ANGLE_DEGREE"));

        const bool advanced_transform = ParamLoader::GetInstance().GetParam<bool>("ROBOT", "ADVANCED_TRANSFORM");

        const bool normalized_outpost_angular_velocity =
            ParamLoader::GetInstance().GetParam<bool>("KS", "RV_ALGO", "NORMALIZED_OUTPOST_ANGULAR_VELOCITY");

        const double  enable_shoot_relative_angle_asymmetry_coefficient = 
            ParamLoader::GetInstance().GetParam<double>("KS", "RV_ALGO", "ENABLE_SHOOT_RELATIVE_ANGLE_ASYMMETRY_COEFFICIENT");

    } param_;

    ArmorSelector armor_selector_{};

    ModeSwitcher mode_switcher_{};
};
}  // namespace KS::RVA
