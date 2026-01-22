//
// Created by wpie on 23-11-1.
//

#ifndef INC_1021_MCU_H
#define INC_1021_MCU_H
#include <condition_variable>
#include <cstdint>
#include <detect_node/interface/interface.h>


class MCU {
   public:
    enum class COMMethod { SERIAL, USB_HID, BLUE_TOOTH };
    enum class COMMode { FLOW, ORDER };

   public:
    struct AutoaimSendInfo {
        double aim_yaw = 1;
        double aim_pitch = 2;
        double distance = 3;
        RM::TargetClassify target = static_cast<RM::TargetClassify>(4);
        bool enable_lock = false;
        bool enable_shoot = false;
        int kalman_score = 0;
    };

    struct AutoaimReceiveInfo {
        enum class SwitchMode { NONE, LEFT, RIGHT, RESET } switch_mode{};
        enum class TargetType { BUILDING, ROBOT, POWER_RUNE, BUILDING_AND_ROBOT } target_type{};
        enum class TargetPriorityRule { ONLY_HERO, AUTO, MANUAL } target_priority_rule{};
        enum class SentryBehaviorMode { NOT_SENTRY, ATTACK, DEFEND } sentry_behavior_mode{};
        enum class SpecialOrder{NONE, USE_SENTRY_BACKUP_MODULE, USE_SECONDARY_CAMERA}special_order{};
        struct EnemyHP {
            int hero_1{};
            int engineer_2{};
            int standard_3{};
            int standard_4{};
            int standard_5{};
            int sentry{};
            int out_pose{};
            int base{};
        } enemy_hp{};
        double imu_yaw{};
        double imu_pitch{};
        double imu_roll{};
        double yaw_motor{};
        double pitch_motor{};
        RM::Color color{};
        int remaining_time_s{};
        uint64_t system_time_point_ms{}; // to define  
    };


    struct HeroLidarSolveSendInfo {
        double hero_lidar_solve_gimbal_yaw{};
        double hero_lidar_solve_gimbal_pitch{};
    };
    struct HeroLidarSolveReceiveInfo {
        double hero_lidar_distance{};
        double hero_gimbal_yaw{};
        double hero_gimbal_pitch{};
    };

   public:
    MCU() = delete;

    MCU(COMMethod input_protocol,
        COMMode input_mode,
        std::chrono::microseconds sleep_us = std::chrono::milliseconds(3));

    struct Orders {
        MCU::AutoaimReceiveInfo::SwitchMode switch_mode;
        MCU::AutoaimReceiveInfo::TargetType target_type;
        MCU::AutoaimReceiveInfo::TargetPriorityRule target_priority_rule;
        MCU::AutoaimReceiveInfo::SentryBehaviorMode sentry_behavior_mode;
        MCU::AutoaimReceiveInfo::SpecialOrder special_order;
        RM::Color color;
    };


};
#endif  // INC_1021_MCU_H
