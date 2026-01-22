//
// Created by wpie on 23-11-1.
//

#ifndef INC_1021_MCU_H
#define INC_1021_MCU_H
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mcu_node/rm_interface.h>
#include <memory>
#include "./serial/protocol.hpp"
#include "./serial/protocol_pack.h"
#include "./serial/serial.h"

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

    void Run();

    void Stop();

    void SetAutoaimSendInfo(const AutoaimSendInfo& input);

    //[[nodiscard]] std::optional<AutoaimReceiveInfo> WaitAutoaimReceiveInfo();

   [[nodiscard]] std::optional<MCU::AutoaimReceiveInfo> GetAutoaimReceiveInfo();

    //void WaitAutoaimSendInfo(); 

    void SetHeroLidarSolveSendInfo(const HeroLidarSolveSendInfo& input);

    [[nodiscard]] std::optional<HeroLidarSolveReceiveInfo> GetHeroLidarSolveReceiveInfo();

    [[nodiscard]] std::optional<AutoaimReceiveInfo::SpecialOrder> GetSpecialOrder();

    void ActivateSensor();

    [[nodiscard]] std::unique_ptr<AutoaimSendInfo> GetSendInfo();

    [[nodiscard]] std::unique_ptr<AutoaimReceiveInfo> GetReceiveInfo();

    void SetReceiveInfo(std::unique_ptr<AutoaimReceiveInfo> receiveinfo);
    
    void SetSendInfo(std::unique_ptr<AutoaimSendInfo> sendinfo);

    [[nodiscard]] bool CanPublish();

    void CanNotPublish();
    
   private:
    PROTOCOL SendMessage : IProtocolData {
        char sensor_acquisition_flag = 0;
        float aim_yaw = -1;
        float aim_pitch = -1;
        float distance = -1;
        char target = -1;
        uint8_t enable_lock = false;
        uint8_t enable_shoot = false;
        int autoaim_living_waterfall_flag{};
        char kalman_score = 0;
        float hero_lidar_solve_yaw = -1;
        float hero_lidar_solve_pitch = -1;
    };

    PROTOCOL RecvMessage : IProtocolData {
        float imu_yaw{};
        float imu_pitch{};
        float imu_roll{};
        float yaw_motor{};
        float pitch_motor{};
        unsigned char color{};
        short order{};
        short hero_1{};
        short engineer_2{};
        short standard_3{};
        short standard_4{};
        short standard_5{};
        short sentry{};
        short out_pose{};
        short base{};
        short remaining_time_s{};
        float hero_lidar_distance{};
        char sentry_behavior_mode{};
        uint64_t system_time_point_ms{};
    };

    struct ThreadSever{
        std::mutex mutex_;
        //std::condition_variable cv_;
        bool need_exit_{};
        AutoaimReceiveInfo autoaim_recv_{};
        AutoaimSendInfo autoaim_send_{};
        HeroLidarSolveReceiveInfo hero_lidar_solve_recv_{};
        HeroLidarSolveSendInfo hero_lidar_solve_send_{};
        int sensor_acquisition_flag = 0;
        std::condition_variable finished_exit_cv{};
        bool successful_receive = false;
        bool get_info_flag = false;
    } thread_server_{};

    std::chrono::microseconds sleep_us_{};

    COMMode com_mode_{};

    COMMethod com_method_{};

    bool is_running = false;

    bool canpublish = false;

    std::unique_ptr<ICommunication<64>> i_communication_ = nullptr;

    std::unique_ptr<AutoaimSendInfo> mcunodereceiveinfo_ = std::make_unique<AutoaimSendInfo>();

    std::unique_ptr<AutoaimReceiveInfo> mcunodesendinfo_ = std::make_unique<AutoaimReceiveInfo>();

  
};
#endif  // INC_1021_MCU_H
