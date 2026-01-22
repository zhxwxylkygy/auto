//
// Created by wpie on 23-11-1.
//

#include "mcu_node/mcu.h"
#include <angles/angles.h>
#include <calculate/basic_calculate.h>
#include <vofa_bridge/vofa_bridge.h>
#include <chrono>
#include <cstring>
#include <exception>
#include <iostream>
#include "thread"


using namespace std::chrono_literals;
MCU::MCU(COMMethod input_protocol, COMMode input_mode, std::chrono::microseconds sleep_us) {
    com_method_ = input_protocol;
    com_mode_ = input_mode;
    sleep_us_ = sleep_us;
}

void MCU::Run() {
    if (com_mode_ == COMMode::FLOW) {
    } else if (com_mode_ == COMMode::ORDER) {
    }

    if (com_method_ == COMMethod::SERIAL) {
        i_communication_ = std::make_unique<Serial>("/dev/ttyACM0");
    }

    is_running = true;
    std::function f = [this]() {
        int last_switch_order = -1;
        int last_reset_order = -1;
        int drop_out_count = 0;
        int autoaim_living_waterfall_flag = 0;
        while (true) {
            {
                std::unique_lock lock(this->thread_server_.mutex_);
                if (this->thread_server_.need_exit_)
                    break;
                char target;
                switch (this->thread_server_.autoaim_send_.target) {
                    case RM::TargetClassify::BASE_DART:
                        target = 0;
                        break;
                    case RM::TargetClassify::BASE_BOTTOM:
                        target = 1;
                        break;
                    case RM::TargetClassify::BASE_TOP:
                        target = 2;
                        break;
                    case RM::TargetClassify::OUTPOST_DART:
                        target = 3;
                        break;
                    case RM::TargetClassify::OUTPOST_SPIN:
                        target = 4;
                        break;
                    case RM::TargetClassify::HERO_1:
                        target = 5;
                        break;
                    case RM::TargetClassify::ENGINEER_2:
                        target = 6;
                        break;
                    case RM::TargetClassify::STANDARD_3:
                        target = 7;
                        break;
                    case RM::TargetClassify::STANDARD_4:
                        target = 8;
                        break;
                    case RM::TargetClassify::STANDARD_5:
                        target = 9;
                        break;
                    case RM::TargetClassify::SENTRY:
                        target = 10;
                        break;
                    case RM::TargetClassify::POWER_RUNE:
                        target = 11;
                        break;
                }
                // std::cout << thread_server_.sensor_acquisition_flag << std::endl;
                SendMessage send_msg{
                    .sensor_acquisition_flag = static_cast<char>(thread_server_.sensor_acquisition_flag),
                    .aim_yaw = static_cast<float>(angles::to_degrees(this->thread_server_.autoaim_send_.aim_yaw)),
                    .aim_pitch = static_cast<float>(angles::to_degrees(this->thread_server_.autoaim_send_.aim_pitch)),
                    .distance = static_cast<float>(this->thread_server_.autoaim_send_.distance),
                    .target = target,
                    .enable_lock = this->thread_server_.autoaim_send_.enable_lock,
                    .enable_shoot = this->thread_server_.autoaim_send_.enable_shoot,
                    .autoaim_living_waterfall_flag = autoaim_living_waterfall_flag,
                    .kalman_score = static_cast<char>(this->thread_server_.autoaim_send_.kalman_score),
                    .hero_lidar_solve_yaw = static_cast<float>(
                        angles::to_degrees(this->thread_server_.hero_lidar_solve_send_.hero_lidar_solve_gimbal_yaw)),
                    .hero_lidar_solve_pitch = static_cast<float>(
                        angles::to_degrees(this->thread_server_.hero_lidar_solve_send_.hero_lidar_solve_gimbal_pitch))};

                autoaim_living_waterfall_flag++;
                static int sensor_acquisition_flag_last = thread_server_.sensor_acquisition_flag;

                sensor_acquisition_flag_last = thread_server_.sensor_acquisition_flag;

                Protocol::Send(*i_communication_, send_msg);
                lock.unlock();

                std::this_thread::sleep_for(std::chrono::microseconds(this->sleep_us_));

                lock.lock();

                auto o = Protocol::Recv<RecvMessage>(*i_communication_);

                
                if (o.has_value()) {
                    thread_server_.successful_receive = true;
                    auto data = o.value();
                    this->thread_server_.autoaim_recv_.enemy_hp.hero_1 = data.hero_1;
                    this->thread_server_.autoaim_recv_.enemy_hp.engineer_2 = data.engineer_2;
                    this->thread_server_.autoaim_recv_.enemy_hp.standard_3 = data.standard_3;
                    this->thread_server_.autoaim_recv_.enemy_hp.standard_4 = data.standard_4;
                    this->thread_server_.autoaim_recv_.enemy_hp.standard_5 = data.standard_5;
                    this->thread_server_.autoaim_recv_.enemy_hp.out_pose = data.out_pose;
                    this->thread_server_.autoaim_recv_.enemy_hp.base = data.base;
                    this->thread_server_.autoaim_recv_.enemy_hp.sentry = data.sentry;

                    if (data.order / 10000 == 1) {
                        this->thread_server_.autoaim_recv_.special_order =
                            AutoaimReceiveInfo::SpecialOrder::USE_SENTRY_BACKUP_MODULE;
                    } else if (data.order / 10000 == 2) {
                        this->thread_server_.autoaim_recv_.special_order =
                            AutoaimReceiveInfo::SpecialOrder::USE_SECONDARY_CAMERA;
                    } else {
                        this->thread_server_.autoaim_recv_.special_order = AutoaimReceiveInfo::SpecialOrder::NONE;
                    }
                    //std::cout << data.order << std::endl;
                    if (!(data.order % 10000 / 1000 == 4 || data.order % 10000 / 1000 == 6)) {
                        std::cerr << "MCU: wrong receive msg type" << std::endl;
                        std::terminate();
                    }
                    int this_reset_order = data.order % 10000 / 1000;
                    if (last_reset_order == -1) {
                        last_reset_order = this_reset_order;
                    }
                    if (this_reset_order != last_reset_order) {
                        this->thread_server_.autoaim_recv_.switch_mode = AutoaimReceiveInfo::SwitchMode::RESET;
                    }
                    last_reset_order = this_reset_order;

                    switch (data.order % 10000 % 1000 % 100 % 10) {
                        case 0:
                            this->thread_server_.autoaim_recv_.target_type = AutoaimReceiveInfo::TargetType::BUILDING;
                            break;
                        case 1:
                            this->thread_server_.autoaim_recv_.target_type = AutoaimReceiveInfo::TargetType::ROBOT;
                            break;
                        case 2:
                            this->thread_server_.autoaim_recv_.target_type = AutoaimReceiveInfo::TargetType::POWER_RUNE;
                            break;
                        case 3:
                            this->thread_server_.autoaim_recv_.target_type =
                                AutoaimReceiveInfo::TargetType::BUILDING_AND_ROBOT;
                            break;
                        default:
                            std::cout << "MCU: wrong receive msg order type" << std::endl;
                            std::terminate();
                    }
                    this->thread_server_.autoaim_recv_.target_type = AutoaimReceiveInfo::TargetType::POWER_RUNE;
                    bool left_switch = false;
                    bool right_switch = false;
                    int this_switch_order = data.order % 10000 % 1000 / 100;

                    //    std::cout << this_switch_order << std::endl;

                    if (last_switch_order == -1)
                        last_switch_order = this_switch_order;
                    if (this_switch_order != last_switch_order) {
                        if (last_switch_order == 0 || last_switch_order == 9) {
                            if (last_switch_order == 0 && this_switch_order == 9)
                                left_switch = true;
                            if (last_switch_order == 0 && this_switch_order == 1)
                                right_switch = true;
                            if (last_switch_order == 9 && this_switch_order == 8)
                                left_switch = true;
                            if (last_switch_order == 9 && this_switch_order == 0)
                                right_switch = true;
                        } else {
                            if (this_switch_order == last_switch_order + 1)
                                right_switch = true;
                            if (this_switch_order == last_switch_order - 1)
                                left_switch = true;
                        }
                        last_switch_order = this_switch_order;
                    }

                    if (thread_server_.autoaim_recv_.switch_mode == AutoaimReceiveInfo::SwitchMode::NONE) {
                        thread_server_.autoaim_recv_.switch_mode = AutoaimReceiveInfo::SwitchMode::NONE;
                        if (left_switch && !right_switch)
                            thread_server_.autoaim_recv_.switch_mode = AutoaimReceiveInfo::SwitchMode::LEFT;
                        if (!left_switch && right_switch)
                            thread_server_.autoaim_recv_.switch_mode = AutoaimReceiveInfo::SwitchMode::RIGHT;
                        if (left_switch && right_switch) {
                            std::cout << "MCU: wrong switch mode " << std::endl;
                            std::terminate();
                        }
                    } else {
                        if (thread_server_.get_info_flag) {
                            thread_server_.autoaim_recv_.switch_mode = AutoaimReceiveInfo::SwitchMode::NONE;
                            if (left_switch && !right_switch)
                                thread_server_.autoaim_recv_.switch_mode = AutoaimReceiveInfo::SwitchMode::LEFT;
                            if (!left_switch && right_switch)
                                thread_server_.autoaim_recv_.switch_mode = AutoaimReceiveInfo::SwitchMode::RIGHT;
                            if (left_switch && right_switch) {
                                std::cout << "MCU: wrong switch mode " << std::endl;
                                std::terminate();
                            }
                        }
                    }

                    switch (data.order % 10000 % 1000 % 100 / 10) {
                        case 0:
                            this->thread_server_.autoaim_recv_.target_priority_rule =
                                AutoaimReceiveInfo::TargetPriorityRule::ONLY_HERO;
                            break;
                        case 1:
                            this->thread_server_.autoaim_recv_.target_priority_rule =
                                AutoaimReceiveInfo::TargetPriorityRule::AUTO;
                            break;
                        case 2:
                            this->thread_server_.autoaim_recv_.target_priority_rule =
                                AutoaimReceiveInfo::TargetPriorityRule::MANUAL;
                            break;
                        default:
                            std::cout << "MCU: wrong receive msg order type" << std::endl;
                            assert(false);
                    }

                    if (data.color == 21) {
                        this->thread_server_.autoaim_recv_.color = RM::Color::RED;
                        // std::cout << "red" << std::endl;
                    } else if (data.color == 53) {
                        this->thread_server_.autoaim_recv_.color = RM::Color::BLUE;
                        // std::cout << "blue" << std::endl;
                    } else {
                        this->thread_server_.autoaim_recv_.color = RM::Color::NONE;
                        // std::cout << "none" << std::endl;
                    }

                    thread_server_.autoaim_recv_.imu_yaw = angles::from_degrees(data.imu_yaw);
                    thread_server_.autoaim_recv_.imu_pitch = angles::from_degrees(data.imu_pitch);
                    thread_server_.autoaim_recv_.imu_roll = angles::from_degrees(data.imu_roll);
                    thread_server_.autoaim_recv_.yaw_motor = angles::from_degrees(data.yaw_motor);
                    thread_server_.autoaim_recv_.pitch_motor = angles::from_degrees(data.pitch_motor);
                    //std::cout << "  " <<  data.imu_yaw << "  " << data.imu_pitch << "  " << data.yaw_motor << "  " << data.pitch_motor << std::endl;

                    thread_server_.autoaim_recv_.remaining_time_s = data.remaining_time_s;
                    thread_server_.autoaim_recv_.remaining_time_s = 420;
                    thread_server_.autoaim_recv_.system_time_point_ms = data.system_time_point_ms;
                    // thread_server_.cv_.notify_all();

                    thread_server_.hero_lidar_solve_recv_.hero_lidar_distance = data.hero_lidar_distance;
                    thread_server_.hero_lidar_solve_recv_.hero_gimbal_yaw = angles::from_degrees(data.imu_yaw);
                    thread_server_.hero_lidar_solve_recv_.hero_gimbal_pitch = angles::from_degrees(data.imu_pitch);
                } else {
                    thread_server_.successful_receive = false;
                    std::cout << "MCU: drop out!" << std::endl;
                    drop_out_count++;
                    if (drop_out_count >= 1000)
                        std::terminate();
                }

                thread_server_.get_info_flag = false;

                lock.unlock();
                canpublish = true;
                std::this_thread::sleep_for(std::chrono::microseconds(this->sleep_us_));
            }
        }
        this->thread_server_.finished_exit_cv.notify_all();
        std::cout << "MCU: thread normal exit" << std::endl;
    };
    std::thread t(f);
    t.detach();

    //ActivateSensor();
    std::this_thread::sleep_for(100ms);
    std::cout << "MCU: start running" << std::endl;
}

std::optional<MCU::AutoaimReceiveInfo> MCU::GetAutoaimReceiveInfo() {
    if (!is_running) {
        std::cout << "MCU: can't get receive info, function haven't start" << std::endl;
        std::terminate();
    }
    {
        std::unique_lock lock(thread_server_.mutex_);
        if (thread_server_.successful_receive) {
            thread_server_.get_info_flag = true;
            return thread_server_.autoaim_recv_;
        } else
            return std::nullopt;
    }
}
// std::optional<MCU::AutoaimReceiveInfo> MCU::WaitAutoaimReceiveInfo() {
//     if (!is_running) {
//         std::cout << "MCU: can't get receive info, function haven't start" << std::endl;
//         std::terminate();
//     }
//     {
//         std::unique_lock lock(thread_server_.mutex_);
//         auto status = thread_server_.cv_.wait_for(lock, std::chrono::milliseconds(10));
//         if (status == std::cv_status::no_timeout) {
//             return thread_server_.autoaim_recv_;
//         } else {
//             return std::nullopt;
//         }
//     }
// }

// void MCU::WaitAutoaimSendInfo() {
//     {
//         std::unique_lock lock(thread_server_.mutex_);
//         auto status = thread_server_.cv_.wait_for(lock, std::chrono::milliseconds(10));
//         if (status != std::cv_status::no_timeout) {
//             std::terminate();
//         }
//     }
// }

void MCU::SetAutoaimSendInfo(const MCU::AutoaimSendInfo& input) {
    if (!is_running) {
        std::cout << "MCU: can't set send info, function haven't start" << std::endl;
        std::terminate();
    }
    {
        std::unique_lock lock(thread_server_.mutex_);
        thread_server_.autoaim_send_ = input;
    }
}
void MCU::SetHeroLidarSolveSendInfo(const MCU::HeroLidarSolveSendInfo& input) {
    if (!is_running) {
        std::cout << "MCU: can't set send info, function haven't start" << std::endl;
        std::terminate();
    }
    {
        std::unique_lock lock(thread_server_.mutex_);
        thread_server_.hero_lidar_solve_send_ = input;
    }
}
std::optional<MCU::HeroLidarSolveReceiveInfo> MCU::GetHeroLidarSolveReceiveInfo() {
    if (!is_running) {
        std::cout << "MCU: can't get receive info, function haven't start" << std::endl;
        std::terminate();
    }
    {
        std::unique_lock lock(thread_server_.mutex_);
        if (thread_server_.successful_receive) {
            // thread_server_.get_info_flag = true;
            return thread_server_.hero_lidar_solve_recv_;
        } else
            return std::nullopt;
    }
}
void MCU::Stop() {
    if (!is_running) {
        std::cout << "MCU: can't stop, function haven't start" << std::endl;
        std::terminate();
    }
    {
        std::unique_lock lock(thread_server_.mutex_);
        thread_server_.need_exit_ = true;
        thread_server_.finished_exit_cv.wait(lock);
        thread_server_.need_exit_ = false;
    }
    is_running = false;
}
void MCU::ActivateSensor() {
    if (!is_running) {
        std::cout << "MCU: can't activate sensor, function haven't start" << std::endl;
        std::terminate();
    }
    {
        std::unique_lock lock(thread_server_.mutex_);
        if (thread_server_.sensor_acquisition_flag == 0) {
            // std::cout << "filp a_flag" << std::endl;
            thread_server_.sensor_acquisition_flag = 9;
        } else if (thread_server_.sensor_acquisition_flag == 9) {
            thread_server_.sensor_acquisition_flag = 0;
            // std::cout << "filp a_flag" << std::endl;
        } else {
            std::cout << "MCU: wrong sensor acquisition flag" << std::endl;
            std::terminate();
        }
    }
}
std::optional<MCU::AutoaimReceiveInfo::SpecialOrder> MCU::GetSpecialOrder() {
    if (!is_running) {
        std::cout << "MCU: can't get special order, function haven't start" << std::endl;
        std::terminate();
    }
    {
        std::unique_lock lock(thread_server_.mutex_);
        if (thread_server_.successful_receive) {
            // thread_server_.get_info_flag = true;
            return thread_server_.autoaim_recv_.special_order;
        } else
            return std::nullopt;
    }
}

std::unique_ptr<MCU::AutoaimSendInfo> MCU::GetSendInfo() {
    return std::move(this->mcunodereceiveinfo_);
}

std::unique_ptr<MCU::AutoaimReceiveInfo> MCU::GetReceiveInfo() {
    return std::move(this->mcunodesendinfo_);
}

void MCU::SetReceiveInfo(std::unique_ptr<MCU::AutoaimReceiveInfo> receiveinfo) {
    std::memcpy(this->mcunodereceiveinfo_.get(), receiveinfo.get(), sizeof(MCU::AutoaimReceiveInfo));
}

void MCU::SetSendInfo(std::unique_ptr<MCU::AutoaimSendInfo> sendinfo) {
    std::memcpy(this->mcunodesendinfo_.get(), sendinfo.get(), sizeof(MCU::AutoaimSendInfo));
}

bool MCU::CanPublish() {
    if (this->canpublish) {
        this->CanNotPublish();
        return true;
    }
    return false;
}

void MCU::CanNotPublish() {
    this->canpublish = false;
}
