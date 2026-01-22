#include <estimate_node/kinematics_solver/kinematics_solver.h>
#include <estimate_node/tracker_pub_node.h>
#include <vofa_bridge/vofa_bridge.h>
#include "estimate_node/interface/rm_interface.h"
AutoaimResultPublisher::AutoaimResultPublisher(const rclcpp::NodeOptions& options)
    : Node("autoaim_result_publisher", options) {
    mcu_senddata_publisher_ = this->create_publisher<interfaces::msg::Mcusenddata>("/autoaimresult", 1);
}
void AutoaimResultPublisher::Setmcudata_(const MCU::AutoaimSendInfo& mcunodereceiveinfo) {
    mcu_senddata_.aim_pitch = mcunodereceiveinfo.aim_pitch;
    mcu_senddata_.aim_yaw = mcunodereceiveinfo.aim_yaw;
    mcu_senddata_.target = static_cast<int>(mcunodereceiveinfo.target);
    mcu_senddata_.distance = mcunodereceiveinfo.distance;
    mcu_senddata_.enable_lock = mcunodereceiveinfo.enable_lock;
    mcu_senddata_.enable_shoot = mcunodereceiveinfo.enable_shoot;
    mcu_senddata_.kalman_score = mcunodereceiveinfo.kalman_score;
    // //RCLCPP_INFO(this->get_logger(), "aim yaw = %f, aim pitch = %f", mcu_senddata_.aim_yaw, mcu_senddata_.aim_pitch);
    //  auto& v = vpie::VofaBridge::Get();
    //  v.SendOnce(mcu_senddata_.aim_yaw, mcu_senddata_.aim_pitch);
}

void AutoaimResultPublisher::publish() {
    interfaces::msg::Mcusenddata mcudata{};
    // //RCLCPP_INFO(this->get_logger(), "publishing mcusendinfo");
    if (has_target_flag) {
        mcu_senddata_publisher_->publish(mcu_senddata_);
    } else {
        //RCLCPP_INFO(this->get_logger(), "NO mcusendinfo");
        mcu_senddata_publisher_->publish(GetEmptyMcuData());
    }
}

interfaces::msg::Mcusenddata AutoaimResultPublisher::GetEmptyMcuData() {
    interfaces::msg::Mcusenddata mcudata;
    mcudata.aim_pitch = 0;
    mcudata.aim_yaw = 0;
    mcudata.target = 5;
    mcudata.enable_shoot = false;
    mcudata.enable_lock = false;
    mcudata.distance = 0;
    return mcudata;
}