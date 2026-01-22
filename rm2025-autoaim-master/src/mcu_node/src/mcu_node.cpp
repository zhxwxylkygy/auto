#include <angles/angles.h>
#include <mcu_node/mcu.h>
#include <mcu_node/mcu_node.h>
#include <mcu_node/rm_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vofa_bridge/vofa_bridge.h>
#include <chrono>
#include <exception>
#include <interfaces/msg/detail/order__struct.hpp>
#include <interfaces/msg/mcucolor.hpp>
#include <interfaces/msg/mcusenddata.hpp>
#include <interfaces/srv/detail/activatesensor__struct.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <param_loader.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include "rclcpp/rclcpp.hpp"
using namespace mcunode;

McuNode::McuNode(const rclcpp::NodeOptions& options) : Node("Mcunode", options) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Mcu node is Initializing ...");

    //rclcpp::Clock sys_clock(RCL_SYSTEM_TIME);

    mcu->Run();

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    order_publisher_ = this->create_publisher<interfaces::msg::Order>("/mcu/order", qos);

    motor_angle_publisher_ = this->create_publisher<interfaces::msg::Angle>("/mcu/angle", qos);

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/mcu/imu_msg", qos);

    mcusenddata_subscription_ = this->create_subscription<interfaces::msg::Mcusenddata>(
        "/autoaimresult", 1,
        [this](const interfaces::msg::Mcusenddata::SharedPtr senddata) { McuSenddataListenerCallback(senddata); });

    activate_order_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/sensor_manager/activate_order", rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::String::SharedPtr order) { ActivateOrderCallback(order); });


    activate_server_ = this->create_service<interfaces::srv::Activatesensor>(
        "/mcu/activate_camera", [this](const std::shared_ptr<interfaces::srv::Activatesensor::Request> request,
                             const std::shared_ptr<interfaces::srv::Activatesensor::Response> response) {
            ActivateRequestCallback(request, response);
        });
    std::thread(&McuNode::Publish, this).detach();

    mcu_heart_beat_ = this->create_publisher<std_msgs::msg::String>("/mcu/heartbeat", rclcpp::SensorDataQoS());

    // timer_ = this->create_wall_timer(std::chrono::seconds(3), [this]() {
    //     std_msgs::msg::String s;
    //     s.data = "mcu_node";
    //     this->mcu_heart_beat_->publish(s);
    // });

    RCLCPP_INFO_STREAM(this->get_logger(), "Mcu node was successfully Initialized  ");
}

void McuNode::MessageAdapter(MCU::AutoaimReceiveInfo mcunodereceiveinfo) {
    SetOrder(mcunodereceiveinfo);
    SetImuData(mcunodereceiveinfo);
    SetMotorAngle(mcunodereceiveinfo);
}

void McuNode::McuSenddataListenerCallback(const interfaces::msg::Mcusenddata::SharedPtr senddata) {
    auto d = *mcusenddata_;
    mcusenddata_->aim_pitch = senddata->aim_pitch;
    mcusenddata_->aim_yaw = senddata->aim_yaw;
    mcusenddata_->distance = senddata->distance;
    mcusenddata_->enable_lock = senddata->enable_lock;
    mcusenddata_->enable_shoot = senddata->enable_shoot;
    mcusenddata_->kalman_score = senddata->kalman_score;
    mcusenddata_->target = static_cast<RM::TargetClassify>(senddata->target);
    mcu->SetAutoaimSendInfo(*mcusenddata_);
}

void McuNode::ActivateOrderCallback(const std_msgs::msg::String::SharedPtr order){
    this->mcu->ActivateSensor();
    //RCLCPP_INFO_STREAM(this->get_logger(), "activate" << order->data);
}

void McuNode::Publish() {
    while (rclcpp::ok()) {
        // auto time = std::chrono::system_clock::now();
        auto mcureceiveinfo = mcu->GetAutoaimReceiveInfo();
        if (mcureceiveinfo.has_value()) {
            MessageAdapter(mcureceiveinfo.value());
            order_publisher_->publish(order_info_);
            motor_angle_publisher_->publish(motor_angle_);
            imu_publisher_->publish(imu_info_);
            std_msgs::msg::String s;
            s.data = "mcu_node";
            this->mcu_heart_beat_->publish(s);
        } else {
            RCLCPP_ERROR_STREAM(this->get_logger(), "mcureceiveinfo no value");
        }
    }
}

void McuNode::SetImuData(MCU::AutoaimReceiveInfo mcunodereceiveinfo) {
    this->imu_info_.header.frame_id = "chassis";
    this->imu_info_.header.stamp = rclcpp::Clock().now();
    this->imu_info_.header.stamp.nanosec -= mcudelay_offset_nanosecond_;
    // //RCLCPP_INFO(this->get_logger(), "yaw %f", mcunodereceiveinfo.imu_yaw);
    tf2::Quaternion q;
    q.setRPY(mcunodereceiveinfo.imu_roll, mcunodereceiveinfo.imu_pitch, mcunodereceiveinfo.imu_yaw);
    this->imu_info_.orientation.w = q.w();
    this->imu_info_.orientation.x = q.x();
    this->imu_info_.orientation.y = q.y();
    this->imu_info_.orientation.z = q.z();
}

void McuNode::SetMotorAngle(MCU::AutoaimReceiveInfo mcunodereceiveinfo) {
    this->motor_angle_.header.frame_id = "chassis";
    this->motor_angle_.header.stamp = this->now();
    this->motor_angle_.header.stamp.nanosec -= mcudelay_offset_nanosecond_;
    this->motor_angle_.yaw_motor = mcunodereceiveinfo.yaw_motor;
    this->motor_angle_.pitch_motor = mcunodereceiveinfo.pitch_motor;
}

void McuNode::SetOrder(MCU::AutoaimReceiveInfo mcunodereceiveinfo) {
    this->order_info_.switch_mode = static_cast<int>(mcunodereceiveinfo.switch_mode);
    this->order_info_.target_type = static_cast<int>(mcunodereceiveinfo.target_type);
    this->order_info_.target_priority_rule = static_cast<int>(mcunodereceiveinfo.target_priority_rule);
    this->order_info_.sentry_behavior_mode = static_cast<int>(mcunodereceiveinfo.sentry_behavior_mode);
    this->order_info_.special_order = static_cast<int>(mcunodereceiveinfo.special_order);
    this->order_info_.hero_1 = mcunodereceiveinfo.enemy_hp.hero_1;
    this->order_info_.engineer_2 = mcunodereceiveinfo.enemy_hp.engineer_2;
    this->order_info_.standard_3 = mcunodereceiveinfo.enemy_hp.standard_3;
    this->order_info_.standard_4 = mcunodereceiveinfo.enemy_hp.standard_4;
    this->order_info_.standard_5 = mcunodereceiveinfo.enemy_hp.standard_5;
    this->order_info_.sentry = mcunodereceiveinfo.enemy_hp.sentry;
    this->order_info_.out_pose = mcunodereceiveinfo.enemy_hp.out_pose;
    this->order_info_.base = mcunodereceiveinfo.enemy_hp.base;
    this->order_info_.color = static_cast<int>(mcunodereceiveinfo.color);
    this->order_info_.remaining_time_s = mcunodereceiveinfo.remaining_time_s;
    this->order_info_.system_time_point_ms = mcunodereceiveinfo.system_time_point_ms;
}

void McuNode::ActivateRequestCallback(const std::shared_ptr<interfaces::srv::Activatesensor::Request> request,
                                      const std::shared_ptr<interfaces::srv::Activatesensor::Response> response) {
    if (request->activate_sensor == interfaces::srv::Activatesensor::Request::CONTINUOUSTRIGGER) {
        // static bool is_need_trigger = true;
        // // RCLCPP_INFO(this->get_logger(), "Start Activate sensor");
        // if(is_need_trigger){
        //     std::thread([this]() {
        //     while (rclcpp::ok()) {
        //         this->mcu->ActivateSensor();
        //         static int num = 1;
        //         std::this_thread::sleep_for(std::chrono::milliseconds(11));
        //     }
        //     }).detach();
        //     is_need_trigger = false;
        // }
        
    } else if (request->activate_sensor == interfaces::srv::Activatesensor::Request::SYNCSENSOR) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        this->mcu->ActivateSensor();
        response->header.stamp = this->now();
        auto t = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        RCLCPP_INFO_STREAM(this->get_logger(), "Start Sync sensor Time is : " << t << "ros time" <<response->header.stamp.sec << response->header.stamp.nanosec);
        response->header.frame_id = "first_time_activate";
    } 
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mcunode::McuNode)