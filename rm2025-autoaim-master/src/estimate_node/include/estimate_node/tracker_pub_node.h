#pragma once
#include <estimate_node/interface/mcu.h>
#include <estimate_node/interface/rm_interface.h>
#include <interfaces/msg/detail/mcucolor__struct.hpp>
#include <interfaces/msg/feature.hpp>
#include <interfaces/msg/mcucolor.hpp>
#include <interfaces/msg/mcusenddata.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class AutoaimResultPublisher : public rclcpp::Node {
   public:
    explicit AutoaimResultPublisher(const rclcpp::NodeOptions& options);
    void Setmcudata_(const MCU::AutoaimSendInfo& info);
    bool has_target_flag = false;
    void publish();

   private:
    [[nodiscard]] interfaces::msg::Mcusenddata GetEmptyMcuData();
    rclcpp::Publisher<interfaces::msg::Mcusenddata>::SharedPtr mcu_senddata_publisher_;
    interfaces::msg::Mcusenddata mcu_senddata_{};
};