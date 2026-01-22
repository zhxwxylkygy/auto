#include <angles/angles.h>
#include <mcu_node/mcu.h>
#include <mcu_node/rm_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <condition_variable>
#include <interfaces/msg/angle.hpp>
#include <interfaces/msg/detail/order__struct.hpp>
#include <interfaces/msg/feature.hpp>
#include <interfaces/msg/mcucolor.hpp>
#include <interfaces/msg/mcusenddata.hpp>
#include <interfaces/msg/order.hpp>
#include <interfaces/srv/activatesensor.hpp>
#include <interfaces/srv/color.hpp>
#include <interfaces/srv/detail/activatesensor__struct.hpp>
#include <memory>
#include <optional>
#include <param_loader.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
namespace mcunode {

class McuNode : public rclcpp::Node {
   public:
    explicit McuNode(const rclcpp::NodeOptions& options);

   private:
    std::shared_ptr<MCU> mcu = std::make_shared<MCU>(MCU::COMMethod::SERIAL, MCU::COMMode::FLOW, 1us);
    const uint32_t mcudelay_offset_nanosecond_ = 2000000;

   private:
    void Publish();
    void MessageAdapter(MCU::AutoaimReceiveInfo mcunodereceiveinfo);

   private:
    rclcpp::Publisher<interfaces::msg::Order>::SharedPtr order_publisher_;
    void SetOrder(MCU::AutoaimReceiveInfo mcunodereceiveinfo);
    interfaces::msg::Order order_info_{};

   private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    void SetImuData(MCU::AutoaimReceiveInfo mcunodereceiveinfo);
    sensor_msgs::msg::Imu imu_info_{};

   private:
    rclcpp::Publisher<interfaces::msg::Angle>::SharedPtr motor_angle_publisher_;
    void SetMotorAngle(MCU::AutoaimReceiveInfo mcunodereceiveinfo);
    interfaces::msg::Angle motor_angle_{};

   private:
    rclcpp::Subscription<interfaces::msg::Mcusenddata>::SharedPtr mcusenddata_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr activate_order_subscription_;
    void McuSenddataListenerCallback(const interfaces::msg::Mcusenddata::SharedPtr senddata);
    void ActivateOrderCallback(const std_msgs::msg::String::SharedPtr order);
    std::shared_ptr<MCU::AutoaimSendInfo> mcusenddata_ = std::make_shared<MCU::AutoaimSendInfo>();

   private:
    rclcpp::Service<interfaces::srv::Activatesensor>::SharedPtr activate_server_;
    void ActivateRequestCallback(const std::shared_ptr<interfaces::srv::Activatesensor::Request> request,
                                 const std::shared_ptr<interfaces::srv::Activatesensor::Response> response);
    rclcpp::Subscription<interfaces::msg::Feature>::SharedPtr feature_subscription_;
    void FeatureListenerCallback(const interfaces::msg::Feature msg);
    // private:
    //     rclcpp::Service<interfaces::srv::Color>::SharedPtr color_server_;
    //     void ColorRequestCallback(const std::shared_ptr<interfaces::srv::Color::Request> request,
    //                                  std::shared_ptr<interfaces::srv::Color::Response> response);
    //     int color_{};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mcu_heart_beat_;

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mcunode