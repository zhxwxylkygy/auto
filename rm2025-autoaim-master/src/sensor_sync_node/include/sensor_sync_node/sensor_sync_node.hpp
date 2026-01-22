#pragma once
#include <builtin_interfaces/msg/detail/time__struct.hpp>
#include <cstdint>
#include <image_transport/image_transport.hpp>
#include <interfaces/msg/angle.hpp>
#include <interfaces/msg/detail/angle__struct.hpp>
#include <interfaces/msg/detail/order__struct.hpp>
#include <interfaces/srv/activatesensor.hpp>
#include <interfaces/srv/cameratimestamp.hpp>
#include <interfaces/srv/detail/activatesensor__struct.hpp>
#include <interfaces/srv/detail/cameratimestamp__struct.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <std_msgs/msg/detail/int64__struct.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int64.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
namespace sensorsync {

class SensorSyncNode : public rclcpp::Node {
   public:
    explicit SensorSyncNode(const rclcpp::NodeOptions& options);
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();
    ~SensorSyncNode(){
        auto stop_trigger = [this](){
            this->stop_signal.set_value();
        };
        stop_trigger();
        activate_thread.join();
    }
   private:
    // trigger camera relative
    rclcpp::Service<interfaces::srv::Activatesensor>::SharedPtr activate_server_;  // from order_node

    rclcpp::Client<interfaces::srv::Activatesensor>::SharedPtr activate_client_;  // to mcu
    void ActivateSignalCallback(const std::shared_ptr<interfaces::srv::Activatesensor::Request> request,
                                std::shared_ptr<interfaces::srv::Activatesensor::Response> response);

    // camera relative
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr camera_status_subscription_;
    void ImgSyncListenerCallback(const std::shared_ptr<sensor_msgs::msg::Image> image_msg);
    void CameraAwakeCallback(const std::shared_ptr<std_msgs::msg::Int64> image_msg);
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr activate_order_;
    sensor_msgs::msg::Image img_data_{};
    sensor_msgs::msg::CameraInfo camera_info_{};
    image_transport::Publisher camera_pub_{};
    long count{0};
    const std_msgs::msg::String kOrder;

    // time point relative
    rclcpp::Time first_time_activate_camera_stamp_in_world_{};
    rclcpp::Time first_time_activate_camera_stamp_in_camera_{};
    rclcpp::Duration camera_awake_time_;

    void CalculateAwakeTime();
    builtin_interfaces::msg::Time SyncImageTimeStamp(const rclcpp::Time& time);

    // camera info relative
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    void CameraInfoListenerCallback(const std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg);
    // motor angle relative
    rclcpp::Subscription<interfaces::msg::Angle>::SharedPtr motor_angle_subscription_;
    void MotorAngleListenerCallback(const std::shared_ptr<interfaces::msg::Angle> motor_angle_msg);
    // imu relative
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    void ImuListenerCallback(const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg);

    enum class WORK_STATUS {
        SLEEP = interfaces::msg::Order::SYNC_SLEEP,
        START = interfaces::msg::Order::SYNC_START,
        WAITING = interfaces::msg::Order::SYNC_WAIT,
        WORKING = interfaces::msg::Order::SYNC_WORKING,
    };

    WORK_STATUS status = WORK_STATUS::SLEEP;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_sync_heart_beat_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::thread activate_thread;
    std::promise<void> stop_signal;
    std::shared_future<void> stop_future = stop_signal.get_future();


};

}  // namespace sensorsync
