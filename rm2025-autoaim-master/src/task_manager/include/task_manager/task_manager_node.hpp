#include <interfaces/msg/detail/order__struct.hpp>
#include <interfaces/msg/order.hpp>
#include <interfaces/srv/activatesensor.hpp>
#include <interfaces/srv/color.hpp>
#include <interfaces/srv/detail/activatesensor__struct.hpp>
#include <interfaces/srv/detail/color__struct.hpp>
#include <interfaces/srv/order.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <std_msgs/msg/string.hpp>
#include <task_manager/order.hpp>
#include <task_manager/order_interface.hpp>
#include <task_manager/auto_delete.hpp>
namespace TM {
class TaskManagerNode : rclcpp::Node {
   public:
    explicit TaskManagerNode(const rclcpp::NodeOptions& options);

    std::shared_ptr<rclcpp::Node> SharedFromThis();

    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

   private:
    // mcu order relative
    rclcpp::Subscription<interfaces::msg::Order>::SharedPtr order_subscription_;
    void McuOrderListenerCallback(const interfaces::msg::Order::SharedPtr order_msg);

    void ActivateSensorCallback(const std::shared_ptr<interfaces::srv::Activatesensor::Request> request,
                                std::shared_ptr<interfaces::srv::Activatesensor::Response> response);

    // distribute order
    rclcpp::Service<interfaces::srv::Activatesensor>::SharedPtr activate_sensor_server_;
    rclcpp::Client<interfaces::srv::Activatesensor>::SharedPtr activate_client_;

    rclcpp::Publisher<interfaces::msg::Order>::SharedPtr order_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void PublishOrderTimely();

    rclcpp::Service<interfaces::srv::Color>::SharedPtr color_sevrer_;
    void ColorRequestCallback(const std::shared_ptr<interfaces::srv::Color::Request> request,
                              std::shared_ptr<interfaces::srv::Color::Response> response);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_sync_heart_beat_;

    std::unique_ptr<ad::AutoDelete> ad_;
};
}  // namespace TM
