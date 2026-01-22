#include <chrono>
#include <interfaces/msg/detail/order__struct.hpp>
#include <interfaces/srv/detail/activatesensor__struct.hpp>
#include <interfaces/srv/detail/color__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <task_manager/task_manager_node.hpp>
#include <thread>
#include "task_manager/order_interface.hpp"

namespace TM {
TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions& options) : Node("node", options) {
    rclcpp::Clock sys_clock(RCL_SYSTEM_TIME);
    order_subscription_ = this->create_subscription<interfaces::msg::Order>(
        "/mcu/order", rclcpp::SensorDataQoS(),
        [this](const interfaces::msg::Order::SharedPtr order_msg) { McuOrderListenerCallback(order_msg); });

    activate_sensor_server_ = this->create_service<interfaces::srv::Activatesensor>(
        "/mamager/activate_server", [this](const std::shared_ptr<interfaces::srv::Activatesensor::Request> request,
                                           std::shared_ptr<interfaces::srv::Activatesensor::Response> response) {
            ActivateSensorCallback(request, response);
        });

    activate_client_ = this->create_client<interfaces::srv::Activatesensor>("/order2sync");
    while (!activate_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "waitting is interrupted ...");
            return;
        }
        //RCLCPP_INFO(this->get_logger(), "waitting ...");
    }
    order_publisher_ = this->create_publisher<interfaces::msg::Order>("/manager/order", rclcpp::SensorDataQoS());

    color_sevrer_ = this->create_service<interfaces::srv::Color>(
        "/manager/color", [this](const std::shared_ptr<interfaces::srv::Color::Request> request,
                                 std::shared_ptr<interfaces::srv::Color::Response> response) {
            ColorRequestCallback(request, response);
        });
    sensor_sync_heart_beat_ = this->create_publisher<std_msgs::msg::String>("/manager/heartbeat", rclcpp::SensorDataQoS());

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this] {
        PublishOrderTimely();
        std_msgs::msg::String s;
        s.data = "task_manager";
        this->sensor_sync_heart_beat_->publish(s);
    });

    ad_ = std::make_unique<ad::AutoDelete>("/root/master/program/RM2025-autoaim/record", 3);
}
void TaskManagerNode::McuOrderListenerCallback(const interfaces::msg::Order::SharedPtr order_msg) {
    Order::GetInstance()->UpdateOrders(order_msg);
    Order::GetInstance()->UpdateTime(order_msg);
}

void TaskManagerNode::PublishOrderTimely() {
    if (true) {  // Order::GetInstance()->IsStateChange()){
        auto order = Order::GetInstance()->GetOrderMessage();
        // RCLCPP_INFO_STREAM(this->get_logger(), "time :" << order.remaining_time_s);
        order_publisher_->publish(order);
        if(Order::GetInstance()->GetSyncStatus()){
            auto activate_request = std::make_shared<interfaces::srv::Activatesensor::Request>();
            activate_request->activate_sensor = interfaces::srv::Activatesensor::Request::CONTINUOUSTRIGGER;
            activate_client_->async_send_request(activate_request,
                                             [this](rclcpp::Client<interfaces::srv::Activatesensor>::SharedFuture future) {
                                                 Order::GetInstance()->SetSyncAlreadyState();
                                             });
        }
        
    } else {
        RCLCPP_DEBUG(this->get_logger(), "no message update");
    }
}

void TaskManagerNode::ActivateSensorCallback(const std::shared_ptr<interfaces::srv::Activatesensor::Request> request,
                                             std::shared_ptr<interfaces::srv::Activatesensor::Response> response) {
    auto activate_request = std::make_shared<interfaces::srv::Activatesensor::Request>(*request);
    activate_client_->async_send_request(activate_request,
                                         [this](rclcpp::Client<interfaces::srv::Activatesensor>::SharedFuture future) {
                                             Order::GetInstance()->SetSyncAlreadyState();
                                         });
}

void TaskManagerNode::ColorRequestCallback(const std::shared_ptr<interfaces::srv::Color::Request> request,
                                           std::shared_ptr<interfaces::srv::Color::Response> response) {
    //RCLCPP_INFO(this->get_logger(), "Get request");
    if (Order::GetInstance()->FetchColorOrder() != RM::Color::NONE) {
        response->color = static_cast<int>(Order::GetInstance()->FetchColorOrder());
        if (response->color == interfaces::srv::Color::Response::BLUE)
            RCLCPP_INFO_STREAM(this->get_logger(), "response color [[  BLUE  ]]");
        if (response->color == interfaces::srv::Color::Response::RED)
        RCLCPP_INFO_STREAM(this->get_logger(), "response color [[  RED  ]]");
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "color is not vaild " << response->color);
    }
}
std::shared_ptr<rclcpp::Node> TaskManagerNode::SharedFromThis() {
    return this->shared_from_this();
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> TaskManagerNode::get_node_base_interface() {
    return rclcpp::Node::get_node_base_interface();
}

}  // namespace TM
