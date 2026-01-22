#include <angles/angles.h>
#include <mcu_node/mcu.h>
#include <mcu_node/mcu_node.h>
#include <mcu_node/rm_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <interfaces/msg/mcucolor.hpp>
#include <interfaces/msg/mcusenddata.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <param_loader.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mcunode::McuNode>(rclcpp::NodeOptions().use_intra_process_comms(true));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}