#include <memory>
#include <rclcpp/utilities.hpp>
#include <task_manager/task_manager_node.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TM::TaskManagerNode>(rclcpp::NodeOptions().use_intra_process_comms(true));
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}