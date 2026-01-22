#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_mapper.hpp>
#include <detect_node/sub_img_node.h>
#include <sensor_sync_node/sensor_sync_node.hpp>
#include <camera_node/camera_node.hpp>
#include <thread>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto detect_node = std::make_shared<detector::Subscriber>(rclcpp::NodeOptions().use_intra_process_comms(true));
    auto sensor_sync_node = std::make_shared<sensorsync::SensorSyncNode>(rclcpp::NodeOptions().use_intra_process_comms(true));
    auto camera_node = std::make_shared<gxcamera::CameraNode>(rclcpp::NodeOptions().use_intra_process_comms(true));

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(camera_node);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    executor.add_node(sensor_sync_node);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    executor.add_node(detect_node);
    
    

    executor.spin();
    return 0;
}