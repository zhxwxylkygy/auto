#include <chrono>
#include <image_transport/camera_publisher.hpp>
#include <memory>
#include <param_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <camera_node/gx_camera.h>

namespace gxcamera {


class CameraNode : public rclcpp::Node {
   public:
    explicit CameraNode(const rclcpp::NodeOptions& options);

   private:
    void Run();

    GxCamera::FrameData frame_data_;
    GxCamera gx_camera_;
    sensor_msgs::msg::Image img_data_{};
    sensor_msgs::msg::CameraInfo camera_info_{};
    image_transport::Publisher camera_pub_{};
    
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr camera_waterfall_;

    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace gxcamera