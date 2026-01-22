#include <rmw/types.h>
#include <camera_node/camera_node.hpp>
#include <chrono>
#include <rclcpp/clock.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <std_msgs/msg/detail/int64__struct.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>

using namespace gxcamera;
CameraNode::CameraNode(const rclcpp::NodeOptions& options) : Node("Gxcamera", options) {
    rclcpp::Clock sys_clock(RCL_SYSTEM_TIME);
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera node is initializing......");
    if (!ParamLoader::GetInstance().GetParam<std::string>("CAMERA", "MAIN_CAMERA_SN").empty())
        gx_camera_.LoadCameraSN(ParamLoader::GetInstance().GetParam<std::string>("CAMERA", "MAIN_CAMERA_SN").c_str());
    gx_camera_.LoadRoiParam(ParamLoader::GetInstance().GetParam<cv::Size>("CAMERA", "IMAGE_SIZE").width,
                            ParamLoader::GetInstance().GetParam<cv::Size>("CAMERA", "IMAGE_SIZE").height, 0, 0);
    gx_camera_.LoadExposureGainParam(false, false, ParamLoader::GetInstance().GetParam<int>("CAMERA", "EXPOSURE_TIME"),
                                     800, 8000, ParamLoader::GetInstance().GetParam<int>("CAMERA", "GAIN"), 0, 16, 127);
    gx_camera_.LoadWhiteBalanceParam(true, GX_AWB_LAMP_HOUSE_ADAPTIVE);
    if (gx_camera_.InitWithParam() != 0)
        std::terminate();
    if (gx_camera_.Run(frame_data_, /*std::chrono::milliseconds(8))*/ GxCamera::ExternalTriggerIndex::GPIO_LINE2,
                       GxCamera::ExternalTriggerActivation::RAISING_EDGE) != 0)
        std::terminate();
    const auto intrinsics_mm = ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA", "INTRINSICS_MM");
    const auto distortion_coefficient =
        ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA", "DISTORTION_COEFFICIENT");
    this->camera_info_.header.frame_id = "gimbal";
    this->camera_info_.distortion_model = "plumb_bob";
    this->camera_info_.height = ParamLoader::GetInstance().GetParam<cv::Size>("CAMERA", "IMAGE_SIZE").height;
    this->camera_info_.width = ParamLoader::GetInstance().GetParam<cv::Size>("CAMERA", "IMAGE_SIZE").width;
    // std::memcpy(this->camera_info_.d.data(), distortion_coefficient.data,
    //             distortion_coefficient.total() * distortion_coefficient.elemSize());
    // std::memcpy(this->camera_info_.k.data(), intrinsics_mm.data, intrinsics_mm.total() * intrinsics_mm.elemSize());
    this->img_data_.header.frame_id = "gimbal";
    this->img_data_.set__encoding("rgb8");
    this->img_data_.set__is_bigendian(false);
    this->img_data_.set__step(0);
    // auto qos = rmw_qos_profile_default;
    // qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    // qos.depth = 1;
    // qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    // qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
    .keep_last(1);


    camera_pub_ = image_transport::create_publisher(this, "/camera/image", qos.get_rmw_qos_profile());

    header_pub_ = this->create_publisher<std_msgs::msg::Header>("/camera/headerdata", rclcpp::SensorDataQoS());

    camera_waterfall_ = this->create_publisher<std_msgs::msg::Int64>("/camera/waterfall", rclcpp::SensorDataQoS());

    
    std::thread(&CameraNode::Run, this).detach();
    
    timer_ = this->create_wall_timer(std::chrono::seconds(3), [this]() {
        std_msgs::msg::Int64 id;
        static long num{0};
        id.data = num;
        camera_waterfall_->publish(id);
        RCLCPP_INFO_STREAM(this->get_logger(), "camera awake " << num);
        num++;
    });

    RCLCPP_INFO_STREAM(this->get_logger(), "Camera node was initialized");
}

void CameraNode::Run() {
    //std::cout << "first run" << std::chrono::high_resolution_clock::now().time_since_epoch().count() << std::endl;
    //cv::startWindowThread();
    while (rclcpp::ok()) {
        sensor_msgs::msg::Image img_msg;
        if (gx_camera_.WaitForImgmsg(img_msg, camera_info_, frame_data_, std::chrono::milliseconds(100000000000))) {
            RCLCPP_ERROR(this->get_logger(), "Frame wait out of time");
            break;
        }
        static int n = 1;
        camera_pub_.publish(img_msg);
        header_pub_->publish(img_msg.header);
        if(n == 1){
            RCLCPP_INFO_STREAM(this->get_logger(), "publish " << img_msg.header.stamp.sec<< "." << img_msg.header.stamp.nanosec);
            RCLCPP_INFO_STREAM(this->get_logger(), "publish " << this->now().nanoseconds());
        }
            
        n++;
        //this->camera_heart_beat_->publish(s);
    }
    gx_camera_.Stop();
    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gxcamera::CameraNode)
