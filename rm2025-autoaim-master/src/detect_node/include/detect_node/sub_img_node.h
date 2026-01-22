#ifndef SUBIMG
#define SUBIMG
#include <detect_node/frame_processer/frame_processor.h>
#include <detect_node/interface/mcu.h>
#include <detect_node/interface/rm_interface.h>
#include <builtin_interfaces/msg/detail/time__struct.hpp>
#include <condition_variable>
#include <exception>
#include <image_transport/image_transport.hpp>
#include <interfaces/msg/detail/mcucolor__struct.hpp>
#include <interfaces/msg/detail/order__struct.hpp>
#include <interfaces/msg/feature.hpp>
#include <interfaces/msg/mcucolor.hpp>
#include <interfaces/msg/order.hpp>
#include <interfaces/srv/color.hpp>
#include <interfaces/srv/detail/activatesensor__struct.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <param_loader.hpp>
#include <queue>
#include <rclcpp/client.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>
#include <vector>
#include "detect_node/interface/interface.h"
#include <interfaces/srv/cameratimestamp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
namespace detector {

class Subscriber : public rclcpp::Node {
   public:
    explicit Subscriber(const rclcpp::NodeOptions& options);

   private:
    // std::unique_ptr<MCU::AutoaimReceiveInfo> mcudata_ = std::make_unique<MCU::AutoaimReceiveInfo>();
    // img relative
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgs_subscription_;
    void PublishResultImage(const std_msgs::msg::Header& header, const cv::Mat& res);
    image_transport::Publisher camera_pub_{};
    void ImgListenerCallback(const std::shared_ptr<const sensor_msgs::msg::Image>& msg);

    // task relative
    void OrderListenerCallback(const interfaces::msg::Order::SharedPtr msg);
    rclcpp::Subscription<interfaces::msg::Order>::SharedPtr order_subscription_;

    MCU::Orders orders_{};

    // activate sensor relative
    rclcpp::Client<interfaces::srv::Activatesensor>::SharedPtr activate_client_;
    void ActivateResultCallback(rclcpp::Client<interfaces::srv::Activatesensor>::SharedFuture result_future);

   private:
    // feature relative
    void SetFeature(std::vector<std::shared_ptr<FP::ArmorInfo>> armor_features);
    void SetFeature(std::vector<std::shared_ptr<FP::FanInfo>> fan_feature);
    void SetTarget(bool isfan);
    void SetFeatureHeader(std_msgs::msg::Header header);
    void Publish();
    void SetFeatureToPublish(const std::vector<std::shared_ptr<FP::IFeature>>& features);

    rclcpp::Publisher<interfaces::msg::Feature>::SharedPtr feature_publisher_;
    std::shared_ptr<interfaces::msg::Feature> feature_ = std::make_shared<interfaces::msg::Feature>();
        
    std::string current_id_;
    rclcpp::Duration camera_awake_time_;
    enum class HeaderState{
        HEADER_BUFFER_START_WORKING,
        HEADER_BUFFER_UNAVAIBLE,
    };
    HeaderState hd_ = HeaderState::HEADER_BUFFER_UNAVAIBLE;


   private:
    bool has_secondary_camera_ =
        !ParamLoader::GetInstance().GetParam<std::string>("CAMERA", "SECONDARY_CAMERA_SN").empty();
    // detect relative
    std::vector<std::shared_ptr<FP::FanInfo>> fan_featere_{};
    std::vector<std::shared_ptr<FP::ArmorInfo>> armor_features_{};
    std::unique_ptr<FP::FrameProcessor> frame_processor_;
    image_transport::Subscriber sub_;

   private:
    // color relative
    rclcpp::Client<interfaces::srv::Color>::SharedPtr color_client_;
    void ColorResultCallback(rclcpp::Client<interfaces::srv::Color>::SharedFuture result_future);

   private:
    // Debug relative
    struct Params {
        const bool is_time_log;
        const int cv_threads;
    };
    Params params_{.is_time_log = ParamLoader::GetInstance().GetParam<bool>("DETECT", "DEBUG"),
                   .cv_threads = ParamLoader::GetInstance().GetParam<int>("DETECT", "CV_THREAD_NUMS")};

    void Logging();
    struct TimeLog {
        using Tp = std::chrono::steady_clock::time_point;
        Tp node_start;
        Tp pub_time;
        Tp node_end;

        static Tp GetTimePoint() { return std::chrono::steady_clock::now(); }
    };
    TimeLog tl;

    private:
    struct ThreadSever{
    std::shared_ptr<const sensor_msgs::msg::Image> img_msg;
        std::mutex mtx;
        std::condition_variable cv;
    };
    ThreadSever thread_sever_;

    void ImageProcessTask();

    //debug
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr time_log_publisher_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detect_heart_beat_;

    rclcpp::TimerBase::SharedPtr timer_;

};


geometry_msgs::msg::Point cvPointToGeometryPoint(const cv::Point& cv_point);
geometry_msgs::msg::Point cvPointToGeometryPoint(const cv::Point3d& cv_point);

}  // namespace detector
#endif