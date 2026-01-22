#include "detect_node/sub_img_node.h"
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <algorithm>
#include <builtin_interfaces/msg/detail/time__struct.hpp>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <interfaces/msg/detail/armor__struct.hpp>
#include <interfaces/msg/detail/order__struct.hpp>
#include <interfaces/srv/detail/activatesensor__struct.hpp>
#include <interfaces/srv/detail/color__struct.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "detect_node/interface/interface.h"
#include "detect_node/interface/rm_interface.h"

using namespace detector;

void set_thread_priority(int priority) {
    pthread_t thread = pthread_self();  // 获取当前线程
    struct sched_param sched;
    sched.sched_priority = priority;                    // 设置线程优先级
    pthread_setschedparam(thread, SCHED_FIFO, &sched);  // 使用 FIFO 调度策略
}

Subscriber::Subscriber(const rclcpp::NodeOptions& options)
    : Node("imgsub", options), camera_awake_time_(rclcpp::Duration::from_seconds((0.114514))) {
    rclcpp::Clock sys_clock(RCL_SYSTEM_TIME);
    //RCLCPP_INFO(this->get_logger(), "[Sub] Initializing Subscriber ......");
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
    .keep_last(1);

    feature_publisher_ = this->create_publisher<interfaces::msg::Feature>("/feature", qos);
    feature_->fan.keypoints.emplace_back(cvPointToGeometryPoint(cv::Point(0.0)));
    feature_->fan.keypoints.emplace_back(cvPointToGeometryPoint(cv::Point(0.0)));
    feature_->fan.keypoints.emplace_back(cvPointToGeometryPoint(cv::Point(0.0)));
    feature_->fan.keypoints.emplace_back(cvPointToGeometryPoint(cv::Point(0.0)));
    feature_->header.frame_id = "gimbal";
    feature_->header.stamp.nanosec = 0.f;
    feature_->header.stamp.sec = 0.f;
    interfaces::msg::Armor armor;
    armor.armor_id = static_cast<int>(1);
    armor.armor_size = static_cast<int>(0);
    // armor.camera_coord_pose =  cvPointToGeometryPoint(feature->geographic_coord_pose);
    armor.keypoints.emplace_back(cvPointToGeometryPoint(cv::Point(0, 0)));
    armor.keypoints.emplace_back(cvPointToGeometryPoint(cv::Point(0, 0)));
    armor.keypoints.emplace_back(cvPointToGeometryPoint(cv::Point(0, 0)));
    armor.keypoints.emplace_back(cvPointToGeometryPoint(cv::Point(0, 0)));
    armor.is_dart_armor = false;
    this->feature_->armors.emplace_back(armor);
    color_client_ = this->create_client<interfaces::srv::Color>("/manager/color");
    while (!color_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "waitting is interrupted ...");
            return;
        }
        RCLCPP_WARN_STREAM(this->get_logger(), "waitting  color service ...");
    }
    auto color_request = std::make_shared<interfaces::srv::Color_Request>();
    //RCLCPP_INFO(this->get_logger(), "Send color request");
    color_client_->async_send_request(color_request,
                                      std::bind(&Subscriber::ColorResultCallback, this, std::placeholders::_1));

    // activate_client_ = this->create_client<interfaces::srv::Activatesensor>("/mamager/activate_server");
    // while (!activate_client_->wait_for_service(std::chrono::seconds(1))) {
    //     if (!rclcpp::ok()) {
    //         RCLCPP_ERROR_STREAM(this->get_logger(), "waitting is interrupted ...");
    //         return;
    //     }
    //     RCLCPP_DEBUG_STREAM(this->get_logger(), "waitting activate service ...");
    // }

    imgs_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/sync/image", rclcpp::SensorDataQoS(),
        std::bind(&Subscriber::ImgListenerCallback, this, std::placeholders::_1));

    order_subscription_ = this->create_subscription<interfaces::msg::Order>(
        "/manager/order", rclcpp::SensorDataQoS(),
        std::bind(&Subscriber::OrderListenerCallback, this, std::placeholders::_1));

    camera_pub_ = image_transport::create_publisher(this, "/camera/result");

    cv::startWindowThread();

    detect_heart_beat_ = this->create_publisher<std_msgs::msg::String>("/detect/heartbeat", rclcpp::SensorDataQoS());

    // timer_ = this->create_wall_timer(std::chrono::seconds(3), [this]() {
        
    // });

    // std::this_thread::sleep_for(std::chrono::seconds(1));

    // auto activate_request = std::make_shared<interfaces::srv::Activatesensor::Request>();
    // activate_request->activate_sensor = interfaces::srv::Activatesensor::Request::SYNCSENSOR;
    // activate_client_->async_send_request(activate_request,
    //                                      std::bind(&Subscriber::ActivateResultCallback, this, std::placeholders::_1));

    time_log_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/loop_cost", rclcpp::SensorDataQoS());

    cv::setNumThreads(params_.cv_threads);

    std::thread([this]() {
        // set_thread_priority(10);
        this->ImageProcessTask();
    }).detach();
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Send  request");
}

void Subscriber::ImgListenerCallback(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {
    std::lock_guard<std::mutex> lock(thread_sever_.mtx);
    thread_sever_.img_msg = msg;
    thread_sever_.cv.notify_one();
}

void Subscriber::ImageProcessTask() {
    sensor_msgs::msg::Image img_msg;
    while (rclcpp::ok()) {
        {
            std::unique_lock lock(thread_sever_.mtx);
            thread_sever_.cv.wait(lock, [&]() { return !!thread_sever_.img_msg; });
            if (thread_sever_.img_msg) {
                img_msg = *thread_sever_.img_msg;
                thread_sever_.img_msg.reset();
            }
        }
        SetFeatureHeader(img_msg.header);
        tl.node_start = TimeLog::GetTimePoint();
        this->current_id_ = img_msg.header.frame_id;
        cv::Mat res;
        auto features = frame_processor_->Run(img_msg, orders_, res);

        SetFeatureToPublish(features);
        Publish();
        tl.node_end = TimeLog::GetTimePoint();
        Logging();
        std_msgs::msg::String s;
        s.data = "detect_node";
        this->detect_heart_beat_->publish(s);
    }
}

void Subscriber::Publish() {
    interfaces::msg::Feature msg = *feature_;
    feature_publisher_->publish(msg);
    if (msg.isfan) {
        RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Publishing: Power Rune");
    } else {
        for (const auto& armor : msg.armors) {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*this->get_clock(), 1000, "Publishing: Armor "<< armor.armor_id);
        }
    }
}

void Subscriber::OrderListenerCallback(const interfaces::msg::Order::SharedPtr msg) {
    orders_.special_order = static_cast<MCU::AutoaimReceiveInfo::SpecialOrder>(msg->special_order);
    orders_.switch_mode = static_cast<MCU::AutoaimReceiveInfo::SwitchMode>(msg->switch_mode);
    orders_.target_type = static_cast<MCU::AutoaimReceiveInfo::TargetType>(msg->target_type);
    orders_.target_priority_rule = static_cast<MCU::AutoaimReceiveInfo::TargetPriorityRule>(msg->target_priority_rule);
    orders_.sentry_behavior_mode = static_cast<MCU::AutoaimReceiveInfo::SentryBehaviorMode>(msg->sentry_behavior_mode);
    orders_.color = static_cast<RM::Color>(msg->color);
}

void Subscriber::SetFeatureToPublish(const std::vector<std::shared_ptr<FP::IFeature>>& features) {
    fan_featere_.clear();
    armor_features_.clear();
    if (orders_.target_type == MCU::AutoaimReceiveInfo::TargetType::POWER_RUNE) {
        std::for_each(features.begin(), features.end(), [this](auto feature) {
            auto fan = static_pointer_cast<FP::FanInfo>(feature);
            this->fan_featere_.push_back(fan);
        });
        SetFeature(fan_featere_);
        SetTarget(true);
    } else {
        std::for_each(features.begin(), features.end(), [this](auto feature) {
            auto armor = static_pointer_cast<FP::ArmorInfo>(feature);
            auto a = *armor;
            this->armor_features_.push_back(armor);
        });
        SetFeature(armor_features_);
        SetTarget(false);
    }
}

void Subscriber::SetFeature(std::vector<std::shared_ptr<FP::ArmorInfo>> armor_features) {
    this->feature_->armors.clear();
    std::for_each(armor_features.begin(), armor_features.end(), [this](auto feature) {
        interfaces::msg::Armor armor;
        armor.armor_id = static_cast<int>(feature->armor_id);
        armor.armor_size = static_cast<int>(feature->armor_size);
        armor.keypoints.emplace_back(cvPointToGeometryPoint(feature->image_coord_armor_points.l_top));
        armor.keypoints.emplace_back(cvPointToGeometryPoint(feature->image_coord_armor_points.l_bottom));
        armor.keypoints.emplace_back(cvPointToGeometryPoint(feature->image_coord_armor_points.r_bottom));
        armor.keypoints.emplace_back(cvPointToGeometryPoint(feature->image_coord_armor_points.r_top));
        armor.is_dart_armor = feature->is_dart_armor;
        this->feature_->armors.emplace_back(armor);
    });
}

void Subscriber::SetFeature(std::vector<std::shared_ptr<FP::FanInfo>> fan_feature) {
    std::for_each(fan_feature.begin(), fan_feature.end(), [this](auto feature) {
        interfaces::msg::Fan fan;
        fan.keypoints.emplace_back(cvPointToGeometryPoint(feature->image_coord_lights_surface_points.left));
        fan.keypoints.emplace_back(cvPointToGeometryPoint(feature->image_coord_lights_surface_points.top));
        fan.keypoints.emplace_back(cvPointToGeometryPoint(feature->image_coord_lights_surface_points.right));
        fan.keypoints.emplace_back(cvPointToGeometryPoint(feature->image_coord_lights_surface_points.bottom));
        fan.image_coord_r_symbol_point = (cvPointToGeometryPoint(feature->image_coord_r_symbol_point));
        fan.is_target_fan = feature->is_target_fan;
        this->feature_->fan = fan;
    });
}

void Subscriber::SetTarget(bool isfan) {
    this->feature_->isfan = isfan;
}

void Subscriber::SetFeatureHeader(std_msgs::msg::Header header) {
    this->feature_->header = std::move(header);
    this->feature_->header.frame_id = "gimbal";
}

void Subscriber::ActivateResultCallback(rclcpp::Client<interfaces::srv::Activatesensor>::SharedFuture result_future) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto activate_request = std::make_shared<interfaces::srv::Activatesensor::Request>();
    activate_request->activate_sensor = interfaces::srv::Activatesensor::Request::CONTINUOUSTRIGGER;
    activate_client_->async_send_request(
        activate_request, [this](rclcpp::Client<interfaces::srv::Activatesensor>::SharedFuture result_future) {});
}

void Subscriber::ColorResultCallback(rclcpp::Client<interfaces::srv::Color>::SharedFuture result_future) {
    const auto& response = result_future.get();
    auto color_ = static_cast<RM::Color>(response->color);

    frame_processor_ = std::make_unique<FP::FrameProcessor>(color_);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "[Sub] Subscriber Initialized" << static_cast<int>(color_));
}

geometry_msgs::msg::Point detector::cvPointToGeometryPoint(const cv::Point& cv_point) {
    geometry_msgs::msg::Point ros_point;
    ros_point.x = static_cast<double>(cv_point.x);
    ros_point.y = static_cast<double>(cv_point.y);
    ros_point.z = 0.0;
    return ros_point;
}

geometry_msgs::msg::Point detector::cvPointToGeometryPoint(const cv::Point3d& cv_point) {
    geometry_msgs::msg::Point ros_point;
    ros_point.x = cv_point.x;
    ros_point.y = cv_point.y;
    ros_point.z = cv_point.z;
    return ros_point;
}

void Subscriber::Logging() {
    if (params_.is_time_log) {
        //RCLCPP_INFO_STREAM(this->get_logger(), "Img Handler: tid=" << getpid() << "/" << gettid());
        RCLCPP_INFO(this->get_logger(), "time : %ld",
                    std::chrono::duration_cast<std::chrono::milliseconds>(tl.node_end - tl.node_start).count());
        auto now = TimeLog::GetTimePoint();
        static auto last = now;
        RCLCPP_INFO(this->get_logger(), "true time : %ld",
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count());
        std_msgs::msg::Int64 t;
        t.set__data(std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count());
        time_log_publisher_->publish(t);
        last = now;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detector::Subscriber)
