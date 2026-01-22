#pragma once
#include <estimate_node/ergo_target_selector/ergo_target_selector.h>
#include <estimate_node/interface/mcu.h>
#include <estimate_node/interface/rm_interface.h>
#include <estimate_node/kinematics_solver/kinematics_solver.h>
#include <estimate_node/multichannel_tracker/multichannel_tracker.h>
#include <estimate_node/tracker_pub_node.h>
#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <interfaces/msg/detail/armor__struct.hpp>
#include <interfaces/msg/detail/fan__struct.hpp>
#include <interfaces/msg/detail/feature__struct.hpp>
#include <interfaces/msg/detail/mcucolor__struct.hpp>
#include <interfaces/msg/detail/order__struct.hpp>
#include <interfaces/msg/feature.hpp>
#include <interfaces/msg/mcucolor.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "estimate_node/pose_solver/pose_solver.h"
#include "interface/interface.h"
namespace trackers {

class FeatureSubscriber : public rclcpp::Node {
   public:
    explicit FeatureSubscriber(const rclcpp::NodeOptions& options);

   private:
    // Subscriber with tf2 message_filter

    std::string target_frame_ = "gimbal";
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<interfaces::msg::Feature> feature_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<interfaces::msg::Feature>> tf2_filter_;

   private:
    void OrderListenerCallback(const interfaces::msg::Order::SharedPtr msg);
    rclcpp::Subscription<interfaces::msg::Order>::SharedPtr order_subscription_;
    std::shared_ptr<MCU::AutoaimReceiveInfo> mcudata_ = std::make_shared<MCU::AutoaimReceiveInfo>();
    MCU::Orders orders{};

   private:
    enum class TARGET_TYPE {
        NONE,
        POWERRUNE,
        ARMOR,
    };
    void FeatureListenerCallback(const interfaces::msg::Feature msg);
    void SetTargetValue(TARGET_TYPE type);
    rclcpp::Subscription<interfaces::msg::Feature>::SharedPtr feature_subscription_;
    std::vector<std::shared_ptr<interfaces::msg::Armor>> armor_features_{};
    std::shared_ptr<interfaces::msg::Fan> fan_features_ = std::make_shared<interfaces::msg::Fan>();
    std_msgs::msg::Header header_;

   private:
    MT::MultichannelTracker multichannel_tracker_{};
    ETS::ErgoTargetSelector ergo_target_selector_{};
    KS::KinematicsSolver kinematics_solver_{};
    FP::PoseSolver pose_solver_{};

    std::vector<std::shared_ptr<FP::IFeature>> features_;
    std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>> aimming_target{
        std::make_pair(RM::TargetClassify::BASE_DART, std::make_shared<MT::StaticTargetInfo>()),
        std::make_pair(RM::TargetClassify::BASE_BOTTOM, std::make_shared<MT::StaticTargetInfo>()),
        std::make_pair(RM::TargetClassify::BASE_TOP, std::make_shared<MT::StaticTargetInfo>()),
        std::make_pair(RM::TargetClassify::OUTPOST_DART, std::make_shared<MT::StaticTargetInfo>()),
        std::make_pair(RM::TargetClassify::OUTPOST_SPIN, std::make_shared<MT::RVTargetInfo>()),
        std::make_pair(RM::TargetClassify::HERO_1, std::make_shared<MT::RVTargetInfo>()),
        std::make_pair(RM::TargetClassify::ENGINEER_2, std::make_shared<MT::RVTargetInfo>()),
        std::make_pair(RM::TargetClassify::STANDARD_3, std::make_shared<MT::RVTargetInfo>()),
        std::make_pair(RM::TargetClassify::STANDARD_4, std::make_shared<MT::RVTargetInfo>()),
        std::make_pair(RM::TargetClassify::STANDARD_5, std::make_shared<MT::RVTargetInfo>()),
        std::make_pair(RM::TargetClassify::SENTRY, std::make_shared<MT::RVTargetInfo>()),
        std::make_pair(RM::TargetClassify::POWER_RUNE, std::make_shared<MT::PowerRuneTargetInfo>())};
    std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>> last_targets;
    rclcpp::Time after_ekf_timestamp;
    bool has_secondary_camera =
        !ParamLoader::GetInstance().GetParam<std::string>("CAMERA", "SECONDARY_CAMERA_SN").empty();

   private:
    std::shared_ptr<AutoaimResultPublisher> autoaim_result_pub_node_ =
        std::make_shared<AutoaimResultPublisher>(rclcpp::NodeOptions().use_intra_process_comms(true));

   private:
    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker linear_v_marker_;
    visualization_msgs::msg::Marker angular_v_marker_;
    visualization_msgs::msg::Marker armor_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    void PublishMarkers(
        const std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>>& est_target,
        const TARGET_TYPE& type);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracker_heart_beat_;

    rclcpp::TimerBase::SharedPtr timer_;
};

cv::Point GeometryPointTocvPoint(const geometry_msgs::msg::Point& geom_point);
cv::Point3d GeometryPointTocvPoint3d(const geometry_msgs::msg::Point& geom_point);
geometry_msgs::msg::Point toGeometryPoint(const cv::Point3d& cv_point);
}  // namespace trackers