#include <estimate_node/tracker_sub_node.h>
#include <vofa_bridge/vofa_bridge.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <interfaces/msg/armor.hpp>
#include <interfaces/msg/feature.hpp>
#include <interfaces/msg/order.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <string>
#include "estimate_node/interface/interface.h"
#include "estimate_node/interface/rm_interface.h"

using namespace trackers;

FeatureSubscriber::FeatureSubscriber(const rclcpp::NodeOptions& options) : Node("feature_subscriber", options) {
    RCLCPP_INFO_STREAM(this->get_logger(), "estimate node initilazing");
    rclcpp::Clock sys_clock(RCL_SYSTEM_TIME);

    order_subscription_ = this->create_subscription<interfaces::msg::Order>(
        "/manager/order", rclcpp::SensorDataQoS(),
        std::bind(&FeatureSubscriber::OrderListenerCallback, this, std::placeholders::_1));

    MT::MultichannelTracker::InitTargetVec(aimming_target);
    last_targets = aimming_target;
    // Subscriber with tf2 message_filter
    // tf2 relevant

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // subscriber and filter
    feature_sub_.subscribe(this, "/feature", rmw_qos_profile_sensor_data);
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<interfaces::msg::Feature>>(
        feature_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(1));

    // Register a callback with tf2_ros::MessageFilter to be called when
    // transforms are available
    tf2_filter_->registerCallback(&FeatureSubscriber::FeatureListenerCallback, this);

    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker_.header.frame_id = "world";
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;
    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    armor_marker_.ns = "armors";
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);

    tracker_heart_beat_ = this->create_publisher<std_msgs::msg::String>("/estimate/heartbeat", rclcpp::SensorDataQoS());

    timer_ = this->create_wall_timer(std::chrono::seconds(3), [this]() {
        std_msgs::msg::String s;
        s.data = "estimate_node";
        this->tracker_heart_beat_->publish(s);
    });

    RCLCPP_INFO_STREAM(this->get_logger(), "estimate node initilazed");
}

void FeatureSubscriber::FeatureListenerCallback(const interfaces::msg::Feature msg) {
    header_ = msg.header;
    //RCLCPP_INFO_STREAM(this->get_logger(), "header time " << header_.stamp.sec <<  header_.stamp.nanosec);
    std::array<double, 3> rpy_{};
    try {
        auto gimbal_tf = tf2_buffer_->lookupTransform("world", "gimbal", rclcpp::Time(0));
        auto msg_q = gimbal_tf.transform.rotation;
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);

    } catch (tf2::TransformException& ex) {
        printf("armor_solver %s", ex.what());
        // throw ex;
    }
    for (auto& [key, target] : aimming_target) {
        if (auto rvTarget = std::static_pointer_cast<MT::RVTargetInfo>(target))
            rvTarget->header = header_;
    }
    if (header_.frame_id != "None") {


        TARGET_TYPE type = TARGET_TYPE::NONE;
        if (msg.isfan) {
            fan_features_ = std::make_shared<interfaces::msg::Fan>(msg.fan);
            type = TARGET_TYPE::POWERRUNE;
            RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Got Power Rune Feature from FP");
        } else {
            std::for_each(msg.armors.begin(), msg.armors.end(), [this](auto armor_feature) {
                auto armor = std::make_shared<interfaces::msg::Armor>(armor_feature);
                this->armor_features_.emplace_back(armor);
            });
            type = TARGET_TYPE::ARMOR;
            RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Got Armor Feature from FP");
        }

        SetTargetValue(type);

        auto p = pose_solver_.PoseSolve(features_, orders, last_targets, after_ekf_timestamp, header_, tf2_buffer_);

         multichannel_tracker_.Run(aimming_target, features_, orders);

         features_.clear();
         after_ekf_timestamp = rclcpp::Clock().now();
         auto ets_data_o = ergo_target_selector_.Run(aimming_target, orders, rpy_[2]);
         PublishMarkers(ets_data_o, type);

        if (ets_data_o.has_value()) {
            auto target_solution = kinematics_solver_.Run(ets_data_o.value(), orders, p);
            autoaim_result_pub_node_->has_target_flag = true;
            autoaim_result_pub_node_->Setmcudata_(MCU::AutoaimSendInfo{
                target_solution.yaw +
                    angles::from_degrees(
                        has_secondary_camera
                            ? ParamLoader::GetInstance().GetParam<double>("KS", "SECONDARY_GIMBAL_YAW_OFFSET_DEGREE")
                            : ParamLoader::GetInstance().GetParam<double>("KS", "GIMBAL_YAW_OFFSET_DEGREE")),
                target_solution.pitch +
                    angles::from_degrees(
                        has_secondary_camera
                            ? ParamLoader::GetInstance().GetParam<double>("KS", "SECONDARY_GIMBAL_PITCH_OFFSET_DEGREE")
                            : ParamLoader::GetInstance().GetParam<double>("KS", "GIMBAL_PITCH_OFFSET_DEGREE")),
                norm(ets_data_o.value().second->target_ref_coord), ets_data_o.value().first, true,
                target_solution.enable_shoot});
                // vpie::VofaBridge::Get().SendOnce(target_solution.yaw +
                //     angles::from_degrees(
                //         has_secondary_camera
                //             ? ParamLoader::GetInstance().GetParam<double>("KS", "SECONDARY_GIMBAL_YAW_OFFSET_DEGREE")
                //            : ParamLoader::GetInstance().GetParam<double>("KS", "GIMBAL_YAW_OFFSET_DEGREE")), 
                //            target_solution.pitch +
                //            angles::from_degrees(
                //                has_secondary_camera
                //                    ? ParamLoader::GetInstance().GetParam<double>("KS", "SECONDARY_GIMBAL_PITCH_OFFSET_DEGREE")
                //                    : ParamLoader::GetInstance().GetParam<double>("KS", "GIMBAL_PITCH_OFFSET_DEGREE")));
        } else {
            autoaim_result_pub_node_->has_target_flag = false;
            RCLCPP_DEBUG_STREAM(this->get_logger(), "est.o no value");
        }
        last_targets = aimming_target;
    } else {
        autoaim_result_pub_node_->has_target_flag = false;
    }
    autoaim_result_pub_node_->publish();
}

void FeatureSubscriber::PublishMarkers(
    const std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>>& est_target,
    const TARGET_TYPE& type) {
    header_.frame_id = "world";
    position_marker_.header = header_;
    linear_v_marker_.header = header_;
    angular_v_marker_.header = header_;
    armor_marker_.header = header_;
    visualization_msgs::msg::MarkerArray marker_array;
    if (est_target.has_value()) {
        auto target = est_target.value();
        if (type == TARGET_TYPE::ARMOR) {
            if (static_cast<int>(target.first) <= 4 && static_cast<int>(target.first) >= 0) {
                auto aimming_static_target = static_pointer_cast<MT::StaticTargetInfo>(target.second);
                position_marker_.action = visualization_msgs::msg::Marker::ADD;
                position_marker_.pose.position.x = aimming_static_target->c_position.x;
                position_marker_.pose.position.y = aimming_static_target->c_position.y;
                position_marker_.pose.position.z = aimming_static_target->c_position.z;

                linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
                linear_v_marker_.points.clear();
                linear_v_marker_.points.emplace_back(position_marker_.pose.position);
                geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
                arrow_end.x += aimming_static_target->c_velocity.x;
                arrow_end.y += aimming_static_target->c_velocity.y;
                arrow_end.z += aimming_static_target->c_velocity.z;
                linear_v_marker_.points.emplace_back(arrow_end);

                armor_marker_.action = visualization_msgs::msg::Marker::ADD;
                armor_marker_.scale.y = aimming_static_target->armor_size == RM::ArmorSize::SMALL ? 0.135 : 0.23;
                auto a_n = static_cast<size_t>(aimming_static_target->target_armor_num);
                geometry_msgs::msg::Point p_a;
                double yaw = aimming_static_target->armors_vec[0]->geographic_coord_yaw;
                for (size_t i = 0; i < a_n; i++) {
                    double tmp_yaw = yaw + i * (2 * M_PI / a_n);
                    p_a = toGeometryPoint(aimming_static_target->armors_vec[i]->geographic_coord_pose);
                    armor_marker_.id = i;
                    armor_marker_.pose.position = p_a;
                    tf2::Quaternion q;
                    q.setRPY(0, aimming_static_target->armor_id == RM::ArmorId::OUTPOST ? -0.26 : 0.26, tmp_yaw);
                    armor_marker_.pose.orientation = tf2::toMsg(q);
                    marker_array.markers.emplace_back(armor_marker_);
                }
            } else if (static_cast<int>(target.first) >= 5 && static_cast<int>(target.first) <= 10) {
                auto aimming_rv_target = static_pointer_cast<MT::RVTargetInfo>(target.second);
                double yaw = aimming_rv_target->yaw, r1 = aimming_rv_target->r_1, r2 = aimming_rv_target->r_2;
                double xc = aimming_rv_target->c_position.x, yc = aimming_rv_target->c_position.y,
                       za = aimming_rv_target->c_position.z;
                double vx = aimming_rv_target->c_velocity.x, vy = aimming_rv_target->c_velocity.y,
                       vz = aimming_rv_target->c_velocity.z;
                double dz = aimming_rv_target->d_z;
                position_marker_.action = visualization_msgs::msg::Marker::ADD;
                position_marker_.pose.position.x = xc;
                position_marker_.pose.position.y = yc;
                position_marker_.pose.position.z = za + dz / 2;

                linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
                linear_v_marker_.points.clear();
                linear_v_marker_.points.emplace_back(position_marker_.pose.position);
                geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
                arrow_end.x += vx;
                arrow_end.y += vy;
                arrow_end.z += vz;
                linear_v_marker_.points.emplace_back(arrow_end);

                angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
                angular_v_marker_.points.clear();
                angular_v_marker_.points.emplace_back(position_marker_.pose.position);
                arrow_end = position_marker_.pose.position;
                arrow_end.z += aimming_rv_target->v_yaw / M_PI;
                angular_v_marker_.points.emplace_back(arrow_end);

                armor_marker_.action = visualization_msgs::msg::Marker::ADD;
                armor_marker_.scale.y = aimming_rv_target->armor_size == RM::ArmorSize::SMALL ? 0.135 : 0.23;
                bool is_current_pair = true;
                auto a_n = static_cast<size_t>(aimming_rv_target->armor_size);
                geometry_msgs::msg::Point p_a;
                double r = 0;
                for (size_t i = 0; i < a_n; i++) {
                    double tmp_yaw = yaw + i * (2 * M_PI / a_n);
                    // Only 4 armors has 2 radius and height
                    if (a_n == 4) {
                        r = is_current_pair ? r1 : r2;
                        p_a.z = za + (is_current_pair ? 0 : dz);
                        is_current_pair = !is_current_pair;
                    } else {
                        r = r1;
                        p_a.z = za;
                    }
                    p_a.x = xc - r * cos(tmp_yaw);
                    p_a.y = yc - r * sin(tmp_yaw);

                    armor_marker_.id = i;
                    armor_marker_.pose.position = p_a;
                    tf2::Quaternion q;
                    q.setRPY(0, 0.26, tmp_yaw);
                    armor_marker_.pose.orientation = tf2::toMsg(q);
                    marker_array.markers.emplace_back(armor_marker_);
                }
            }

        } else {  // type == TYPE::POWERRUNE
            auto aimming_power_rune_target = static_pointer_cast<MT::PowerRuneTargetInfo>(target.second);
        }
    } else {
        position_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
        linear_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
        angular_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
        armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.emplace_back(armor_marker_);
    }
    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(linear_v_marker_);
    marker_array.markers.emplace_back(angular_v_marker_);
    marker_pub_->publish(marker_array);
}

void FeatureSubscriber::OrderListenerCallback(const interfaces::msg::Order::SharedPtr msg) {
    orders.color = static_cast<RM::Color>(msg->color);
    orders.special_order = static_cast<MCU::AutoaimReceiveInfo::SpecialOrder>(msg->special_order);
    orders.target_type = static_cast<MCU::AutoaimReceiveInfo::TargetType>(msg->target_type);
    orders.sentry_behavior_mode = static_cast<MCU::AutoaimReceiveInfo::SentryBehaviorMode>(msg->sentry_behavior_mode);
    orders.switch_mode = static_cast<MCU::AutoaimReceiveInfo::SwitchMode>(msg->switch_mode);
    orders.target_priority_rule = static_cast<MCU::AutoaimReceiveInfo::TargetPriorityRule>(msg->target_priority_rule);
    orders.remaining_time_s = msg->remaining_time_s;
    //RCLCPP_INFO_STREAM(this->get_logger(), "time :" << orders.remaining_time_s);
}

void FeatureSubscriber::SetTargetValue(TARGET_TYPE type) {
    if (type == TARGET_TYPE::POWERRUNE) {
        auto fan = std::make_shared<FP::FanInfo>();
        fan->is_target_fan = fan_features_->is_target_fan;
        if (fan_features_->keypoints.size() == 4) {
            fan->image_coord_lights_surface_points.left = GeometryPointTocvPoint(fan_features_->keypoints[0]);
            fan->image_coord_lights_surface_points.top = GeometryPointTocvPoint(fan_features_->keypoints[1]);
            fan->image_coord_lights_surface_points.right = GeometryPointTocvPoint(fan_features_->keypoints[2]);
            fan->image_coord_lights_surface_points.bottom = GeometryPointTocvPoint(fan_features_->keypoints[3]);
        }
        fan->image_coord_r_symbol_point = GeometryPointTocvPoint(fan_features_->image_coord_r_symbol_point);
        fan->header = header_;
        if(fan->image_coord_lights_surface_points.left.x != 0 && fan->image_coord_lights_surface_points.right.x != 0.){
            this->features_.emplace_back(fan);
        }
        fan_features_.reset();
    } else {  // type == TARGET_TYPE::ARMOR

        std::for_each(armor_features_.begin(), armor_features_.end(), [this](auto armor_) {
            auto armor = std::make_shared<FP::ArmorInfo>();
            armor->armor_id = static_cast<RM::ArmorId>(armor_->armor_id);
            armor->armor_size = static_cast<RM::ArmorSize>(armor_->armor_size);
            armor->is_dart_armor = armor_->is_dart_armor;
            if (armor_->keypoints.size() == 4) {
                armor->image_coord_armor_points.l_top = GeometryPointTocvPoint(armor_->keypoints[0]);
                armor->image_coord_armor_points.l_bottom = GeometryPointTocvPoint(armor_->keypoints[1]);
                armor->image_coord_armor_points.r_bottom = GeometryPointTocvPoint(armor_->keypoints[2]);
                armor->image_coord_armor_points.r_top = GeometryPointTocvPoint(armor_->keypoints[3]);
            }
            armor->header = header_;
            if(armor->image_coord_armor_points.l_top.x != 0 && armor->image_coord_armor_points.r_bottom.x != 0){
                this->features_.emplace_back(armor);
            }
        });
        armor_features_.clear();
    }
}

cv::Point trackers::GeometryPointTocvPoint(const geometry_msgs::msg::Point& geom_point) {
    return {static_cast<int>(geom_point.x), static_cast<int>(geom_point.y)};
}

cv::Point3d trackers::GeometryPointTocvPoint3d(const geometry_msgs::msg::Point& geom_point) {
    return {geom_point.x, geom_point.y, geom_point.z};
}

geometry_msgs::msg::Point trackers::toGeometryPoint(const cv::Point3d& cv_point) {
    geometry_msgs::msg::Point p;
    p.set__x(cv_point.x);
    p.set__y(cv_point.y);
    p.set__z(cv_point.z);
    return p;
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trackers::FeatureSubscriber)