
#pragma once
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <memory>
#include <opencv2/core/types.hpp>
#include <optional>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include "angles/angles.h"
#include "estimate_node/interface/interface.h"
#include "estimate_node/interface/mcu.h"
#include "estimate_node/interface/rm_interface.h"
#include "high_precision_pose_solver.h"
#include "hpps_k.h"
#include "opencv2/opencv.hpp"
#include "param_loader.hpp"
#include "pnp_solver.h"

namespace FP {
class PoseSolver {
   public:
    PoseSolver() = default;

    cv::Point3d PoseSolve(std::vector<std::shared_ptr<FP::IFeature>>& input_features,
                          const MCU::Orders& input_order_data,
                          const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_last_targets,
                          rclcpp::Time last_after_ekf_timestamp,
                          std_msgs::msg::Header header,
                          std::shared_ptr<tf2_ros::Buffer> tf2_buffer);

   private:
    void ArmorCoordTransform(std::shared_ptr<FP::ArmorInfo>& input,
                             const MCU::Orders& input_order_data,
                             const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_last_targets,
                             rclcpp::Time last_after_ekf_timestamp);

    cv::Point3d PreciseArmorCoordTransform(
        std::shared_ptr<FP::ArmorInfo>& input,
        const MCU::Orders& input_order_data,
        const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_last_targets,
        rclcpp::Time last_after_ekf_timestamp);

    void PowerRuneCoordTransform(std::shared_ptr<FP::FanInfo>& input_fan, const MCU::Orders& input_order_data);
    struct Param {
        // todo add to param loader

        const double gimbal_yaw_to_geographic_x =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "YAW_TO_GEOGRAPHIC_X");
        const double gimbal_yaw_to_geographic_y =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "YAW_TO_GEOGRAPHIC_Y");
        const double gimbal_yaw_to_geographic_z =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "YAW_TO_GEOGRAPHIC_Z");
        const double gimbal_pitch_to_gimbal_yaw_x =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "PITCH_TO_YAW_X");
        const double gimbal_pitch_to_gimbal_yaw_y =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "PITCH_TO_YAW_Y");
        const double gimbal_pitch_to_gimbal_yaw_z =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "PITCH_TO_YAW_Z");
        const double camera_to_gimbal_pitch_x =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "CAMERA_TO_PITCH_X");
        const double camera_to_gimbal_pitch_y =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "CAMERA_TO_PITCH_Y");
        const double camera_to_gimbal_pitch_z =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "CAMERA_TO_PITCH_Z");
        const double camera_to_gimbal_pitch_install_pitch_offset = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "CAMERA_TO_PITCH_INSTALL_PITCH_OFFSET_DEGREE"));
        const double camera_to_gimbal_pitch_install_yaw_offset = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "CAMERA_TO_PITCH_INSTALL_YAW_OFFSET_DEGREE"));

        const double secondary_camera_to_gimbal_pitch_x =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "SECONDARY_CAMERA_TO_PITCH_X");
        const double secondary_camera_to_gimbal_pitch_y =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "SECONDARY_CAMERA_TO_PITCH_Y");
        const double secondary_camera_to_gimbal_pitch_z =
            ParamLoader::GetInstance().GetParam<double>("ROBOT", "SECONDARY_CAMERA_TO_PITCH_Z");
        const double secondary_camera_to_gimbal_pitch_install_pitch_offset = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("ROBOT",
                                                        "SECONDARY_CAMERA_TO_PITCH_INSTALL_PITCH_OFFSET_DEGREE"));
        const double secondary_camera_to_gimbal_pitch_install_yaw_offset = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("ROBOT",
                                                        "SECONDARY_CAMERA_TO_PITCH_INSTALL_YAW_OFFSET_DEGREE"));

        const bool advanced_transform = ParamLoader::GetInstance().GetParam<bool>("ROBOT", "ADVANCED_TRANSFORM");
        const double timestamp_offset =
            ParamLoader::GetInstance().GetParam<double>("ESTIMATE", "TIME_STAMP_OFFSET_IN_SECOND");
    } param_;
    const CameraModuleSolver camera_module_solver_;
    HPPSK hppsk_;
    HighPrecisionPoseSolver hpps_;
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    struct ImuData {
        double imu_pitch;
        double imu_yaw;
        double imu_roll;
    };
    ImuData imu_data;

    double motor_pitch;
    double motor_yaw;
    double empty_roll;
};

class ArmorComparator {
   public:
    static std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> FindMatchRVTarget(
        const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_last_targets,
        RM::ArmorId this_armor_id,
        RM::ArmorSize this_armor_size);

    static double GetAbsBetweenArmorOffset(RM::TargetClassify target_classify);

    static bool AOnLeftOfB(cv::Point3d a, cv::Point3d b);

    static std::optional<double> GetGeographicYaw(cv::Point3d geographic_armor_pose,
                                                  rclcpp::Time last_time_points,
                                                  const std::shared_ptr<MT::RVTargetInfo>& last_target,
                                                  double between_armor_offset);
};
}  // namespace FP
