//
// Created by wpie on 23-11-1.
//

#pragma once
#include <chrono>
#include <memory>
#include <opencv2/core/types.hpp>
#include <optional>

#include <sensor_msgs/msg/image.hpp>
#include "angles/angles.h"
#include "detect_node/frame_processer/hpps_k.h"
#include "detect_node/frame_processer/pnp_solver.h"
#include "detect_node/interface/i_detector.h"
#include "detect_node/interface/interface.h"
#include "detect_node/interface/mcu.h"
#include "detect_node/interface/rm_interface.h"
#include "detect_node/power_rune_detector/power_rune_detector.h"
#include "high_precision_pose_solver.h"
#include "opencv2/opencv.hpp"
#include "param_loader.hpp"

namespace FP {
class FrameProcessor {
   public:
    FrameProcessor();  // be used to placeholder

    explicit FrameProcessor(RM::Color input_color);

    [[nodiscard]] std::vector<std::shared_ptr<FP::IFeature>> Run(const sensor_msgs::msg::Image& msg,
                                                                 const MCU::Orders& input_order_data,
                                                                 cv::Mat& result_img);

   private:
    [[nodiscard]] std::vector<std::shared_ptr<FP::IFeature>> Detect(const cv::Mat& input_image,
                                                                    const MCU::Orders& input_order_data,
                                                                    cv::Mat& result_img);

    // cv::Point3d PoseSolve(std::vector<std::shared_ptr<FP::IFeature>>& input_features,
    //                const MCU::AutoaimReceiveInfo& input_mcu_data,
    //                const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_last_targets,
    //                std::chrono::high_resolution_clock::time_point last_after_ekf_timestamp);

    static void Verify(std::vector<std::shared_ptr<FP::IFeature>>& input_features, const MCU::Orders& input_order_data);

    // void ArmorCoordTransform(std::shared_ptr<FP::ArmorInfo>& input,
    //                          const MCU::AutoaimReceiveInfo& input_mcu_data,
    //                          const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>&
    //                          input_last_targets, std::chrono::high_resolution_clock::time_point
    //                          last_after_ekf_timestamp);

    // cv::Point3d PreciseArmorCoordTransform(std::shared_ptr<FP::ArmorInfo>& input,
    //                          const MCU::AutoaimReceiveInfo& input_mcu_data,
    //                          const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>&
    //                          input_last_targets, std::chrono::high_resolution_clock::time_point
    //                          last_after_ekf_timestamp);

    // void PowerRuneCoordTransform(std::shared_ptr<FP::FanInfo>& input_fan,
    //                              MCU::AutoaimReceiveInfo input_mcu_data);

    enum class ArmorDetectorMode { RV, OLD_VISION, PT4, NEW_CLASSIC_VISION };
    struct Param {
        const ArmorDetectorMode detector_mode =
            static_cast<ArmorDetectorMode>(ParamLoader::GetInstance().GetParam<int>("FP", "mode"));

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

    } param_;

    std::unique_ptr<IDetector> armor_detector_ = nullptr;

    std::unique_ptr<IDetector> secondary_traditional_detector_ = nullptr;

    std::unique_ptr<IDetector> power_rune_detector_ = nullptr;
};


}  // namespace FP
