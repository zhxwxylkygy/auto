//
// Created by wpie on 23-11-2.
//

#pragma once

#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>
#include "detect_node/interface/interface.h"
#include "param_loader.hpp"

namespace FP {
    class CameraModuleSolver {
    public:
        CameraModuleSolver();

        [[nodiscard]] bool SolvePnP(std::shared_ptr<FP::ArmorInfo> &input_armor, bool use_secondary_camera) const;

        [[nodiscard]] bool SolvePnP(std::shared_ptr<FP::FanInfo> &input_fan) const;

        void SolvePinHole(std::shared_ptr<FP::FanInfo> &input_fan) const;

    private:
        struct Param {
            const cv::Mat intrinsics_mm = ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA","INTRINSICS_MM");
            const cv::Mat distortion_coefficient = ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA","DISTORTION_COEFFICIENT");
            const cv::Mat secondary_camera_intrinsics_mm = ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA","SECONDARY_CAMERA_INTRINSICS_MM");
            const cv::Mat secondary_camera_distortion_coefficient = ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA","SECONDARY_CAMERA_DISTORTION_COEFFICIENT");

            // Unit: mm
            const float small_armor_width_mm = static_cast<float>(ParamLoader::GetInstance().GetParam<double>("RM","SMALL_ARMOR_WIDTH") * 1000.);
            const float small_armor_height_mm = static_cast<float>(ParamLoader::GetInstance().GetParam<double>("RM","SMALL_ARMOR_HEIGHT") * 1000.);
            const float large_armor_width_mm = static_cast<float>(ParamLoader::GetInstance().GetParam<double>("RM","LARGE_ARMOR_WIDTH") * 1000.);
            const float large_armor_height_mm = static_cast<float>(ParamLoader::GetInstance().GetParam<double>("RM","LARGE_ARMOR_HEIGHT") * 1000.);
            const float power_rune_top_side_length_mm = static_cast<float>(ParamLoader::GetInstance().GetParam<double>("RM","POWER_RUNE_TOP_SIDE_LENGTH") * 1000.);
            const float power_rune_bottom_side_length_mm = static_cast<float>(ParamLoader::GetInstance().GetParam<double>("RM","POWER_RUNE_BOTTOM_SIDE_LENGTH") * 1000.);
            const float power_rune_top_side_to_centre_distance_mm =
                    static_cast<float>(ParamLoader::GetInstance().GetParam<double>("RM","POWER_RUNE_TOP_SIDE_TO_CENTRE_DISTANCE") * 1000.);
            const float power_rune_bottom_side_to_centre_distance_mm =
                    static_cast<float>(ParamLoader::GetInstance().GetParam<double>("RM","POWER_RUNE_BOTTOM_SIDE_TO_CENTRE_DISTANCE") * 1000.);
        } param_;

        // Four vertices of armor in 3d
        std::vector<cv::Point3f> small_armor_points_;
        std::vector<cv::Point3f> large_armor_points_;
        std::vector<cv::Point3f> power_rune_armor_points_;
    };

}  // namespace rm_auto_aim

