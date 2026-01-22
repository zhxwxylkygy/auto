//
// Created by wpie on 23-11-22.
//

#pragma once

#include <optional>
#include "angles/angles.h"
#include "calculate/basic_calculate.h"
#include "eigen3/Eigen/Eigen"
#include "estimate_node/interface/rm_interface.h"
#include "opencv2/opencv.hpp"
#include "param_loader.hpp"


namespace FP {
class HighPrecisionPoseSolver {
   public:
    HighPrecisionPoseSolver() = default;

    using Contour = std::vector<cv::Point>;

    [[nodiscard]] std::optional<double> Run(const cv::Point3d& armor_pose,
                                            double input_gimbal_yaw,
                                            double input_gimbal_pitch,
                                            double input_gimbal_roll,
                                            const Contour& image_armor_points,
                                            RM::ArmorSize armor_size,
                                            double estimated_armor_yaw,
                                            bool is_outpost_spin,
                                            bool use_sencondary_camera,
                                            double inclined);

   private:
    [[nodiscard]] static double SJTU_get_pts_cost(const std::vector<cv::Point2f>& cv_refs,
                                                  const std::vector<cv::Point2f>& cv_pts,
                                                  const double& inclined);

    [[nodiscard]] std::optional<double> Trichotomy(double gimbal_yaw,
                                                   double gimbal_pitch,
                                                   double gimbal_roll,
                                                   const cv::Point3d& t,
                                                   RM::ArmorSize armor_size,
                                                   const Contour& image_cont,
                                                   double estimated_armor_yaw,
                                                   bool is_outpost_spin,
                                                   bool use_sencondary_camera,
                                                   double inclined) const;

    [[nodiscard]] std::vector<cv::Point2f> Reprojection(double armor_yaw,
                                                        double gimbal_yaw,
                                                        double gimbal_pitch,
                                                        double gimbal_roll,
                                                        const cv::Point3d& t,
                                                        RM::ArmorSize armor_size,
                                                        bool is_outpost_spin,
                                                        bool use_sencondary_camera) const;

    [[maybe_unused]] [[nodiscard]] static double ComputeIOU(const Contour& cont_1,
                                                            const Contour& cont_2,
                                                            const cv::Size& roi);

    struct Param {
        //            cv::Size roi_size = ParamLoader::Camera::IMAGE_SIZE;
        const double trichotomy_left_offset =
            angles::from_degrees(ParamLoader::GetInstance().GetParam<double>("FP", "TRICHOTOMY_LEFT_OFFSET_DEGREE"));
        const double trichotomy_right_offset =
            angles::from_degrees(ParamLoader::GetInstance().GetParam<double>("FP", "TRICHOTOMY_RIGHT_OFFSET_DEGREE"));
        const double trichotomy_termination_accuracy = angles::from_degrees(
            ParamLoader::GetInstance().GetParam<double>("FP", "TRICHOTOMY_TERMINATION_ACCURACY_DEGREE"));
        const int trichotomy_max_iterations =
            ParamLoader::GetInstance().GetParam<int>("FP", "TRICHOTOMY_MAX_ITERATIONS");

        const double small_armor_width = ParamLoader::GetInstance().GetParam<double>("RM", "SMALL_ARMOR_WIDTH");
        const double small_armor_height = ParamLoader::GetInstance().GetParam<double>("RM", "SMALL_ARMOR_HEIGHT");
        const double large_armor_width = ParamLoader::GetInstance().GetParam<double>("RM", "LARGE_ARMOR_WIDTH");
        const double large_armor_height = ParamLoader::GetInstance().GetParam<double>("RM", "LARGE_ARMOR_HEIGHT");

        const double yaw_to_geographic_x = ParamLoader::GetInstance().GetParam<double>("ROBOT", "YAW_TO_GEOGRAPHIC_X");
        const double yaw_to_geographic_y = ParamLoader::GetInstance().GetParam<double>("ROBOT", "YAW_TO_GEOGRAPHIC_Y");
        const double yaw_to_geographic_z = ParamLoader::GetInstance().GetParam<double>("ROBOT", "YAW_TO_GEOGRAPHIC_Z");
        const double pitch_to_yaw_x = ParamLoader::GetInstance().GetParam<double>("ROBOT", "PITCH_TO_YAW_X");
        const double pitch_to_yaw_y = ParamLoader::GetInstance().GetParam<double>("ROBOT", "PITCH_TO_YAW_Y");
        const double pitch_to_yaw_z = ParamLoader::GetInstance().GetParam<double>("ROBOT", "PITCH_TO_YAW_Z");
        const double camera_to_pitch_x = ParamLoader::GetInstance().GetParam<double>("ROBOT", "CAMERA_TO_PITCH_X");
        const double camera_to_pitch_y = ParamLoader::GetInstance().GetParam<double>("ROBOT", "CAMERA_TO_PITCH_Y");
        const double camera_to_pitch_z = ParamLoader::GetInstance().GetParam<double>("ROBOT", "CAMERA_TO_PITCH_Z");

        const double armor_pitch =
            angles::from_degrees(ParamLoader::GetInstance().GetParam<double>("RM", "ARMOR_PITCH_DEGREE"));

        const cv::Mat distortion_coefficient =
            ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA", "DISTORTION_COEFFICIENT").clone();
        const cv::Mat camera_intrinsics_mm =
            ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA", "INTRINSICS_MM").clone();

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

        const cv::Mat secondary_camera_intrinsics_mm =
            ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA", "SECONDARY_CAMERA_INTRINSICS_MM").clone();
        const cv::Mat secondary_camera_distortion_coefficient =
            ParamLoader::GetInstance().GetParam<cv::Mat>("CAMERA", "SECONDARY_CAMERA_DISTORTION_COEFFICIENT").clone();

    } param_;
};
}  // namespace FP
