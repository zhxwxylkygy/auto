//
// Created by wpie on 23-11-2.
//

#include "estimate_node/pose_solver/pnp_solver.h"
#include <vofa_bridge/vofa_bridge.h>

#include <opencv2/calib3d.hpp>
#include <vector>
#include "angles/angles.h"

namespace FP {
CameraModuleSolver::CameraModuleSolver() {
    double small_armor_half_y = param_.small_armor_width_mm / 2.0;
    double small_armor_half_z = param_.small_armor_height_mm / 2.0;
    double large_armor_half_y = param_.large_armor_width_mm / 2.0;
    double large_armor_half_z = param_.large_armor_height_mm / 2.0;

    // Start from bottom left in clockwise order
    // Model coordinate: x forward, y left, z up
    small_armor_points_.emplace_back(0, (float)small_armor_half_y, (float)-small_armor_half_z);
    small_armor_points_.emplace_back(0, (float)small_armor_half_y, (float)small_armor_half_z);
    small_armor_points_.emplace_back(0, (float)-small_armor_half_y, (float)small_armor_half_z);
    small_armor_points_.emplace_back(0, (float)-small_armor_half_y, (float)-small_armor_half_z);

    large_armor_points_.emplace_back(0, (float)large_armor_half_y, (float)-large_armor_half_z);
    large_armor_points_.emplace_back(0, (float)large_armor_half_y, (float)large_armor_half_z);
    large_armor_points_.emplace_back(0, (float)-large_armor_half_y, (float)large_armor_half_z);
    large_armor_points_.emplace_back(0, (float)-large_armor_half_y, (float)-large_armor_half_z);

    const float r = param_.power_rune_aim_radius_mm;
    power_rune_armor_points_.emplace_back(0, r, 0);
    power_rune_armor_points_.emplace_back(0, 0, r);
    power_rune_armor_points_.emplace_back(0, -r, 0);
    power_rune_armor_points_.emplace_back(0, 0, -r);
}

bool CameraModuleSolver::SolvePnP(std::shared_ptr<FP::FanInfo>& input_fan) const {
    std::vector<cv::Point2f> image_armor_points;
    std::vector<cv::Point3f> object_points;
    cv::Mat r_vec;
    cv::Mat t_vec_mm;
    image_armor_points.emplace_back(input_fan->image_coord_lights_surface_points.left);
    image_armor_points.emplace_back(input_fan->image_coord_lights_surface_points.top);
    image_armor_points.emplace_back(input_fan->image_coord_lights_surface_points.right);
    image_armor_points.emplace_back(input_fan->image_coord_lights_surface_points.bottom);
    object_points = power_rune_armor_points_;

    auto is_success = cv::solvePnP(object_points, image_armor_points, param_.intrinsics_mm,
                                   param_.distortion_coefficient, r_vec, t_vec_mm, false, cv::SOLVEPNP_IPPE);
    auto fan = std::static_pointer_cast<FP::FanInfo>(input_fan);
    auto t_vec = t_vec_mm;
    t_vec.at<double>(0) /= 1000;  // �?�?为m
    t_vec.at<double>(1) /= 1000;
    t_vec.at<double>(2) /= 1000;
    fan->surface_points_inter_bull_pnp_r_vec = r_vec;
    fan->surface_points_inter_bull_pnp_t_vec = t_vec;

    return is_success;
}

bool CameraModuleSolver::SolvePnP(std::shared_ptr<FP::ArmorInfo>& input_armor, bool use_secondary_camera) const {
    std::vector<cv::Point2f> image_armor_points;
    std::vector<cv::Point3f> object_points;
    cv::Mat r_vec;
    cv::Mat t_vec_mm;

    image_armor_points.emplace_back(input_armor->image_coord_armor_points.l_bottom);
    image_armor_points.emplace_back(input_armor->image_coord_armor_points.l_top);
    image_armor_points.emplace_back(input_armor->image_coord_armor_points.r_top);
    image_armor_points.emplace_back(input_armor->image_coord_armor_points.r_bottom);
    if (input_armor->armor_size == RM::ArmorSize::SMALL) {
        object_points = small_armor_points_;
    } else if (input_armor->armor_size == RM::ArmorSize::LARGE) {
        object_points = large_armor_points_;
    } else {
        std::cerr << "pnp solver: wrong armor size!" << std::endl;
        std::terminate();
    }

    auto is_success = cv::solvePnP(
        object_points, image_armor_points,
        use_secondary_camera ? param_.secondary_camera_intrinsics_mm : param_.intrinsics_mm,
        use_secondary_camera ? param_.secondary_camera_distortion_coefficient : param_.distortion_coefficient, r_vec,
        t_vec_mm, false, cv::SOLVEPNP_IPPE);
    auto armor = std::static_pointer_cast<FP::ArmorInfo>(input_armor);
    auto t_vec = t_vec_mm;
    t_vec.at<double>(0) /= 1000;  // �?�?为m
    t_vec.at<double>(1) /= 1000;
    t_vec.at<double>(2) /= 1000;
    armor->pnp_r_vec = r_vec;
    armor->pnp_t_vec = t_vec;
    return is_success;

}

void CameraModuleSolver::SolvePinHole(std::shared_ptr<FP::FanInfo>& input_fan) const {
    double fx = param_.intrinsics_mm.at<double>(0, 0);
    double fy = param_.intrinsics_mm.at<double>(1, 1);
    double cx = param_.intrinsics_mm.at<double>(0, 2);
    double cy = param_.intrinsics_mm.at<double>(1, 2);
    cv::Point2f undistort_point;
    std::vector<cv::Point2f> undistort_point_vec;

    cv::undistortPoints(std::vector<cv::Point2f>{input_fan->image_coord_r_symbol_point}, undistort_point_vec,
                        param_.intrinsics_mm, param_.distortion_coefficient, cv::noArray(), param_.intrinsics_mm);
    undistort_point = undistort_point_vec.at(0);

    double tan_yaw = (undistort_point.x - cx) / fx;  // ?
    double tan_pitch = (undistort_point.y - cy) / fy;

    input_fan->right_handed_camera_coord_r_symbol_theta = angles::from_degrees(90) - (atan(tan_pitch));
    input_fan->right_handed_camera_coord_r_symbol_phi = -atan(tan_yaw);

    //        std::cout << angles::to_degrees(-atan(tan_yaw)) << " " << angles::to_degrees(atan(tan_pitch)) <<
    //        std::endl;
}

}  // namespace FP
