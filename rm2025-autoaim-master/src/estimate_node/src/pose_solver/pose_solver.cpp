//
// Created by wpie on 23-11-1.
//

#include "estimate_node/pose_solver/pose_solver.h"
#include <bits/chrono.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vofa_bridge/vofa_bridge.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <execution>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include "angles/angles.h"
#include "calculate/basic_calculate.h"
#include "calculate/line_intersect_sphere_solver.h"
#include "estimate_node/interface/interface.h"
#include "estimate_node/interface/mcu.h"
#include "estimate_node/interface/rm_interface.h"

using namespace FP;

cv::Point3d PoseSolver::PoseSolve(
    std::vector<std::shared_ptr<FP::IFeature>>& input_features,
    const MCU::Orders& input_order_data,
    const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_last_targets,
    rclcpp::Time last_after_ekf_timestamp,
    std_msgs::msg::Header header,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer) {
    static cv::Point3d p;
    try {
        auto gimbal_tf = tf2_buffer->lookupTransform(
            "world", "gimbal", rclcpp::Time(header.stamp) + rclcpp::Duration::from_seconds(param_.timestamp_offset));
        auto msg_q = gimbal_tf.transform.rotation;

        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3(tf_q).getRPY(imu_data.imu_roll, imu_data.imu_pitch, imu_data.imu_yaw);
        auto& v = vpie::VofaBridge::Get();
        //v.SendOnce((rclcpp::Clock().now().seconds() - rclcpp::Time(header.stamp).seconds())*100 , imu_data.imu_yaw, imu_data.imu_roll);
        auto true_gimbal_tf = tf2_buffer->lookupTransform("world", "true_gimbal", rclcpp::Time(header.stamp) + rclcpp::Duration::from_seconds(param_.timestamp_offset));
        auto true_msg_q = true_gimbal_tf.transform.rotation;
        // std::cout << "yaw" << imu_data.imu_yaw << "pitch" << imu_data.imu_pitch << std::endl;
        tf2::Quaternion true_tf_q;
        tf2::fromMsg(true_msg_q, true_tf_q);
        tf2::Matrix3x3(true_tf_q).getRPY(empty_roll, motor_pitch, motor_yaw);
    } catch (tf2::TransformException& ex) {
        printf("armor_solver %s", ex.what());
        // throw ex;
    }

    if (input_order_data.target_type == MCU::AutoaimReceiveInfo::TargetType::POWER_RUNE) {
        //        std::execution::par_unseq,
        std::for_each(input_features.begin(), input_features.end(), [this, &input_order_data](auto& input_feature) {
            auto fan = std::static_pointer_cast<FP::FanInfo>(input_feature);
            auto is_pnp_success = camera_module_solver_.SolvePnP(fan);
            if (!is_pnp_success) {
                std::cerr << "pnp solve failed" << std::endl;
                std::terminate();
            }
            PowerRuneCoordTransform(fan, input_order_data);
        });

    } else {
        //        std::execution::par_unseq,
        std::for_each(
            input_features.begin(), input_features.end(),
            [this, &input_order_data, &p, &input_last_targets, &last_after_ekf_timestamp](auto& input_feature) {
                auto armor = std::static_pointer_cast<FP::ArmorInfo>(input_feature);
                auto is_pnp_success = camera_module_solver_.SolvePnP(
                    armor,
                    input_order_data.special_order == MCU::AutoaimReceiveInfo::SpecialOrder::USE_SECONDARY_CAMERA);
                if (!is_pnp_success) {
                    std::cerr << "pnp solve failed" << std::endl;
                    std::terminate();
                }

                if (param_.advanced_transform) {
                    p = PreciseArmorCoordTransform(armor, input_order_data, input_last_targets,
                                                   last_after_ekf_timestamp);
                } else {
                    ArmorCoordTransform(armor, input_order_data, input_last_targets, last_after_ekf_timestamp);
                }
            });
    }
    return p;
}

void PoseSolver::ArmorCoordTransform(
    std::shared_ptr<FP::ArmorInfo>& input,
    const MCU::Orders& input_order_data,
    const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_last_targets,
    rclcpp::Time last_after_ekf_timestamp) {
    auto gimbal_yaw = imu_data.imu_yaw;
    auto gimbal_pitch = imu_data.imu_pitch;
    auto gimbal_roll = imu_data.imu_roll;

    cv::Mat T_yaw_to_geographic = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_pitch_to_yaw = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_camera_to_pitch = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_camera_to_geographic = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat T_model_to_camera = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_model_to_geographic = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat R_model_to_camera;
    cv::Rodrigues(input->pnp_r_vec, R_model_to_camera);

    R_model_to_camera.copyTo(T_model_to_camera(cv::Rect(0, 0, 3, 3)));
    input->pnp_t_vec.copyTo(T_model_to_camera(cv::Rect(3, 0, 1, 3)));

    // auto& v = vpie::VofaBridge::Get();
    // v.SendOnce(
    //     input->pnp_t_vec.at<double>(0), input->pnp_t_vec.at<double>(1),
    //     input->pnp_t_vec.at<double>(2));

    cv::Mat t_yaw_to_geographic = (cv::Mat_<double>(3, 1) << param_.gimbal_yaw_to_geographic_x,
                                   param_.gimbal_yaw_to_geographic_y, param_.gimbal_yaw_to_geographic_z);
    cv::Mat R_yaw_to_geographic = (cv::Mat_<double>(3, 3) << cos(-gimbal_yaw), sin(-gimbal_yaw), 0, -sin(-gimbal_yaw),
                                   cos(-gimbal_yaw), 0, 0, 0, 1);
    R_yaw_to_geographic.copyTo(T_yaw_to_geographic(cv::Rect(0, 0, 3, 3)));
    t_yaw_to_geographic.copyTo(T_yaw_to_geographic(cv::Rect(3, 0, 1, 3)));

    cv::Mat t_pitch_to_yaw = (cv::Mat_<double>(3, 1) << param_.gimbal_pitch_to_gimbal_yaw_x,
                              param_.gimbal_pitch_to_gimbal_yaw_y, param_.gimbal_pitch_to_gimbal_yaw_z);
    cv::Mat R_pitch_to_yaw = (cv::Mat_<double>(3, 3) << cos(-gimbal_pitch), 0, -sin(-gimbal_pitch), 0, 1, 0,
                              sin(-gimbal_pitch), 0, cos(-gimbal_pitch));
    R_pitch_to_yaw.copyTo(T_pitch_to_yaw(cv::Rect(0, 0, 3, 3)));
    t_pitch_to_yaw.copyTo(T_pitch_to_yaw(cv::Rect(3, 0, 1, 3)));

    bool use_secondary_camera =
        input_order_data.special_order == MCU::AutoaimReceiveInfo::SpecialOrder::USE_SECONDARY_CAMERA;

    cv::Mat t_camera_to_pitch;
    if (use_secondary_camera) {
        t_camera_to_pitch = (cv::Mat_<double>(3, 1) << param_.secondary_camera_to_gimbal_pitch_x,
                             param_.secondary_camera_to_gimbal_pitch_y, param_.secondary_camera_to_gimbal_pitch_z);
    } else {
        t_camera_to_pitch = (cv::Mat_<double>(3, 1) << param_.camera_to_gimbal_pitch_x, param_.camera_to_gimbal_pitch_y,
                             param_.camera_to_gimbal_pitch_z);
    }

    cv::Mat R_camera_to_pitch;

    R_camera_to_pitch = (cv::Mat_<double>(3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0);

    t_camera_to_pitch.copyTo(T_camera_to_pitch(cv::Rect(3, 0, 1, 3)));

    cv::Mat R_camera_to_pitch_install_pitch_offset;
    if (use_secondary_camera) {
        R_camera_to_pitch_install_pitch_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset), 0,
             -sin(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset), 0, 1, 0,
             sin(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset), 0,
             cos(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset));
    } else {
        R_camera_to_pitch_install_pitch_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0,
             -sin(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0, 1, 0,
             sin(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0,
             cos(-param_.camera_to_gimbal_pitch_install_pitch_offset));
    }

    cv::Mat R_camera_to_pitch_install_yaw_offset;
    if (use_secondary_camera) {
        R_camera_to_pitch_install_yaw_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset),
             sin(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset), 0,
             -sin(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset),
             cos(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset), 0, 0, 0, 1);
    } else {
        R_camera_to_pitch_install_yaw_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.camera_to_gimbal_pitch_install_yaw_offset),
             sin(-param_.camera_to_gimbal_pitch_install_yaw_offset), 0,
             -sin(-param_.camera_to_gimbal_pitch_install_yaw_offset),
             cos(-param_.camera_to_gimbal_pitch_install_yaw_offset), 0, 0, 0, 1);
    }
    // std::cout << gimbal_roll << std::endl;

    gimbal_roll = 0;

    cv::Mat R_roll_offset = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(-gimbal_roll), -sin(-gimbal_roll), 0,
                             sin(-gimbal_roll), cos(-gimbal_roll));

    R_camera_to_pitch = R_roll_offset * R_camera_to_pitch_install_yaw_offset * R_camera_to_pitch_install_pitch_offset *
                        R_camera_to_pitch;
    R_camera_to_pitch.copyTo(T_camera_to_pitch(cv::Rect(0, 0, 3, 3)));

    T_camera_to_geographic = T_yaw_to_geographic * T_pitch_to_yaw * T_camera_to_pitch;
    T_model_to_geographic = T_camera_to_geographic * T_model_to_camera;
    cv::Mat result = T_model_to_geographic.inv();
    //                      cv::Mat result = T_model_to_geographic;
    cv::Mat p = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
    p = T_model_to_geographic * p;

    //                      std::cout << T_model_to_geographic << std::endl;
    //                      assert(0);

    input->geographic_coord_pose = cv::Point3d{p.at<double>(0), p.at<double>(1), p.at<double>(2)};

    // vpie::VofaBridge::Get().SendOnce(input->geographic_coord_pose.z);

    // DH::DebugHelper::GetInstance().ShowInGeographicCoord(input->geographic_coord_pose, gimbal_yaw, gimbal_pitch,
    //                                                     "armor");
    // auto& v = vpie::VofaBridge::Get();
    // v.SendOnce(
    //     (float)angles::to_degrees(atan2(input->geographic_coord_pose.y, input->geographic_coord_pose.x)),
    //     (float)angles::to_degrees(atan2(input->geographic_coord_pose.z,
    //                                     GetDistance(input->geographic_coord_pose.x,
    //                                     input->geographic_coord_pose.y))), 
    //     (float)angles::to_degrees(atan2(GetDistance(input->pnp_t_vec.at<double>(2),
    //         input->pnp_t_vec.at<double>(0)), input->pnp_t_vec.at<double>(1))));

    // std::cout << (float)angles::to_degrees(atan2(input->geographic_coord_pose.y, input->geographic_coord_pose.x)) <<"
    // "
    //           << (float)angles::to_degrees(
    //                  atan2(input->geographic_coord_pose.z,
    //                        GetDistance(input->geographic_coord_pose.x, input->geographic_coord_pose.y)))
    //           << std::endl;

    auto yaw1 = atan2(result.at<double>(0, 1), result.at<double>(1, 1));
    // todo 旋转矩阵�??欧拉角的�??式可能存�?�??�异�??
    input->geographic_coord_yaw = yaw1;

    //                      std::cout << result.inv() << std::endl;
    //  v_1.Send((float) angles::to_degrees(input->geographic_coord_yaw));

    // hpps test
    // double default_inclined = 0;
    static double inclined = 0;
    auto rv_target_opt = ArmorComparator::FindMatchRVTarget(input_last_targets, input->armor_id, input->armor_size);
    if (!rv_target_opt.has_value()) {
        // inclined = default_inclined;
    } else {
        auto geographic_yaw_opt = ArmorComparator::GetGeographicYaw(
            input->geographic_coord_pose, last_after_ekf_timestamp,
            std::static_pointer_cast<MT::RVTargetInfo>(rv_target_opt.value().second), 0);
        if (!geographic_yaw_opt.has_value()) {
            // inclined = default_inclined;
        } else {
            inclined = angles::shortest_angular_distance(geographic_yaw_opt.value(), gimbal_yaw);
        }
    }

    HighPrecisionPoseSolver::Contour points{
        input->image_coord_armor_points.l_bottom, input->image_coord_armor_points.l_top,
        input->image_coord_armor_points.r_top, input->image_coord_armor_points.r_bottom};  // 改坐标转�??

    // std::cout << inclined << std::endl;

    auto o = hpps_.Run(input->geographic_coord_pose, gimbal_yaw, gimbal_pitch, gimbal_roll, points, input->armor_size,
                       gimbal_yaw, input->armor_id == RM::ArmorId::OUTPOST && !input->is_dart_armor,
                       use_secondary_camera, inclined);
    if (o.has_value()) {
        input->geographic_coord_yaw = o.value();

        // v_1.Send((float)angles::to_degrees(o.value()), inclined);
        //    v_1.Send((float) angles::to_degrees(yaw1), (float)angles::to_degrees(yaw2));
    }
    //    std::cout << input->geographic_coord_pose << " " << input->geographic_coord_yaw << std::endl;
}

cv::Point3d PoseSolver::PreciseArmorCoordTransform(
    std::shared_ptr<FP::ArmorInfo>& input,
    const MCU::Orders& input_order_data,
    const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_last_targets,
    rclcpp::Time last_after_ekf_timestamp) {
    auto gimbal_yaw = motor_yaw;
    auto gimbal_pitch = motor_pitch;
    auto imu_yaw = imu_data.imu_yaw;
    auto imu_pitch = imu_data.imu_pitch;
    auto imu_roll = imu_data.imu_roll;

    cv::Mat T_yaw_to_chassis = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_pitch_to_yaw = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_camera_to_pitch = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_camera_to_chassis = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat T_model_to_camera = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_model_to_chassis = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat R_model_to_camera;
    cv::Rodrigues(input->pnp_r_vec, R_model_to_camera);

    R_model_to_camera.copyTo(T_model_to_camera(cv::Rect(0, 0, 3, 3)));
    input->pnp_t_vec.copyTo(T_model_to_camera(cv::Rect(3, 0, 1, 3)));

    cv::Mat t_yaw_to_chassis = (cv::Mat_<double>(3, 1) << param_.gimbal_yaw_to_geographic_x,
                                param_.gimbal_yaw_to_geographic_y, param_.gimbal_yaw_to_geographic_z);
    cv::Mat R_yaw_to_chassis = (cv::Mat_<double>(3, 3) << cos(-gimbal_yaw), sin(-gimbal_yaw), 0, -sin(-gimbal_yaw),
                                cos(-gimbal_yaw), 0, 0, 0, 1);
    R_yaw_to_chassis.copyTo(T_yaw_to_chassis(cv::Rect(0, 0, 3, 3)));
    t_yaw_to_chassis.copyTo(T_yaw_to_chassis(cv::Rect(3, 0, 1, 3)));

    cv::Mat t_pitch_to_yaw = (cv::Mat_<double>(3, 1) << param_.gimbal_pitch_to_gimbal_yaw_x,
                              param_.gimbal_pitch_to_gimbal_yaw_y, param_.gimbal_pitch_to_gimbal_yaw_z);
    cv::Mat R_pitch_to_yaw = (cv::Mat_<double>(3, 3) << cos(-gimbal_pitch), 0, -sin(-gimbal_pitch), 0, 1, 0,
                              sin(-gimbal_pitch), 0, cos(-gimbal_pitch));
    R_pitch_to_yaw.copyTo(T_pitch_to_yaw(cv::Rect(0, 0, 3, 3)));
    t_pitch_to_yaw.copyTo(T_pitch_to_yaw(cv::Rect(3, 0, 1, 3)));

    bool use_secondary_camera =
        input_order_data.special_order == MCU::AutoaimReceiveInfo::SpecialOrder::USE_SECONDARY_CAMERA;

    cv::Mat t_camera_to_pitch;
    if (use_secondary_camera) {
        t_camera_to_pitch = (cv::Mat_<double>(3, 1) << param_.secondary_camera_to_gimbal_pitch_x,
                             param_.secondary_camera_to_gimbal_pitch_y, param_.secondary_camera_to_gimbal_pitch_z);
    } else {
        t_camera_to_pitch = (cv::Mat_<double>(3, 1) << param_.camera_to_gimbal_pitch_x, param_.camera_to_gimbal_pitch_y,
                             param_.camera_to_gimbal_pitch_z);
    }

    cv::Mat R_camera_to_pitch;

    R_camera_to_pitch = (cv::Mat_<double>(3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0);
    t_camera_to_pitch.copyTo(T_camera_to_pitch(cv::Rect(3, 0, 1, 3)));

    cv::Mat R_camera_to_pitch_install_pitch_offset;
    if (use_secondary_camera) {
        R_camera_to_pitch_install_pitch_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset), 0,
             -sin(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset), 0, 1, 0,
             sin(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset), 0,
             cos(-param_.secondary_camera_to_gimbal_pitch_install_pitch_offset));
    } else {
        R_camera_to_pitch_install_pitch_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0,
             -sin(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0, 1, 0,
             sin(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0,
             cos(-param_.camera_to_gimbal_pitch_install_pitch_offset));
    }

    cv::Mat R_camera_to_pitch_install_yaw_offset;
    if (use_secondary_camera) {
        R_camera_to_pitch_install_yaw_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset),
             sin(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset), 0,
             -sin(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset),
             cos(-param_.secondary_camera_to_gimbal_pitch_install_yaw_offset), 0, 0, 0, 1);
    } else {
        R_camera_to_pitch_install_yaw_offset =
            (cv::Mat_<double>(3, 3) << cos(-param_.camera_to_gimbal_pitch_install_yaw_offset),
             sin(-param_.camera_to_gimbal_pitch_install_yaw_offset), 0,
             -sin(-param_.camera_to_gimbal_pitch_install_yaw_offset),
             cos(-param_.camera_to_gimbal_pitch_install_yaw_offset), 0, 0, 0, 1);
    }

    R_camera_to_pitch =
        R_camera_to_pitch_install_yaw_offset * R_camera_to_pitch_install_pitch_offset * R_camera_to_pitch;
    R_camera_to_pitch.copyTo(T_camera_to_pitch(cv::Rect(0, 0, 3, 3)));

    T_camera_to_chassis = T_yaw_to_chassis * T_pitch_to_yaw * T_camera_to_pitch;
    T_model_to_chassis = T_camera_to_chassis * T_model_to_camera;
    //    cv::Mat T_geographic_to_model = T_model_to_chassis.inv();

    cv::Mat T_chassis_to_geographic = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat R_pitch_to_geographic = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat R_imu_roll = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat R_imu_pitch = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat R_imu_yaw = cv::Mat::eye(3, 3, CV_64F);

    R_imu_yaw = (cv::Mat_<double>(3, 3) << cos(-imu_yaw), sin(-imu_yaw), 0, -sin(-imu_yaw), cos(-imu_yaw), 0, 0, 0, 1);
    R_imu_pitch =
        (cv::Mat_<double>(3, 3) << cos(-imu_pitch), 0, -sin(-imu_pitch), 0, 1, 0, sin(-imu_pitch), 0, cos(-imu_pitch));
    R_imu_roll =
        (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(-imu_roll), -sin(-imu_roll), 0, sin(-imu_roll), cos(-imu_roll));

    R_pitch_to_geographic = R_imu_yaw * R_imu_pitch * R_imu_roll;

    cv::Mat R_chassis_to_geographic = R_pitch_to_geographic * R_pitch_to_yaw.inv() * R_yaw_to_chassis.inv();

    R_chassis_to_geographic.copyTo(T_chassis_to_geographic(cv::Rect(0, 0, 3, 3)));

    cv::Mat T_model_to_geographic = T_chassis_to_geographic * T_model_to_chassis;
    cv::Mat T_geographic_to_model = T_model_to_geographic.inv();

    cv::Mat p = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
    p = T_model_to_geographic * p;

    input->geographic_coord_pose = cv::Point3d{p.at<double>(0), p.at<double>(1), p.at<double>(2)};
    auto& v = vpie::VofaBridge::Get();
    v.SendOnce(
        (float)angles::to_degrees(atan2(input->geographic_coord_pose.y, input->geographic_coord_pose.x)),
        (float)angles::to_degrees(atan2(input->geographic_coord_pose.z,
                                        GetDistance(input->geographic_coord_pose.x,
                                        input->geographic_coord_pose.y))), 
        (float)angles::to_degrees(gimbal_yaw), 
        (float)angles::to_degrees(gimbal_pitch), 
        (float)angles::to_degrees(imu_yaw), 
        (float)angles::to_degrees(imu_pitch));

    static double inclined = 0;
    auto rv_target_opt = ArmorComparator::FindMatchRVTarget(input_last_targets, input->armor_id, input->armor_size);
    if (!rv_target_opt.has_value()) {
    } else {
        auto geographic_yaw_opt = ArmorComparator::GetGeographicYaw(
            input->geographic_coord_pose, last_after_ekf_timestamp,
            std::static_pointer_cast<MT::RVTargetInfo>(rv_target_opt.value().second), 0);
        if (!geographic_yaw_opt.has_value()) {
        } else {
            inclined = angles::shortest_angular_distance(geographic_yaw_opt.value(), imu_yaw);
        }
    }

    HighPrecisionPoseSolver::Contour points{
        input->image_coord_armor_points.l_bottom, input->image_coord_armor_points.l_top,
        input->image_coord_armor_points.r_top, input->image_coord_armor_points.r_bottom};

    auto o = hppsk_.Run(input->geographic_coord_pose, imu_yaw, imu_pitch, imu_roll, gimbal_yaw, gimbal_pitch, points,
                        input->armor_size, imu_yaw, input->armor_id == RM::ArmorId::OUTPOST && !input->is_dart_armor,
                        use_secondary_camera, inclined);
    if (o.has_value()) {
        input->geographic_coord_yaw = o.value();
    }

    cv::Mat T_boost_to_geographic = T_chassis_to_geographic * T_yaw_to_chassis * T_pitch_to_yaw;
    cv::Mat boost_ori_in_geographic = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
    boost_ori_in_geographic = T_boost_to_geographic * boost_ori_in_geographic;
    // std::cout << T_pitch_to_geographic << std::endl;

    return cv::Point3d{boost_ori_in_geographic.at<double>(0), boost_ori_in_geographic.at<double>(1),
                       boost_ori_in_geographic.at<double>(2)};
}

void PoseSolver::PowerRuneCoordTransform(std::shared_ptr<FP::FanInfo>& input_fan, const MCU::Orders& input_order_data) {
    auto gimbal_yaw = imu_data.imu_yaw;
    auto gimbal_pitch = imu_data.imu_pitch;

    cv::Mat T_gimbal_yaw_to_geographic = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_gimbal_pitch_to_gimbal_yaw = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_camera_to_gimbal_pitch = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_camera_to_geographic = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat T_model_to_camera = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_model_to_geographic = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat R_model_to_camera;
    cv::Rodrigues(input_fan->surface_points_inter_bull_pnp_r_vec, R_model_to_camera);

    R_model_to_camera.copyTo(T_model_to_camera(cv::Rect(0, 0, 3, 3)));
    input_fan->surface_points_inter_bull_pnp_t_vec.copyTo(T_model_to_camera(cv::Rect(3, 0, 1, 3)));

    cv::Mat t_gimbal_yaw_to_geographic = (cv::Mat_<double>(3, 1) << param_.gimbal_yaw_to_geographic_x,
                                          param_.gimbal_yaw_to_geographic_y, param_.gimbal_yaw_to_geographic_z);
    cv::Mat R_gimbal_yaw_to_geographic = (cv::Mat_<double>(3, 3) << cos(-gimbal_yaw), sin(-gimbal_yaw), 0,
                                          -sin(-gimbal_yaw), cos(-gimbal_yaw), 0, 0, 0, 1);
    R_gimbal_yaw_to_geographic.copyTo(T_gimbal_yaw_to_geographic(cv::Rect(0, 0, 3, 3)));
    t_gimbal_yaw_to_geographic.copyTo(T_gimbal_yaw_to_geographic(cv::Rect(3, 0, 1, 3)));

    cv::Mat t_gimbal_pitch_to_gimbal_yaw = (cv::Mat_<double>(3, 1) << param_.gimbal_pitch_to_gimbal_yaw_x,
                                            param_.gimbal_pitch_to_gimbal_yaw_y, param_.gimbal_pitch_to_gimbal_yaw_z);
    cv::Mat R_gimbal_pitch_to_gimbal_yaw = (cv::Mat_<double>(3, 3) << cos(-gimbal_pitch), 0, -sin(-gimbal_pitch), 0, 1,
                                            0, sin(-gimbal_pitch), 0, cos(-gimbal_pitch));
    R_gimbal_pitch_to_gimbal_yaw.copyTo(T_gimbal_pitch_to_gimbal_yaw(cv::Rect(0, 0, 3, 3)));
    t_gimbal_pitch_to_gimbal_yaw.copyTo(T_gimbal_pitch_to_gimbal_yaw(cv::Rect(3, 0, 1, 3)));

    cv::Mat t_camera_to_gimbal_pitch = (cv::Mat_<double>(3, 1) << param_.camera_to_gimbal_pitch_x,
                                        param_.camera_to_gimbal_pitch_y, param_.camera_to_gimbal_pitch_z);
    cv::Mat R_camera_to_gimbal_pitch = (cv::Mat_<double>(3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0);

    cv::Mat R_camera_to_pitch_install_pitch_offset =
        (cv::Mat_<double>(3, 3) << cos(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0,
         -sin(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0, 1, 0,
         sin(-param_.camera_to_gimbal_pitch_install_pitch_offset), 0,
         cos(-param_.camera_to_gimbal_pitch_install_pitch_offset));
    cv::Mat R_camera_to_pitch_install_yaw_offset =
        (cv::Mat_<double>(3, 3) << cos(-param_.camera_to_gimbal_pitch_install_yaw_offset),
         sin(-param_.camera_to_gimbal_pitch_install_yaw_offset), 0,
         -sin(-param_.camera_to_gimbal_pitch_install_yaw_offset),
         cos(-param_.camera_to_gimbal_pitch_install_yaw_offset), 0, 0, 0, 1);

    R_camera_to_gimbal_pitch =
        R_camera_to_pitch_install_yaw_offset * R_camera_to_pitch_install_pitch_offset * R_camera_to_gimbal_pitch;

    R_camera_to_gimbal_pitch.copyTo(T_camera_to_gimbal_pitch(cv::Rect(0, 0, 3, 3)));
    t_camera_to_gimbal_pitch.copyTo(T_camera_to_gimbal_pitch(cv::Rect(3, 0, 1, 3)));

    T_camera_to_geographic = T_gimbal_yaw_to_geographic * T_gimbal_pitch_to_gimbal_yaw * T_camera_to_gimbal_pitch;
    T_model_to_geographic = T_camera_to_geographic * T_model_to_camera;
    cv::Mat R_module_to_geographic = T_model_to_geographic(cv::Rect(0, 0, 3, 3));

    cv::Mat module_point = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);

    cv::Mat camera_point = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
    module_point = T_model_to_geographic * module_point;
    camera_point = T_camera_to_geographic * camera_point;

    cv::Mat module_normal_unit_vec = (cv::Mat_<double>(3, 1) << 1, 0, 0);
    module_normal_unit_vec = R_module_to_geographic * module_normal_unit_vec;

    input_fan->geographic_coord_light_surface_points_infer_bull_pose =
        cv::Point3d{module_point.at<double>(0), module_point.at<double>(1), module_point.at<double>(2)};

    input_fan->geographic_coord_camera_pose = cv::Point3d{
        camera_point.at<double>(0),
        camera_point.at<double>(1),
        camera_point.at<double>(2),
    };
    camera_module_solver_.SolvePinHole(input_fan);

    auto geographic_coord_camera_to_r_symbol_unit_vec_yaw =
        angles::normalize_angle(gimbal_yaw + input_fan->right_handed_camera_coord_r_symbol_phi);
    auto geographic_coord_camera_to_r_symbol_unit_vec_pitch = angles::normalize_angle(
        gimbal_pitch + (angles::from_degrees(90) - input_fan->right_handed_camera_coord_r_symbol_theta));

    geographic_coord_camera_to_r_symbol_unit_vec_yaw += param_.camera_to_gimbal_pitch_install_yaw_offset;
    geographic_coord_camera_to_r_symbol_unit_vec_pitch += param_.camera_to_gimbal_pitch_install_pitch_offset;

    cv::Mat R_vec_yaw_to_geographic = (cv::Mat_<double>(3, 3) << cos(-geographic_coord_camera_to_r_symbol_unit_vec_yaw),
                                       sin(-geographic_coord_camera_to_r_symbol_unit_vec_yaw), 0,
                                       -sin(-geographic_coord_camera_to_r_symbol_unit_vec_yaw),
                                       cos(-geographic_coord_camera_to_r_symbol_unit_vec_yaw), 0, 0, 0, 1);
    cv::Mat R_vec_pitch_to_yaw = (cv::Mat_<double>(3, 3) << cos(-geographic_coord_camera_to_r_symbol_unit_vec_pitch), 0,
                                  -sin(-geographic_coord_camera_to_r_symbol_unit_vec_pitch), 0, 1, 0,
                                  sin(-geographic_coord_camera_to_r_symbol_unit_vec_pitch), 0,
                                  cos(-geographic_coord_camera_to_r_symbol_unit_vec_pitch));

    cv::Mat unit_vec_point = (cv::Mat_<double>(3, 1) << 1, 0, 0);
    unit_vec_point = R_vec_yaw_to_geographic * R_vec_pitch_to_yaw * unit_vec_point;
    //    std::cout << unit_vec_point << std::endl;

    input_fan->geographic_coord_camera_to_r_symbol_unit_vec =
        cv::Point3d{unit_vec_point.at<double>(0), unit_vec_point.at<double>(1), unit_vec_point.at<double>(2)};

    {
        // 球面交线实验———————�?
        cv::Point3d centre(input_fan->geographic_coord_light_surface_points_infer_bull_pose / 2);
        double r = norm((input_fan->geographic_coord_light_surface_points_infer_bull_pose / 2));
        cv::Point3d o(input_fan->geographic_coord_camera_pose);
        cv::Point3d e(input_fan->geographic_coord_camera_to_r_symbol_unit_vec * 10 +
                      input_fan->geographic_coord_camera_pose);

        // cv::Point3d selected_point;
        // auto intersect_points = LineInterSectSphereSolver::Run(o, e, centre, r);
        // if (intersect_points.empty()) {
        //     std::cerr << "fp: line intersect sphere solver has 0 result, invalid status" << std::endl;
        //     std::terminate();
        // } else if (intersect_points.size() == 2) {
        //     selected_point = std::max(intersect_points.at(0), intersect_points.at(1),
        //                               [](auto& a, auto& b) { return cv::norm(b) > cv::norm(a); });
        // } else {  // size() == 1
        //     selected_point = intersect_points.at(0);
        // }

        // input_fan->geographic_coord_r_symbol_pose = selected_point;

        // dh.ShowInGeographicCoord(selected_point, gimbal_yaw, gimbal_pitch, "r point");
        // std::cout << norm(input_fan->geographic_coord_r_symbol_pose) << std::endl;

        // 平面交线实验———————�? 0122
        cv::Point3d power_rune_plane_normal(module_normal_unit_vec);
        cv::Point3d power_rune_plane_points(input_fan->geographic_coord_light_surface_points_infer_bull_pose);
        cv::Point3d line_points(input_fan->geographic_coord_camera_pose);
        cv::Point3d line_unit_vec(input_fan->geographic_coord_camera_to_r_symbol_unit_vec);
        double d = ((power_rune_plane_points - line_points).dot(power_rune_plane_normal)) /
                   (line_unit_vec.dot(power_rune_plane_normal));
        auto test_r_symbol_pose = d * line_unit_vec + line_points;

        input_fan->geographic_coord_r_symbol_pose = test_r_symbol_pose;

        // dh.ShowInGeographicCoord(input_fan->geographic_coord_light_surface_points_infer_bull_pose, gimbal_yaw,
        // gimbal_pitch, "bull");
        // std::cout << norm(input_fan->geographic_coord_r_symbol_pose -
        //                   input_fan->geographic_coord_light_surface_points_infer_bull_pose)
        //           << std::endl;
        // dh.ShowTwoPoint3d(input_fan->geographic_coord_light_surface_points_infer_bull_pose,
        // input_fan->geographic_coord_r_symbol_pose);

        // cv::Point2d horizon_bull(input_fan->geographic_coord_light_surface_points_infer_bull_pose.x,
        //                          input_fan->geographic_coord_light_surface_points_infer_bull_pose.y);

        // cv::Point2d horizon_r_symbol(input_fan->geographic_coord_r_symbol_pose.x,
        //                              input_fan->geographic_coord_r_symbol_pose.y);

        // std::cout << cv::norm(horizon_bull) << " " << cv::norm(horizon_r_symbol) << std::endl;
    }
}

std::optional<std::pair<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>> ArmorComparator::FindMatchRVTarget(
    const std::map<RM::TargetClassify, std::shared_ptr<MT::ITargetInfo>>& input_last_targets,
    RM::ArmorId this_armor_id,
    RM::ArmorSize this_armor_size) {
    auto find_ptr = std::find_if(input_last_targets.begin(), input_last_targets.end(),
                                 [&this_armor_id, &this_armor_size](auto& input_target_pair) {
                                     if (input_target_pair.first == RM::TargetClassify::BASE_BOTTOM ||
                                         input_target_pair.first == RM::TargetClassify::BASE_DART ||
                                         input_target_pair.first == RM::TargetClassify::BASE_TOP ||
                                         input_target_pair.first == RM::TargetClassify::OUTPOST_DART ||
                                         input_target_pair.first == RM::TargetClassify::POWER_RUNE) {
                                         return false;
                                     } else {
                                         auto rv_target =
                                             std::static_pointer_cast<MT::RVTargetInfo>(input_target_pair.second);
                                         return this_armor_id == rv_target->armor_id &&
                                                this_armor_size == rv_target->armor_size && rv_target->target_available;
                                     }
                                 });
    if (find_ptr == input_last_targets.end())
        return std::nullopt;

    return (*find_ptr);
}

double ArmorComparator::GetAbsBetweenArmorOffset(RM::TargetClassify target_classify) {
    if (target_classify == RM::TargetClassify::OUTPOST_SPIN) {
        return angles::from_degrees(120);
    } else if (target_classify == RM::TargetClassify::HERO_1 || target_classify == RM::TargetClassify::ENGINEER_2 ||
               target_classify == RM::TargetClassify::STANDARD_3 || target_classify == RM::TargetClassify::STANDARD_4 ||
               target_classify == RM::TargetClassify::STANDARD_5 || target_classify == RM::TargetClassify::SENTRY) {
        return angles::from_degrees(90);
    } else {
        std::terminate();
    }
}

bool ArmorComparator::AOnLeftOfB(cv::Point3d a, cv::Point3d b) {
    auto a_yaw = GetYaw(a.x, a.y);
    auto b_yaw = GetYaw(b.x, b.y);
    auto angle_dev = angles::shortest_angular_distance(a_yaw, b_yaw);
    return angle_dev <= 0;
}

std::optional<double> ArmorComparator::GetGeographicYaw(cv::Point3d geographic_armor_pose,
                                                        rclcpp::Time last_time_points,
                                                        const std::shared_ptr<MT::RVTargetInfo>& last_target,
                                                        double between_armor_offset [[maybe_unused]]) {
    const double max_match_distance = 0.3;
    auto rv_target = std::static_pointer_cast<MT::RVTargetInfo>(last_target);
    double duration_time = (rclcpp::Clock().now() - last_time_points).seconds();

    auto speed_vec = rv_target->c_velocity;
    auto pose = rv_target->c_position;
    auto this_pose = pose + speed_vec * duration_time;

    auto yaw = rv_target->yaw;
    auto v_yaw = rv_target->v_yaw;
    auto this_yaw = yaw + v_yaw * duration_time;

    auto r = rv_target->r_1;

    auto this_armor_pose =
        cv::Point3d(this_pose.x - std::cos(this_yaw) * r, this_pose.y - std::sin(this_yaw) * r, this_pose.z);

    if (cv::norm(geographic_armor_pose - this_armor_pose) <= max_match_distance)
        return this_yaw;
    else
        return std::nullopt;
}