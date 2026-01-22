//
// Created by wpie on 23-11-1.
//

#pragma once

#include <opencv2/core/types.hpp>
#include <optional>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <variant>
#include <opencv2/opencv.hpp>
#include "rm_interface.h"
// #include <calculate/vector3.h>

namespace FP {  // frame processor

struct IFeature {
    virtual ~IFeature() = default;
    std_msgs::msg::Header header;
};

struct ArmorInfo : IFeature {
    struct ArmorPoints {
        cv::Point l_bottom;
        cv::Point l_top;
        cv::Point r_top;
        cv::Point r_bottom;
    };
    cv::Rect roi_rect{};

    ArmorPoints image_coord_armor_points;

    RM::ArmorId armor_id{};
    RM::ArmorSize armor_size{};

    cv::Point3d geographic_coord_pose{};
    double geographic_coord_yaw{};

    cv::Mat pnp_r_vec;
    cv::Mat pnp_t_vec;

    bool is_dart_armor{};  // 飞镖装甲板判定
};

struct FanInfo : IFeature {
    struct LightsSurfacePoints {
        cv::Point top;
        cv::Point bottom;
        cv::Point left;
        cv::Point right;
        cv::Point center;
    };
    LightsSurfacePoints image_coord_lights_surface_points;

    std::optional<cv::Point> image_coord_bull_point;
    cv::Point image_coord_r_symbol_point;

    cv::Point3d geographic_coord_camera_to_r_symbol_unit_vec;

    cv::Point3d geographic_coord_light_surface_points_infer_bull_pose;
    cv::Point3d geographic_coord_r_symbol_pose;
    bool is_target_fan;

    cv::Point3d geographic_coord_camera_pose;

    //        std::optional<double> geographic_coord_bull_theta;
    //        std::optional<double> geographic_coord_bull_phi;

    cv::Mat surface_points_inter_bull_pnp_r_vec;
    cv::Mat surface_points_inter_bull_pnp_t_vec;
    double right_handed_camera_coord_r_symbol_theta;
    double right_handed_camera_coord_r_symbol_phi;

    // todo 目前没有作使用靶心坐标在pnp灯板平面作矫正的设计 后续如有需要添加 也就是说目前识别的靶心点是无用的
};

namespace RVD {
struct Light : public cv::RotatedRect {
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box) {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;
        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);
        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = static_cast<float>(tilt_angle / CV_PI * 180);
    }

    RM::Color color{};
    cv::Point2f top{}, bottom{};
    double length{};
    double width{};
    float tilt_angle{};
};

struct RVDArmor {
    RVDArmor() = default;

    cv::Point2f center{};

    RVDArmor(const Light& l1, const Light& l2) {
        if (l1.center.x < l2.center.x) {
            left_light = l1, right_light = l2;
        } else {
            left_light = l2, right_light = l1;
        }
        center = (left_light.center + right_light.center) / 2;
    }

    // Light pairs part
    Light left_light{}, right_light{};

    // Number part
    cv::Mat number_img{};
    float confidence{};
    std::string classification_result{};
    RM::ArmorId number{};
    RM::ArmorSize type{};
};
}  // namespace RVD

}  // namespace FP
namespace MT {  // multichannel tracker

struct ITargetInfo {
    bool target_available = false;
    cv::Point3d target_ref_coord{};
    int kalman_score = 0;
    std_msgs::msg::Header header;

    [[nodiscard]] double GetMyTimestamp() const{
        return header.stamp.sec + (static_cast<double>(header.stamp.nanosec) / 1000000000.f);
    }

    ITargetInfo() = default;
    virtual ~ITargetInfo() = default;
};

struct RVTargetInfo : public ITargetInfo {
    RM::ArmorId armor_id{};
    RM::ArmorSize armor_size{};
    RM::TargetArmorNum target_armor_num{};
    std::vector<std::shared_ptr<FP::ArmorInfo>> armors_vec{};

    cv::Point3d c_position{};
    cv::Point3d c_velocity{};
    double yaw{};
    double v_yaw{};
    double r_1{};
    double r_2{};
    double d_z{};
};

inline std::ostream& operator<<(std::ostream& os, const RVTargetInfo& obj) {
    return os << "c_p: " << obj.c_position << " c_v: " << obj.c_velocity << " yaw: " << obj.yaw * 57.32
              << " v_yaw: " << obj.v_yaw * 57.32 << " r_1: " << obj.r_1 << " r_2: " << obj.r_2 << " d_z: " << obj.d_z;
}

struct StaticTargetInfo : ITargetInfo {
    RM::ArmorId armor_id{};
    RM::ArmorSize armor_size{};
    RM::TargetArmorNum target_armor_num{};
    std::vector<std::shared_ptr<FP::ArmorInfo>> armors_vec{};

    cv::Point3d c_position{};
    cv::Point3d c_velocity{};
};
struct PowerRuneTargetInfo : ITargetInfo {
    std::vector<std::shared_ptr<FP::FanInfo>> fans_vec{};

    struct PowerRune {
        bool is_clockwise{};
        double bull_roll{};
        double bull_abs_v_roll{};
        cv::Point3d geographic_coord_r_symbol_pose{};
    };
    struct LargePowerRune : public PowerRune {
        double func_a{};
        double func_omega{};
        double func_phi{};

        double radius{};
    };
    struct SmallPowerRune : public PowerRune {
        double radius{};
    };
    struct SSPR : public PowerRune {};
    struct SLPR : public PowerRune {
        double func_a{};
        double func_omega{};
        double func_phi{};

        double t_sum;
    };

    std::variant<std::monostate, SmallPowerRune, LargePowerRune, SSPR, SLPR> motion_info{};
    //        double bull_roll;
    //        double bull_v_roll;
    //        double radius;
    //        double func_a;
    //        double func_omega;
    //        double func_phi;
    //        cv::Point3d geographic_coord_r_symbol_pose;
};

namespace RVT {
//        struct Light : public cv::RotatedRect {
//            Light() = default;
//
//            explicit Light(cv::RotatedRect box) : cv::RotatedRect(box) {
//                cv::Point2f p[4];
//                box.points(p);
//                std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) {
//                    return a.y < b.y;
//                });
//                top = (p[0] + p[1]) / 2;
//                bottom = (p[2] + p[3]) / 2;
//
//                length = cv::norm(top - bottom);
//                width = cv::norm(p[0] - p[1]);
//
//                tilt_angle =
//                        std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
//                tilt_angle = tilt_angle / CV_PI * 180;
//            }
//
//            RM::Color color;
//            cv::Point2f top, bottom;
//            double length;
//            double width;
//            float tilt_angle;
//        };

//        struct RVTArmor {
//            // Number part
//            RM::ArmorId armor_id;
//            RM::ArmorSize type;
//            double yaw{};
//            tf2::Vector3 position{};
//            RM::TargetArmorNum target_armor_num;
//        };
}  // namespace RVT
}  // namespace MT

namespace ETS {  // ergo target_classify selector

}

namespace KS {  // kinematics solver
struct TargetSolution {
    double yaw;
    double pitch;
    bool enable_shoot;
};
}  // namespace KS
