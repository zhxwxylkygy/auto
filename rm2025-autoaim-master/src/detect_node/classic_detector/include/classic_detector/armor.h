#pragma once

#include <fmt/format.h>
#include <eigen3/Eigen/Core>
#include <opencv2/core/mat.hpp>
#include "classic_detector/common.h"
#include "light.h"

namespace vpie {
struct Armor {
    struct ClassifyResult {
        double confidence;
        std::string number;
        std::string classfication_result;

        ClassifyResult(double confidence_, std::string_view number_)
            : confidence(confidence_),
              number(number_),
              classfication_result(fmt::format("{}: {:.1f}%", number_, confidence_ * 100.0)) {}
        ClassifyResult() = default;

        bool Is(std::string_view v) const { return number == v; }
        bool As(std::string_view v) const { return Is(v) || number == "negative"; }
    };

    static constexpr const int N_LANDMARKS = 6;
    static constexpr const int N_LANDMARKS_2 = N_LANDMARKS * 2;
    Armor() = default;
    Armor(const Light& l1, const Light& l2);

    // Build the points in the object coordinate system, start from bottom left in
    // clockwise order
    template <typename PointType>
    static std::vector<PointType> buildObjectPoints(const double& w, const double& h);

    void SetClassifyResult(const ClassifyResult&);

    // Landmarks start from bottom left in clockwise order
    std::vector<cv::Point2f> landmarks() const;

    // Armor pose part
    cv::Mat rmat;
    cv::Mat tvec;
    double roll{};
    Eigen::Matrix3d imu2camera;

    // Light pairs part
    Light left_light, right_light;
    cv::Point2f center;
    ArmorType type{ArmorType::INVALID};

    // Number part
    mutable cv::Mat original_img;
    cv::Mat number_img;
    std::string number;
    double confidence{};
    std::string classfication_result;
    bool is_corrected{};

    bool valid{true};
};
}  // namespace vpie