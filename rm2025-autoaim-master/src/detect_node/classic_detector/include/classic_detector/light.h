#pragma once

#include <opencv2/core/types.hpp>
#include "common.h"

namespace vpie {
struct Light : public cv::RotatedRect {
    Light() = default;
    explicit Light(const std::vector<cv::Point>& contour);
    EnemyColor color{};
    cv::Point2f top, bottom, center;
    double length{};
    double width{};
    float tilt_angle{};
    std::vector<cv::Point> contour_{};
};
}  // namespace vpie