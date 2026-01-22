#pragma once

// opencv
#include <opencv2/opencv.hpp>
// project
#include <classic_detector/armor.h>

namespace vpie {

struct SymmetryAxis {
    cv::Point2f centroid;
    cv::Point2f direction;
    float mean_val;  // Mean brightness
};

// This class is used to improve the precision of the corner points of the light bar.
// First, the PCA algorithm is used to find the symmetry axis of the light bar,
// and then along the symmetry axis to find the corner points of the light bar based on the gradient of brightness.
class LightCornerCorrector {
   public:
    explicit LightCornerCorrector() noexcept = default;

    void correctCorners(Armor& armor, const cv::Mat& gray_img);

    SymmetryAxis findSymmetryAxis(const cv::Mat& gray_img, const Light& light);

    cv::Point2f findCorner(const cv::Mat& gray_img, const Light& light, const SymmetryAxis& axis, std::string order);
};

}  // namespace vpie
