// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>
#include "detect_node/interface/interface.h"
#include "detect_node/interface/rm_interface.h"
#include "param_loader.hpp"
#include "detect_node/rv_detector/rv_number_classifier.h"


namespace FP::RVD {
class RVDetectorCore {
   public:
    struct LightParams {
        // width / height
        double min_ratio;
        double max_ratio;
        // vertical angle
        double max_angle;
    };

    struct ArmorParams {
        double min_light_ratio;
        // light pairs distance
        double min_small_center_distance;
        double max_small_center_distance;
        double min_large_center_distance;
        double max_large_center_distance;
        // horizontal angle
        double max_angle;
    };

    RVDetectorCore(const int& bin_threshold, const RM::Color& color, const LightParams& l, const ArmorParams& a);

    std::vector<RVDArmor> Detect(const cv::Mat& input);

    cv::Mat PreProcessImage(const cv::Mat& input);

    std::vector<Light> FindLights(const cv::Mat& input_image, const cv::Mat& binary_img);

    std::vector<RVDArmor> MatchLights(const std::vector<Light>& lights);

    // For debug usage
    cv::Mat getAllNumbersImage();

    void DrawResults(cv::Mat& img);

    int binary_threshold_;
    LightParams l_;
    ArmorParams a_;

    std::unique_ptr<RVNumberClassifier> classifier_;

    cv::Mat binary_img_;

   private:
    bool IsLight(const Light& possible_light);

    bool ContainLight(const Light& light_1, const Light& light_2, const std::vector<Light>& lights);

    RM::ArmorSize IsArmor(const Light& light_1, const Light& light_2);

    std::vector<Light> lights_;
    std::vector<RVDArmor> armors_;

    struct Param {
        const bool debug = ParamLoader::GetInstance().GetParam<bool>("FP","RVD","ENABLE_DEBUG_SHOW");
        const int contour_area_threshold = ParamLoader::GetInstance().GetParam<int>("FP","RVD","CONTOUR_AREA_THRESHOLD");
    } param_;

    RM::Color target_color_;
};

}  // namespace FP::RVD

#endif