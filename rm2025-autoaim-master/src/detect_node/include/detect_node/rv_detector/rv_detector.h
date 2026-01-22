// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#pragma once

// STD
#include <memory>
#include <string>
#include <vector>
#include "detect_node/interface/i_detector.h"

#include "param_loader.hpp"
#include "rv_detector_core.h"
#include "rv_number_classifier.h"

namespace FP::RVD {

class RVDetector : public IDetector {
   public:
    RVDetector() = delete;

    explicit RVDetector(RM::Color input_color);

    std::vector<std::shared_ptr<IFeature>> Run(const cv::Mat& img_msg,
                                               const MCU::Orders& input_order_data,
                                               cv::Mat& result_img) final;

   private:
    std::unique_ptr<RVDetectorCore> InitDetector(RM::Color input_color);

    std::vector<RVDArmor> DetectArmors(const cv::Mat& img_msg);

    // Armor Detector
    std::unique_ptr<RVDetectorCore> rv_detector_core_;
    std::chrono::steady_clock::time_point last_time_;

    struct Param {
        // int binary_threshold = ParamLoader::FP::RVD::LIGHT_BINARY_THRESHOLD;

        int binary_threshold = ParamLoader::GetInstance().GetParam<int>("FP", "RVD", "LIGHT_BINARY_THRESHOLD");
        double light_min_ratio = ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "LIGHT_MIN_RATIO");
        double light_max_ratio = ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "LIGHT_MAX_RATIO");
        double light_max_angle = ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "LIGHT_MAX_ANGLE");
        double armor_min_light_ratio =
            ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "ARMOR_MIN_LIGHT_RATIO");
        double armor_min_small_center_distance =
            ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "SMALL_ARMOR_MIN_CENTRE_DISTANCE");
        double armor_max_small_center_distance =
            ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "SMALL_ARMOR_MAX_CENTRE_DISTANCE");
        double armor_min_large_center_distance =
            ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "LARGE_ARMOR_MIN_CENTRE_DISTANCE");
        double armor_max_large_center_distance =
            ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "LARGE_ARMOR_MAX_CENTRE_DISTANCE");
        double armor_max_angle_degree =
            ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "ARMOR_MAX_ANGLE_DEGREE");
        double classifier_threshold =
            ParamLoader::GetInstance().GetParam<double>("FP", "RVD", "CLASSIFIER_CONFIDENCE_THRESHOLD");
        bool debug_mode = ParamLoader::GetInstance().GetParam<bool>("FP", "RVD", "ENABLE_DEBUG_SHOW");
        std::vector<int> ignore_classes;
        std::string module_path = ParamLoader::GetInstance().GetParam<std::string>("FP", "RVD", "MODULE_PATH");
    } param_;
};
}  // namespace FP::RVD
