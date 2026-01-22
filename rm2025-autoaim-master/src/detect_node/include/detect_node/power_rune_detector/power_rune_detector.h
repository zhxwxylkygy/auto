//
// Created by wpie on 24-1-9.
//
#pragma once

// STD
#include <memory>
#include <opencv2/core/types.hpp>
#include <optional>
#include <string>
#include <vector>
#include "power_rune_yolov11.h"
#include "detect_node/interface/i_detector.h"

#include "param_loader.hpp"

#include "detect_node/interface/mcu.h"

namespace FP::PD {

class PowerRuneDetector : public IDetector {
   public:
    PowerRuneDetector() = delete;

    explicit PowerRuneDetector(RM::Color input_color);

    std::vector<std::shared_ptr<IFeature>> Run(const cv::Mat& input_image, const MCU::Orders& input_order_data, cv::Mat& result_img) final;

   private:
    std::optional<cv::Point> DetectRSymbol(const FP::FanInfo& input_fan_info,
                                           const cv::Mat& input_image,
                                           bool enable_debug_image);

    std::chrono::steady_clock::time_point last_time_;

    std::unique_ptr<PowerRuneYOLOv11> power_rune_yolo_v11_;

    struct Param {
        const float confidence_threshold = ParamLoader::GetInstance().GetParam<float>("FP","PD","CONFIDENCE_THRESHOLD");
        const float nms_threshold = ParamLoader::GetInstance().GetParam<float>("FP","PD","NMS_THRESHOLD");
        const float score_threshold = ParamLoader::GetInstance().GetParam<float>("FP","PD","SCORE_THRESHOLD");
        const int input_width = ParamLoader::GetInstance().GetParam<int>("FP","PD","INPUT_WIDTH");
        const int input_height = ParamLoader::GetInstance().GetParam<int>("FP","PD","INPUT_HEIGHT");
        const std::string onnx_path = ParamLoader::GetInstance().GetParam<std::string>("FP","PD","ONNX_PATH");
        const bool enable_debug_image = ParamLoader::GetInstance().GetParam<bool>("FP","PD","ENABLE_DEBUG_SHOW");
        const double to_centre_vec_coefficient =
            ParamLoader::GetInstance().GetParam<double>("FP","PD","TO_CENTRE_VEC_COEFFICIENT");
        const int r_symbol_detect_roi_half_length = ParamLoader::GetInstance().GetParam<int>("FP","PD","R_SYMBOL_DETECT_ROI_HALF_LENGTH");
        const int gray_img_threshold_value = ParamLoader::GetInstance().GetParam<int>("FP","PD","GRAY_IMAGE_THRESHOLD_VALUE");
        const int contour_area_threshold_min = ParamLoader::GetInstance().GetParam<int>("FP","PD","CONTOUR_AREA_THRESHOLD_MIN");
        const int contour_area_threshold_max = ParamLoader::GetInstance().GetParam<int>("FP","PD","CONTOUR_AREA_THRESHOLD_MAX");

        // const int dilate_rect_size;
        // const int max_area_threshold;
        // const int first_area_threshold_min_area;
        // const int second_area_threshold_max_area;
        // const int second_area_threshold_min_area;
        // todo add some param
    } param_;
};
}  // namespace FP::PD
