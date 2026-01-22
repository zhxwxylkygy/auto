//
// Created by wpie on 24-1-9.
//
#pragma once

// STD
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include "detect_node/interface/i_detector.h"

#include "../rv_detector/rv_detector.h"
#include "../rv_detector/rv_number_classifier.h"
#include "detect_node/interface/interface.h"
#include "detect_node/interface/rm_interface.h"
#include "param_loader.hpp"
#include "pt4_yolo_v8k.h"
#include "yolo-pose.h"
namespace FP::PT4D {

class Pt4Detector : public IDetector {
   public:
    Pt4Detector() = delete;

    explicit Pt4Detector(RM::Color input_color);

    std::vector<std::shared_ptr<IFeature>> Run(const cv::Mat& input_image,
                                               const MCU::Orders& input_order_data,
                                               cv::Mat& result_img) final;

   private:
    RM::Color enemy_color_;

    static RM::ArmorSize JudgeArmorSizeByRatio(FP::ArmorInfo::ArmorPoints input_armor_points);

    static std::optional<RM::ArmorSize> JudgeArmorSizeByMorphology(FP::ArmorInfo::ArmorPoints input_armor_points,
                                                                   const cv::Mat& input_image);

    void ExcludeImageEdgeArmor(std::vector<std::shared_ptr<FP::ArmorInfo>>& input_armors);

    std::tuple<RM::ArmorId, double> GetArmorIdByDNN(FP::ArmorInfo::ArmorPoints& input_armor_points,
                                                    RM::ArmorSize input_armor_size,
                                                    const cv::Mat& input_image);

    struct Param {
        enum class YOLOVersion { V8, V5 };
        const YOLOVersion yolo_version = static_cast<YOLOVersion>(0);
        // todo: add to param loader
        const float confidence_threshold =
            ParamLoader::GetInstance().GetParam<float>("FP", "PT4D", "CONFIDENCE_THRESHOLD");
        const float nms_threshold = ParamLoader::GetInstance().GetParam<float>("FP", "PT4D", "NMS_THRESHOLD");
        const float score_threshold = ParamLoader::GetInstance().GetParam<float>("FP", "PT4D", "SCORE_THRESHOLD");
        const int input_width = ParamLoader::GetInstance().GetParam<int>("FP", "PT4D", "INPUT_WIDTH");
        const int input_height = ParamLoader::GetInstance().GetParam<int>("FP", "PT4D", "INPUT_HEIGHT");
        const std::string yolo_v5_onnx_path =
            ParamLoader::GetInstance().GetParam<std::string>("FP", "PT4D", "YOLO_V5_ONNX_PATH");
        const std::string yolo_v8_onnx_path =
            ParamLoader::GetInstance().GetParam<std::string>("FP", "PT4D", "YOLO_V8_ONNX_PATH");
        const std::string yolo_v8k_onnx_path =
            ParamLoader::GetInstance().GetParam<std::string>("FP", "PT4D", "YOLO_V8K_ONNX_PATH");
        const bool enable_debug_image = ParamLoader::GetInstance().GetParam<bool>("FP", "PT4D", "ENABLE_DEBUG_SHOW");
        const cv::Size image_size = ParamLoader::GetInstance().GetParam<cv::Size>("CAMERA", "IMAGE_SIZE");
        const int min_distance_to_image_edge = 10;
        const std::string classify_dnn_module_path = "./module/rv_dnn.onnx";
        const double classifier_confidence_threshold = 0.3;

    } param_;

    std::chrono::steady_clock::time_point last_time_;
    std::unique_ptr<Pt4YoloV8K> pt4_yolo_v8k_;
    std::unique_ptr<Pt4YoloV8K> pt4_yolo_v8k_sentry_backup_;
    std::unique_ptr<YoloPoseDetector> pt4_yolo_pose;

    std::unique_ptr<RVD::RVNumberClassifier> rv_number_classifier_;

    std::unique_ptr<FP::RVD::RVDetector> rv_detector_;
};
}  // namespace FP::PT4D
