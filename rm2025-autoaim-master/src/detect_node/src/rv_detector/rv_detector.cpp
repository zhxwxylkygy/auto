// Copyright 2022 Chen Jun
// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include "detect_node/rv_detector/rv_detector.h"
#include <algorithm>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "detect_node/interface/interface.h"

using namespace FP;
using namespace FP::RVD;
RVDetector::RVDetector(RM::Color input_color) {
    // Detector
    rv_detector_core_ = InitDetector(input_color);
}

std::vector<std::shared_ptr<IFeature>> RVDetector::Run(const cv::Mat& img_msg, const MCU::Orders& input_order_data, cv::Mat& result_img) {
    std::vector<std::shared_ptr<IFeature>> output;
    auto rvd_armors = DetectArmors(img_msg);

    std::for_each(rvd_armors.begin(), rvd_armors.end(), [&output](const RVDArmor& input) {
        std::shared_ptr<FP::ArmorInfo> armor = std::make_shared<FP::ArmorInfo>();
        armor->image_coord_armor_points = FP::ArmorInfo::ArmorPoints{input.left_light.bottom, input.left_light.top,
                                                                     input.right_light.top, input.right_light.bottom};
        armor->armor_size = input.type;
        armor->armor_id = input.number;

        auto feature = std::static_pointer_cast<IFeature>(armor);
        output.push_back(armor);
    });

    return output;
}

std::unique_ptr<RVDetectorCore> RVDetector::InitDetector(RM::Color input_color) {
    int binary_threshold = param_.binary_threshold;

    RM::Color detect_color{};
    detect_color = input_color;

    RVDetectorCore::LightParams l_params = {
        .min_ratio = param_.light_min_ratio, .max_ratio = param_.light_max_ratio, .max_angle = param_.light_max_angle};

    RVDetectorCore::ArmorParams a_params = {.min_light_ratio = param_.armor_min_light_ratio,
                                            .min_small_center_distance = param_.armor_min_small_center_distance,
                                            .max_small_center_distance = param_.armor_max_small_center_distance,
                                            .min_large_center_distance = param_.armor_min_large_center_distance,
                                            .max_large_center_distance = param_.armor_max_large_center_distance,
                                            .max_angle = param_.armor_max_angle_degree};

    auto detector = std::make_unique<RVDetectorCore>(binary_threshold, detect_color, l_params, a_params);

    auto module_path = param_.module_path;
    double threshold = param_.classifier_threshold;
    std::vector<RM::ArmorId> ignore_classes{};
    for (const auto& ignore_class : param_.ignore_classes) {
        ignore_classes.emplace_back(static_cast<RM::ArmorId>(ignore_class));
    }
    detector->classifier_ = std::make_unique<RVNumberClassifier>(module_path, threshold, ignore_classes);

    return detector;
}

std::vector<RVDArmor> RVDetector::DetectArmors(const cv::Mat& img_msg) {
    auto debug_img = img_msg.clone();

    auto armors = rv_detector_core_->Detect(img_msg);

    //        if (!armors.empty()) std::cout << static_cast<int>(armors[0].number) << std::endl;

    auto time = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(time - last_time_).count();

    // Publish debug info
    bool debug = param_.debug_mode;

    if (debug) {
        rv_detector_core_->DrawResults(debug_img);
        // Draw camera center
        //            cv::circle(img, {static_cast<int>(param::CAMERA_MATRIX.at<double>(0, 2)),
        //                             static_cast<int>(param::CAMERA_MATRIX.at<double>(1, 2))},
        //                       5, cv::Scalar(255, 0, 0), 2);
        // Draw latency
        std::stringstream latency_ss;
        latency_ss << "Latency: " << std::fixed << std::setprecision(4) << dt << "ms";
        auto latency_s = latency_ss.str();
        cv::putText(debug_img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        imshow("Run", debug_img);
    }

    last_time_ = time;

    return armors;
}