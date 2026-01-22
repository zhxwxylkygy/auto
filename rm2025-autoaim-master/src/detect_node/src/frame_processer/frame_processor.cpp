//
// Created by wpie on 23-11-1.
//

#include "detect_node/frame_processer/frame_processor.h"
#include <bits/chrono.h>
#include <detect_node/pt4_detector/yolo-pose.h>
#include <math.h>
#include <vofa_bridge/vofa_bridge.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <execution>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <rclcpp/logging.hpp>
#include <string>
#include <thread>
#include "angles/angles.h"
#include "calculate/basic_calculate.h"
#include "calculate/line_intersect_sphere_solver.h"
#include "detect_node/interface/interface.h"
#include "detect_node/interface/mcu.h"
#include "detect_node/interface/rm_interface.h"
#include "detect_node/ncd_adapter/ncd_adapter.hpp"
#include "detect_node/pt4_detector/pt4_detector.h"
#include "detect_node/rv_detector/rv_detector.h"


using namespace FP;

FrameProcessor::FrameProcessor(const RM::Color input_color) {
    switch (param_.detector_mode) {
        case ArmorDetectorMode::RV:
            armor_detector_ = std::make_unique<FP::RVD::RVDetector>(input_color);
            break;
        case ArmorDetectorMode::OLD_VISION:
            std::cerr << "FP: old_vision detector unfinished" << std::endl;
            std::terminate();
            break;
        case ArmorDetectorMode::PT4:
            armor_detector_ = std::make_unique<FP::PT4D::Pt4Detector>(input_color);
            break;
        case ArmorDetectorMode::NEW_CLASSIC_VISION:
            armor_detector_ = std::make_unique<vpie::NewClassicDetector>(input_color);
            break;
    }

    power_rune_detector_ = std::make_unique<FP::PD::PowerRuneDetector>(input_color);
}

std::vector<std::shared_ptr<FP::IFeature>> FrameProcessor::Run(const sensor_msgs::msg::Image& msg,
                                                               const MCU::Orders& input_order_data,
                                                               cv::Mat& result_img) {
    cv::Mat bayer_rg_8(msg.height, msg.width, CV_8U, (void*)msg.data.data());
    cv::Mat bgr_img(bayer_rg_8.size(), CV_8UC3);
    cv::cvtColor(bayer_rg_8, bgr_img, cv::COLOR_BayerRG2RGB);  // COLOR_BayerBG2BGR
    if (ParamLoader::GetInstance().GetParam<bool>("ROBOT", "INVERT_CAMERA")) {
        cv::flip(bgr_img, bgr_img, -1);
    }
    auto features = Detect(bgr_img, input_order_data, result_img);
    Verify(features, input_order_data);
    return features;
}

std::vector<std::shared_ptr<FP::IFeature>> FrameProcessor::Detect(const cv::Mat& input_image,
                                                                  const MCU::Orders& input_order_data,
                                                                  cv::Mat& result_img) {
    if (input_order_data.target_type == MCU::AutoaimReceiveInfo::TargetType::POWER_RUNE) {
        return power_rune_detector_->Run(input_image, input_order_data, result_img);
    } else {
        return armor_detector_->Run(input_image, input_order_data, result_img);
    }
}

void FrameProcessor::Verify(std::vector<std::shared_ptr<FP::IFeature>>& input_features,
                            const MCU::Orders& input_order_data) {
    auto new_end_iter = std::remove_if(
        input_features.begin(), input_features.end(), [&input_order_data](std::shared_ptr<FP::IFeature>& input) {
            if (input_order_data.target_type == MCU::AutoaimReceiveInfo::TargetType::POWER_RUNE) {
                // unfinished
                return false;

            } else {
                auto armor = std::static_pointer_cast<FP::ArmorInfo>(input);
                switch (armor->armor_id) {
                    case RM::ArmorId::ONE: {
                        if (armor->armor_size != RM::ArmorSize::LARGE)
                            return true;
                        break;
                    }
                    case RM::ArmorId::TWO:
                    case RM::ArmorId::SENTRY:
                    case RM::ArmorId::OUTPOST: {
                        if (armor->armor_size != RM::ArmorSize::SMALL)
                            return true;
                        break;
                    }
                    default: {
                        break;
                    }
                }
                if (armor->armor_size == RM::ArmorSize::INVALID)
                    return true;
                return false;
            }
        });
    input_features.erase(new_end_iter, input_features.end());
}
