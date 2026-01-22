// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include "detect_node/pt4_detector/pt4_detector.h"
#include <opencv2/core/hal/interface.h>
#include <algorithm>
#include <chrono>
#include <exception>
#include <forward_list>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include "detect_node/pt4_detector/pt4_yolo_v8k.h"
#include "detect_node/pt4_detector/yolo-pose.h"
#include "detect_node/rv_detector/rv_detector.h"
#include "detect_node/rv_detector/rv_number_classifier.h"
#include "detect_node/interface/interface.h"
#include "detect_node/interface/rm_interface.h"
#include <calculate/basic_calculate.h>
#include "detect_node/interface/mcu.h"

using namespace FP;
using namespace FP::PT4D;
Pt4Detector::Pt4Detector([[maybe_unused]] const RM::Color input_color) {
    pt4_yolo_v8k_ = std::make_unique<Pt4YoloV8K>(Pt4YoloV8K::Config{param_.confidence_threshold, param_.nms_threshold,
                                                                    param_.score_threshold, param_.input_width,
                                                                    param_.input_height, param_.yolo_v8k_onnx_path});
    pt4_yolo_v8k_sentry_backup_ = std::make_unique<Pt4YoloV8K>(
        Pt4YoloV8K::Config{0.5, 0.5, 0.5, param_.input_width, param_.input_height, param_.yolo_v8k_onnx_path});

    pt4_yolo_pose = std::make_unique<YoloPoseDetector>(YoloPoseDetector::Config{
                0.5, 0.5, 0.5, param_.yolo_v8k_onnx_path});
    rv_number_classifier_ = std::make_unique<RVD::RVNumberClassifier>(param_.classify_dnn_module_path,
                                                                      param_.classifier_confidence_threshold);

    rv_detector_ = std::make_unique<RVD::RVDetector>(input_color);

    enemy_color_ = input_color;
}

std::vector<std::shared_ptr<IFeature>> Pt4Detector::Run(const cv::Mat& input_image,
                                                        const MCU::Orders& input_order_data,
                                                        cv::Mat& result_img) {
    auto debug_img = input_image.clone();
    std::vector<std::shared_ptr<ArmorInfo>> armors;
    if (true) {
        auto yolo_output = pt4_yolo_pose->Run(input_image, param_.enable_debug_image);
        // todo switchable design

        std::for_each(yolo_output.begin(), yolo_output.end(), [this, &armors, &input_image](auto& input) {
            std::shared_ptr<ArmorInfo> armor = std::make_shared<ArmorInfo>();

            if (enemy_color_ == RM::Color::BLUE) {
                if (input.class_id >= 12 && input.class_id <= 23) {
                    return;
                }
            } else if (enemy_color_ == RM::Color::RED) {
                if (input.class_id >= 0 && input.class_id <= 11) {
                    return;
                }
            } else {
                std::cerr << "PRD: unknown enemy color" << std::endl;
                std::terminate();
            }
            armor->image_coord_armor_points.l_top = input.kpt[0];
            armor->image_coord_armor_points.l_bottom = input.kpt[1];
            armor->image_coord_armor_points.r_bottom = input.kpt[2];
            armor->image_coord_armor_points.r_top = input.kpt[3];


            switch (input.class_id) {
                case 0:
                case 12: {
                    armor->armor_id = RM::ArmorId::SENTRY;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 1:
                case 13: {
                    armor->armor_id = RM::ArmorId::ONE;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
                case 2:
                case 14: {
                    armor->armor_id = RM::ArmorId::TWO;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 3:
                case 15: {
                    armor->armor_id = RM::ArmorId::THREE;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 4:
                case 16: {
                    armor->armor_id = RM::ArmorId::FOUR;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 5:
                case 17: {
                    armor->armor_id = RM::ArmorId::FIVE;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 6:
                case 18: {
                    armor->armor_id = RM::ArmorId::OUTPOST;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 7:
                case 19: {
                    armor->armor_id = RM::ArmorId::BASE;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 8:
                case 20: {
                    armor->armor_id = RM::ArmorId::BASE;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
                case 9:
                case 21: {
                    armor->armor_id = RM::ArmorId::THREE;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
                case 10:
                case 22: {
                    armor->armor_id = RM::ArmorId::FOUR;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
                case 11:
                case 23: {
                    armor->armor_id = RM::ArmorId::FIVE;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
            }

            armor->is_dart_armor = false;

            armors.push_back(armor);
        });

        ExcludeImageEdgeArmor(armors);

        // std::cout << "111" << std::endl;
    } else if (input_order_data.special_order == MCU::AutoaimReceiveInfo::SpecialOrder::USE_SENTRY_BACKUP_MODULE) {
        auto yolo_output = pt4_yolo_v8k_sentry_backup_->Run(input_image, param_.enable_debug_image);
        // todo switchable design

        std::for_each(yolo_output.begin(), yolo_output.end(), [this, &armors, &input_image](auto& input) {
            std::shared_ptr<ArmorInfo> armor = std::make_shared<ArmorInfo>();

            if (enemy_color_ == RM::Color::BLUE) {
                if (input.class_id >= 12 && input.class_id <= 23) {
                    return;
                }
            } else if (enemy_color_ == RM::Color::RED) {
                if (input.class_id >= 0 && input.class_id <= 11) {
                    return;
                }
            } else {
                std::cerr << "PRD: unknown enemy color" << std::endl;
                std::terminate();
            }
            armor->image_coord_armor_points.l_top = input.kpt[0];
            armor->image_coord_armor_points.l_bottom = input.kpt[1];
            armor->image_coord_armor_points.r_bottom = input.kpt[2];
            armor->image_coord_armor_points.r_top = input.kpt[3];



            switch (input.class_id) {
                case 0:
                case 12: {
                    armor->armor_id = RM::ArmorId::SENTRY;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 1:
                case 13: {
                    armor->armor_id = RM::ArmorId::ONE;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
                case 2:
                case 14: {
                    armor->armor_id = RM::ArmorId::TWO;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 3:
                case 15: {
                    armor->armor_id = RM::ArmorId::THREE;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 4:
                case 16: {
                    armor->armor_id = RM::ArmorId::FOUR;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 5:
                case 17: {
                    armor->armor_id = RM::ArmorId::FIVE;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 6:
                case 18: {
                    armor->armor_id = RM::ArmorId::OUTPOST;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 7:
                case 19: {
                    armor->armor_id = RM::ArmorId::BASE;
                    armor->armor_size = RM::ArmorSize::SMALL;
                    break;
                }
                case 8:
                case 20: {
                    armor->armor_id = RM::ArmorId::BASE;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
                case 9:
                case 21: {
                    armor->armor_id = RM::ArmorId::THREE;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
                case 10:
                case 22: {
                    armor->armor_id = RM::ArmorId::FOUR;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
                case 11:
                case 23: {
                    armor->armor_id = RM::ArmorId::FIVE;
                    armor->armor_size = RM::ArmorSize::LARGE;
                    break;
                }
            }

            armor->is_dart_armor = false;

            armors.push_back(armor);
        });

        ExcludeImageEdgeArmor(armors);

        // std::cout << "222" << std::endl;
    }

    if (param_.enable_debug_image) {
        cv::Mat debug_image;
        input_image.copyTo(debug_img);
        for (auto& armor : armors) {
            cv::line(debug_img, armor->image_coord_armor_points.l_top, armor->image_coord_armor_points.r_bottom,
                     cv::Scalar(255, 255, 0));
            cv::line(debug_img, armor->image_coord_armor_points.l_bottom, armor->image_coord_armor_points.r_top,
                     cv::Scalar(255, 255, 0));
            std::stringstream str_s;

            if (armor->armor_size == RM::ArmorSize::LARGE) {
                str_s << "L: ";
            } else if (armor->armor_size == RM::ArmorSize::SMALL) {
                str_s << "S: ";
            }

            if (armor->armor_id == RM::ArmorId::BASE)
                str_s << "BASE";
            if (armor->armor_id == RM::ArmorId::OUTPOST)
                str_s << "OUTPOST";
            if (armor->armor_id == RM::ArmorId::ONE)
                str_s << "ONE";
            if (armor->armor_id == RM::ArmorId::TWO)
                str_s << "TWO";
            if (armor->armor_id == RM::ArmorId::THREE)
                str_s << "THREE";
            if (armor->armor_id == RM::ArmorId::FOUR)
                str_s << "FOUR";
            if (armor->armor_id == RM::ArmorId::FIVE)
                str_s << "FIVE";
            if (armor->armor_id == RM::ArmorId::SENTRY)
                str_s << "SENTRY";
            if (armor->armor_id == RM::ArmorId::INVALID)
                str_s << "----";

            cv::putText(debug_img, str_s.str(), armor->image_coord_armor_points.l_top, cv::FONT_HERSHEY_COMPLEX_SMALL,
                        1.0, cv::Scalar(255, 0, 255), 1);
        }

        cv::imshow("fixed pt4 debug", debug_img);
    }

    std::vector<std::shared_ptr<IFeature>> output;
    std::for_each(armors.begin(), armors.end(), [&output](std::shared_ptr<ArmorInfo>& armor) {
        output.push_back(std::static_pointer_cast<IFeature>(armor));
    });

    return output;
}
RM::ArmorSize Pt4Detector::JudgeArmorSizeByRatio(FP::ArmorInfo::ArmorPoints input_armor_points) {
    const double light_length_diff_weight = 8.;
    const double ratio_threshold = 0.3;
    double left_light_length = cv::norm(input_armor_points.l_top - input_armor_points.l_bottom);
    double right_light_length = cv::norm(input_armor_points.r_top - input_armor_points.r_bottom);
    double light_length_diff = std::fabs(right_light_length - left_light_length);
    double light_length_avg = (left_light_length + right_light_length) / 2;
    cv::Point left_light_mid_pt = (input_armor_points.l_top + input_armor_points.l_bottom) / 2;
    cv::Point right_light_mid_pt = (input_armor_points.r_top + input_armor_points.r_bottom) / 2;
    double mid_distance = cv::norm(left_light_mid_pt - right_light_mid_pt);
    double virtual_width = mid_distance + light_length_diff * light_length_diff_weight;
    double virtual_height = light_length_avg;

    double ratio = virtual_height / virtual_width;

    // std::cout << virtual_width << " " << virtual_height << " " << light_length_diff << std::endl;
    if (ratio <= ratio_threshold) {
        return RM::ArmorSize::LARGE;
    } else {
        return RM::ArmorSize::SMALL;
    }
}

std::optional<RM::ArmorSize> Pt4Detector::JudgeArmorSizeByMorphology(FP::ArmorInfo::ArmorPoints input_armor_points,
                                                                     const cv::Mat& input_image) {
    // auto a = std::chrono::steady_clock::now();
    // const int pattern_threshold = 50;
    const double pattern_rate_threshold = 0.23;

    const double armor_height_coefficient = 2.0;
    const double armor_width_coefficient = 0.8;

    cv::Point l_light_mid = (input_armor_points.l_bottom + input_armor_points.l_top) / 2;
    cv::Point r_light_mid = (input_armor_points.r_bottom + input_armor_points.r_top) / 2;
    double l_light_length = cv::norm(input_armor_points.l_bottom - input_armor_points.l_top);
    double r_light_length = cv::norm(input_armor_points.r_bottom - input_armor_points.r_top);
    // double adjusted_l_light_length = l_light_length * armor_height_coefficient;
    // double adjusted_r_light_length = r_light_length * armor_height_coefficient;
    cv::Point lights_mid_mid = (l_light_mid + r_light_mid) / 2;
    double lights_mid_distance = cv::norm(l_light_mid - r_light_mid);
    // double adjusted_lights_mid_distance = lights_mid_distance * armor_width_coefficient;

    cv::Point adjusted_l_light_mid = lights_mid_mid + (r_light_mid - l_light_mid) / 2 * armor_width_coefficient;
    cv::Point adjusted_r_light_mid = lights_mid_mid + (l_light_mid - r_light_mid) / 2 * armor_width_coefficient;

    cv::Point adjusted_l_top =
        adjusted_l_light_mid + (input_armor_points.l_top - input_armor_points.l_bottom) / 2 * armor_height_coefficient;
    cv::Point adjusted_l_bottom =
        adjusted_l_light_mid + (input_armor_points.l_bottom - input_armor_points.l_top) / 2 * armor_height_coefficient;
    cv::Point adjusted_r_top =
        adjusted_r_light_mid + (input_armor_points.r_top - input_armor_points.r_bottom) / 2 * armor_height_coefficient;
    cv::Point adjusted_r_bottom =
        adjusted_r_light_mid + (input_armor_points.r_bottom - input_armor_points.r_top) / 2 * armor_height_coefficient;

    std::vector<cv::Point> contour;
    // contour.push_back(input_armor_points.l_bottom);
    // contour.push_back(input_armor_points.l_top);
    // contour.push_back(input_armor_points.r_top);
    // contour.push_back(input_armor_points.r_bottom);
    contour.push_back(adjusted_l_bottom);
    contour.push_back(adjusted_l_top);
    contour.push_back(adjusted_r_top);
    contour.push_back(adjusted_r_bottom);
    double contour_area = cv::contourArea(contour);

    auto roi_rect = cv::boundingRect(contour);

    if (!IsPointInImage(roi_rect.tl(), input_image) || !IsPointInImage(roi_rect.br(), input_image))
        return std::nullopt;

    std::vector<cv::Point> pts_in_roi_rect{contour};
    std::for_each(pts_in_roi_rect.begin(), pts_in_roi_rect.end(), [&roi_rect](auto& input) { input -= roi_rect.tl(); });

    cv::Mat mask = cv::Mat::zeros(roi_rect.size(), CV_8UC3);
    cv::fillConvexPoly(mask, pts_in_roi_rect.data(), static_cast<int>(pts_in_roi_rect.size()),
                       cv::Scalar(255, 255, 255));

    cv::Mat roi_rect_image(input_image, roi_rect);
    cv::Mat armor_roi;
    roi_rect_image.copyTo(armor_roi, mask);

    cv::cvtColor(armor_roi, armor_roi, cv::COLOR_BGR2GRAY);

    cv::threshold(armor_roi, armor_roi, ParamLoader::GetInstance().GetParam<int>("FP","RVD","DNN_BINARY_THRESHOLD"), 255, cv::THRESH_BINARY);
    auto element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
    auto element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::erode(armor_roi, armor_roi, element2);
    cv::dilate(armor_roi, armor_roi, element1);

    auto m = cv::moments(armor_roi);

    double cx = m.m10 / m.m00;
    double cy = m.m01 / m.m00;

    cv::Point centre(cx, cy);

    double mu00 = m.m00;
    double mu11 = m.m11 - cx * m.m01;
    double mu20 = m.m20 - cx * m.m10;
    double mu02 = m.m02 - cy * m.m01;

    double theta = 1. / 2. * std::atan2(2 * mu11 / mu00, (mu20 - mu02) / mu00);

    // cv::imshow("333", armor_roi);

    double pattern_area = cv::sum(armor_roi)[0] / 255;

    // std::cout << pattern_area << " " << contour_area << std::endl;

    // auto b = std::chrono::steady_clock::now();

    // std::cout << "judge armor size by morphology: "
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count() << std::endl;

    if (pattern_area / contour_area >= pattern_rate_threshold)
        return RM::ArmorSize::SMALL;
    else
        return RM::ArmorSize::LARGE;
}

void Pt4Detector::ExcludeImageEdgeArmor(std::vector<std::shared_ptr<FP::ArmorInfo>>& input_armors) {
    auto judge_edge_point_f = [this](cv::Point point) {
        if (point.x >= param_.image_size.width - param_.min_distance_to_image_edge ||
            point.x <= 0 + param_.min_distance_to_image_edge ||
            point.y >= param_.image_size.height - param_.min_distance_to_image_edge ||
            point.y <= 0 + param_.min_distance_to_image_edge) {
            return true;
        } else {
            return false;
        };
    };

    auto new_end_iter = std::remove_if(input_armors.begin(), input_armors.end(), [judge_edge_point_f](auto& armor) {
        if (judge_edge_point_f(armor->image_coord_armor_points.l_top))
            return true;
        if (judge_edge_point_f(armor->image_coord_armor_points.r_top))
            return true;
        if (judge_edge_point_f(armor->image_coord_armor_points.l_bottom))
            return true;
        if (judge_edge_point_f(armor->image_coord_armor_points.r_bottom))
            return true;
        return false;
    });
    input_armors.erase(new_end_iter, input_armors.end());
}

std::tuple<RM::ArmorId, double> Pt4Detector::GetArmorIdByDNN(FP::ArmorInfo::ArmorPoints& input_armor_points,
                                                             RM::ArmorSize input_armor_size,
                                                             const cv::Mat& input_image) {
    RVD::RVDArmor rvd_armor;

    rvd_armor.left_light.top = input_armor_points.l_top;
    rvd_armor.left_light.bottom = input_armor_points.l_bottom;
    rvd_armor.right_light.top = input_armor_points.r_top;
    rvd_armor.right_light.bottom = input_armor_points.r_bottom;
    rvd_armor.type = input_armor_size;

    std::vector<RVD::RVDArmor> rvd_armors_warpper{rvd_armor};

    rv_number_classifier_->extractNumbers(input_image, rvd_armors_warpper);
    rv_number_classifier_->classify(rvd_armors_warpper);

    return std::make_tuple(rvd_armors_warpper.begin()->number,
                           static_cast<double>(rvd_armors_warpper.begin()->confidence));
}