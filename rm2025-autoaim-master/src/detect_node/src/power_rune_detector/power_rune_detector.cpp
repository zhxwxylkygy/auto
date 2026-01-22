// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include "detect_node/power_rune_detector/power_rune_detector.h"
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <thread>
#include <unordered_map>
#include <vector>
#include "detect_node/interface/interface.h"

using namespace FP;
using namespace FP::PD;
PowerRuneDetector::PowerRuneDetector([[maybe_unused]] const RM::Color input_color) {

    power_rune_yolo_v11_ = std::make_unique<PowerRuneYOLOv11>(
        PowerRuneYOLOv11::Config{param_.confidence_threshold, param_.nms_threshold, param_.score_threshold, param_.onnx_path}
    );
}

std::vector<std::shared_ptr<IFeature>> PowerRuneDetector::Run(const cv::Mat& input_image, const MCU::Orders& input_order_data, cv::Mat& result_img) {
    auto debug_img = input_image.clone();

    auto yolo_output = power_rune_yolo_v11_->Run(input_image, param_.enable_debug_image);

    std::vector<std::shared_ptr<FanInfo>> fans;

    std::for_each(yolo_output.begin(), yolo_output.end(), [&fans](PowerRuneYOLOv11::Fan& input) {
        std::shared_ptr<FanInfo> feature = std::make_shared<FanInfo>();
        feature->image_coord_lights_surface_points.left = input.lights_surface.left;
        feature->image_coord_lights_surface_points.top = input.lights_surface.top;
        feature->image_coord_lights_surface_points.bottom = input.lights_surface.bottom;
        feature->image_coord_lights_surface_points.right = input.lights_surface.right;
        feature->image_coord_lights_surface_points.center = input.lights_surface.center;
        //feature->image_coord_r_symbol_point = input.bull_point;
        feature->is_target_fan = true;
        fans.push_back(feature);
    });
        
    if (fans.size() > 1)
        fans.clear();
    // todo 将识�?结果的纠正和过滤抽象为防抖层

    if (!fans.empty()) {
        auto r_symbol_point_opt = DetectRSymbol(*fans.at(0).get(), input_image, param_.enable_debug_image);
        if (!r_symbol_point_opt.has_value()) {
            fans.clear();
        } else {
            std::for_each(fans.begin(), fans.end(), [&r_symbol_point_opt](const std::shared_ptr<FanInfo>& input_fans) {
                input_fans->image_coord_r_symbol_point = r_symbol_point_opt.value();
            });
        }
    }

    std::vector<std::shared_ptr<IFeature>> output;
    std::for_each(fans.begin(), fans.end(), [&output](std::shared_ptr<FanInfo>& fan) {
        output.push_back(std::static_pointer_cast<IFeature>(fan));
    });

    if (param_.enable_debug_image) {
        cv::Mat debug_img = input_image.clone();
        std::for_each(fans.begin(), fans.end(), [&debug_img](auto& fan) {
            cv::line(debug_img, fan->image_coord_lights_surface_points.left, fan->image_coord_lights_surface_points.top,
                     cv::Scalar(100, 100, 255), 4);
            cv::line(debug_img, fan->image_coord_lights_surface_points.top,
                     fan->image_coord_lights_surface_points.right, cv::Scalar(100, 100, 255), 4);
            cv::line(debug_img, fan->image_coord_lights_surface_points.right,
                     fan->image_coord_lights_surface_points.bottom, cv::Scalar(0, 250, 0), 4);
            cv::line(debug_img, fan->image_coord_lights_surface_points.bottom,
                     fan->image_coord_lights_surface_points.left, cv::Scalar(100, 100, 255), 4);
            cv::line(debug_img, fan->image_coord_lights_surface_points.top, fan->image_coord_lights_surface_points.bottom,
                     cv::Scalar(100, 100, 255), 4);
            cv::line(debug_img, fan->image_coord_lights_surface_points.left,
                     fan->image_coord_lights_surface_points.right, cv::Scalar(250, 0, 0), 4);
            cv::line(debug_img, fan->image_coord_lights_surface_points.left, fan->image_coord_r_symbol_point,
                        cv::Scalar(33, 66, 99), 4);
            cv::line(debug_img, fan->image_coord_lights_surface_points.right,
                        fan->image_coord_r_symbol_point, cv::Scalar(188, 88, 66), 4);
        });
        cv::imshow("test", debug_img);
    }

    return output;
}

std::optional<cv::Point> PowerRuneDetector::DetectRSymbol(const FP::FanInfo& input_fan_info,
                                                          const cv::Mat& input_image,
                                                          bool enable_debug_image) {
    // cv::Point top_mid = (input_fan_info.image_coord_lights_surface_points.left +
    //     input_fan_info.image_coord_lights_surface_points.right) /
    //                    2;
    // cv::Point bottom_mid = (input_fan_info.image_coord_lights_surface_points.top +
    //                        input_fan_info.image_coord_lights_surface_points.bottom) /
    //                       2;

    // cv::Point to_centre_vec = bottom_mid - top_mid;
    cv::Point  to_centre_vec = input_fan_info.image_coord_lights_surface_points.bottom - input_fan_info.image_coord_lights_surface_points.top;                                  
    cv::Point estimated_centre_point = input_fan_info.image_coord_lights_surface_points.center;
    // robustness too low...
    auto half_roi_length = param_.r_symbol_detect_roi_half_length;
    cv::Point left_top_offset_vec(-half_roi_length, -half_roi_length);
    cv::Point right_bottom_offset_vec(half_roi_length, half_roi_length);
    cv::Point to_r_symbol_vec = to_centre_vec* param_.to_centre_vec_coefficient;                          
    cv::Point r_symbol_approximate_point = estimated_centre_point + to_r_symbol_vec;
    // cv::Mat dddd =    input_image.clone();                                                 
    // cv::circle(dddd, r_symbol_approximate_point, 6, cv::Scalar(0, 0, 255), -1);
    // cv::line(dddd, estimated_centre_point, estimated_centre_point + to_centre_vec,
    //     cv::Scalar(100, 100, 255), 4);
    // cv::line(dddd, estimated_centre_point, estimated_centre_point + to_r_symbol_vec,
    //         cv::Scalar(100, 100, 255), 1);
    //cv::imshow("debug", dddd);
    auto pt_1 = r_symbol_approximate_point + left_top_offset_vec;
    auto pt_2 = r_symbol_approximate_point + right_bottom_offset_vec;
    cv::Rect roi(pt_1, pt_2);
    if (!(roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= input_image.cols &&
          roi.y + roi.height <= input_image.rows)) {
        return std::nullopt;
    }
    cv::Mat roi_img = input_image(roi);

    cv::cvtColor(roi_img, roi_img, cv::COLOR_BGR2GRAY);
    cv::threshold(roi_img, roi_img, param_.gray_img_threshold_value, 255, cv::THRESH_BINARY);
    if (enable_debug_image)
        cv::imshow("roi_gray_image", roi_img);

    // 除去�?小的碎块
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roi_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    contours.erase(std::remove_if(contours.begin(), contours.end(),
                                  [](std::vector<cv::Point>& input_contour) {
                                      auto area = contourArea(input_contour);
                                      return area <= 40;
                                      // todo add to param loader
                                  }),
                   contours.end());

    roi_img.setTo(0);
    cv::drawContours(roi_img, contours, -1, cv::Scalar(255), -1);

    if (enable_debug_image)
        cv::imshow("roi_after_filtrate_img", roi_img);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(roi_img, roi_img, cv::MORPH_DILATE, kernel);
    cv::morphologyEx(roi_img, roi_img, cv::MORPH_DILATE, kernel);
    cv::morphologyEx(roi_img, roi_img, cv::MORPH_DILATE, kernel);

    if (enable_debug_image)
        cv::imshow("roi_after_opening_img", roi_img);

    contours.clear();
    cv::findContours(roi_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    contours.erase(std::remove_if(contours.begin(), contours.end(),
                                  [this](std::vector<cv::Point>& input_contour) {
                                      auto area = contourArea(input_contour);
                                      return area <= param_.contour_area_threshold_min || area >= param_.contour_area_threshold_max;
                                  }),
                   contours.end());

    if (enable_debug_image) {
        cv::Mat after_area_threshold_img(roi_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        cv::drawContours(after_area_threshold_img, contours, -1, cv::Scalar(100, 100, 255), -1);
        cv::imshow("roi_aft_area_threshold_img", after_area_threshold_img);
    }

    // if (contours.size() != 1) {
    //     std::cerr << "r symbol num unequal to 1" << std::endl;
    //     return std::nullopt;
    // }
    // std::map<cv::Point2d, std::vector<cv::Point>> counters_centre_map;
    // std::unordered_map<cv::Point2d, std::vector<cv::Point>> counters_centre_map;
    std::vector<std::pair<cv::Point2d, std::vector<cv::Point>>> counters_centre_map;
    

    if (contours.empty())
        return std::nullopt;

    std::for_each(contours.begin(), contours.end(), [&counters_centre_map](auto& input) {
        auto moments = cv::moments(input);
        cv::Point2d centre_point(moments.m10 / moments.m00, moments.m01 / moments.m00);
        counters_centre_map.push_back(std::make_pair(centre_point, input));
    });

    auto target_countour_pair_ptr =
        std::min_element(counters_centre_map.begin(), counters_centre_map.end(), [half_roi_length](auto& a, auto& b) {
            cv::Point2d roi_mid_point(half_roi_length, half_roi_length);
            auto a_centre_distance = cv::norm(roi_mid_point - a.first);
            auto b_centre_distance = cv::norm(roi_mid_point - b.first);
            return a_centre_distance < b_centre_distance;
        });

    auto contour_centre = target_countour_pair_ptr->first;
    cv::Point original_img_r_symbol_centre = cv::Point(contour_centre) + roi.tl();

    if (enable_debug_image) {
        auto debug_img = input_image.clone();
        cv::circle(debug_img, original_img_r_symbol_centre, 5, cv::Scalar(114, 51, 4), -1);
        cv::imshow("power rune r symbol", debug_img);
    }

    return original_img_r_symbol_centre;
}