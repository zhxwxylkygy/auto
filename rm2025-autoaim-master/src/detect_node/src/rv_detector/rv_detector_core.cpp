// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>
#include "angles/angles.h"
#include "detect_node/interface/interface.h"
#include "detect_node/interface/rm_interface.h"
#include "detect_node/rv_detector/rv_detector_core.h"

using namespace FP;
using namespace FP::RVD;

RVDetectorCore::RVDetectorCore(const int& bin_threshold,
                               const RM::Color& color,
                               const LightParams& l,
                               const ArmorParams& a)
    : binary_threshold_(bin_threshold), l_(l), a_(a) {
    target_color_ = color;
}

std::vector<RVDArmor> RVDetectorCore::Detect(const cv::Mat& input) {
    binary_img_ = PreProcessImage(input);
    if (param_.debug)
        cv::imshow("binary", binary_img_);
    lights_ = FindLights(input, binary_img_);
    // cout << lights_.size() << endl;
    armors_ = MatchLights(lights_);
    //    cv::imshow("?", input);
    //    std::cout << armors_.size() << std::endl;
    std::vector<std::vector<cv::Point>> vec;
    vec.resize(armors_.size());
    for (auto i = armors_.begin(); i != armors_.end(); i++) {
        vec.at(distance(armors_.begin(), i)).push_back(i->left_light.top);
        vec.at(distance(armors_.begin(), i)).push_back(i->right_light.top);
        vec.at(distance(armors_.begin(), i)).push_back(i->right_light.bottom);
        vec.at(distance(armors_.begin(), i)).push_back(i->left_light.bottom);
    }

    if (!armors_.empty()) {
        classifier_->extractNumbers(input, armors_);
        classifier_->classify(armors_);
    }

    // todo add invalid armor judge and dart armor judge
    return armors_;
}

cv::Mat RVDetectorCore::PreProcessImage(const cv::Mat& rgb_img) {
    const int red_gray_threshold = 80;
    const int red_split_sub_threshold = 90;
    const int blue_gray_threshold = 200;
    const int blue_split_sub_threshold = 10;

    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

    std::vector<cv::Mat> bgr_channel_img;
    cv::split(rgb_img, bgr_channel_img);  // qishi shi  bgr

    cv::Mat split_sub_img;

    cv::Mat color_threshold_bin;
    cv::Mat gray_threshold_bin;

    if (target_color_ == RM::Color::BLUE) {
        split_sub_img = bgr_channel_img.at(0) - bgr_channel_img.at(2);
        cv::threshold(split_sub_img, color_threshold_bin, blue_split_sub_threshold, 255,
                      cv::THRESH_BINARY);                                                          // 90 min_split_red
        cv::threshold(gray_img, gray_threshold_bin, blue_gray_threshold, 255, cv::THRESH_BINARY);  // 30 min_gray_red
    } else if (target_color_ == RM::Color::RED) {
        split_sub_img = bgr_channel_img.at(2) - bgr_channel_img.at(0);
        cv::threshold(split_sub_img, color_threshold_bin, red_split_sub_threshold, 255,
                      cv::THRESH_BINARY);                                                         // 30 min_split_blue
        cv::threshold(gray_img, gray_threshold_bin, red_gray_threshold, 255, cv::THRESH_BINARY);  // 50 min_gray_blue
    }

    cv::dilate(color_threshold_bin, color_threshold_bin, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    // cv::erode(color_threshold_bin, color_threshold_bin, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // cv::imshow("color", color_threshold_bin);
    // cv::imshow("gray", gray_threshold_bin);

    cv::Mat binary_img = gray_threshold_bin & color_threshold_bin;

    // cv::imshow("binary_img", binary_img);

    // cv::Mat binary_img;
    // cv::threshold(gray_img, binary_img, binary_threshold_, 255, cv::THRESH_BINARY);

    return binary_img;
}

std::vector<Light> RVDetectorCore::FindLights(const cv::Mat& input_image, const cv::Mat& binary_img) {
    using std::vector;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    vector<Light> lights;

    for (const auto& contour : contours) {
        if (static_cast<int>(contour.size()) < param_.contour_area_threshold)
            continue;

        auto r_rect = cv::minAreaRect(contour);
        auto light = Light(r_rect);

        if (IsLight(light)) {
            auto rect = light.boundingRect();
            if (  // Avoid assertion failed
                0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= input_image.cols && 0 <= rect.y &&
                0 <= rect.height && rect.y + rect.height <= input_image.rows) {
                int sum_r = 0, sum_b = 0;
                auto roi = input_image(rect);
                // Iterate through the ROI
                // for (int i = 0; i < roi.rows; i++) {
                //     for (int j = 0; j < roi.cols; j++) {
                //         if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
                //             // if point is inside contour
                //             sum_r += roi.at<cv::Vec3b>(i, j)[2];
                //             sum_b += roi.at<cv::Vec3b>(i, j)[0];
                //         }
                //     }
                // }
                // Sum of red pixels > sum of blue pixels ?
                // light.color = sum_r > sum_b ? RM::Color::RED : RM::Color::BLUE;
                lights.emplace_back(light);
            }
        }
    }

    return lights;
}

bool RVDetectorCore::IsLight(const Light& light) {
    // The ratio of light (short side / long side)
    float ratio = light.width / light.length;
    bool ratio_ok = l_.min_ratio < ratio && ratio < l_.max_ratio;

    bool angle_ok = light.tilt_angle < l_.max_angle;

    bool is_light = ratio_ok && angle_ok;

    return is_light;
}

std::vector<RVDArmor> RVDetectorCore::MatchLights(const std::vector<Light>& lights) {
    std::vector<RVDArmor> armors;
    //    std::cout << (detect_color == RM::Color::RED ? "red" : "blue") << std::endl;

    // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
            //            if (light_1->color != detect_color || light_2->color != detect_color) continue;
            // todo !!!!rv颜色判定在高曝光下可能出现问�?

            if (ContainLight(*light_1, *light_2, lights)) {
                continue;
            }

            auto type = IsArmor(*light_1, *light_2);

            //                if(type ==ArmorSize::SMALL){
            //                    cout << "small" << endl;
            //                }else if(type == ArmorSize::BIG){
            //                    cout << "big" << endl;
            //                }else{
            //                    cout << "invalid" << endl;
            //                }

            // cout << "is_armor:" << type << endl;
            if (type != RM::ArmorSize::INVALID) {
                auto armor = RVDArmor(*light_1, *light_2);
                armor.type = type;
                armors.emplace_back(armor);
            }
        }
    }

    return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool RVDetectorCore::ContainLight(const Light& light_1, const Light& light_2, const std::vector<Light>& lights) {
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto& test_light : lights) {
        if (test_light.center == light_1.center || test_light.center == light_2.center)
            continue;

        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center)) {
            return true;
        }
    }

    return false;
}

RM::ArmorSize RVDetectorCore::IsArmor(const Light& light_1, const Light& light_2) {
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio =
        light_1.length < light_2.length ? light_1.length / light_2.length : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > a_.min_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    //        std::cout << center_distance << std::endl;
    bool center_distance_ok =
        (a_.min_small_center_distance <= center_distance && center_distance < a_.max_small_center_distance) ||
        (a_.min_large_center_distance <= center_distance && center_distance < a_.max_large_center_distance);

    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < a_.max_angle;

    //    bool angle_diff_ok = fabs(light_1.tilt_angle - light_2.tilt_angle) < 10;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
    // bool is_armor = 1 && 1 && 1;

    // Judge armor type
    if (is_armor) {
        return center_distance > a_.min_large_center_distance ? RM::ArmorSize::LARGE : RM::ArmorSize::SMALL;
    } else {
        return RM::ArmorSize::INVALID;
    }
}

cv::Mat RVDetectorCore::getAllNumbersImage() {
    if (armors_.empty()) {
        return cv::Mat(cv::Size(20, 28), CV_8UC1);
    } else {
        std::vector<cv::Mat> number_imgs;
        number_imgs.reserve(armors_.size());
        for (auto& armor : armors_) {
            number_imgs.emplace_back(armor.number_img);
        }
        cv::Mat all_num_img;
        cv::vconcat(number_imgs, all_num_img);
        return all_num_img;
    }
}

void RVDetectorCore::DrawResults(cv::Mat& img) {
    // Draw Lights
    for (const auto& light : lights_) {
        cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
        cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
        auto line_color = light.color != RM::Color::RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
        cv::line(img, light.top, light.bottom, line_color, 1);
    }

    // Draw armors
    for (const auto& armor : armors_) {
        cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
        cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
    }

    // Show numbers and confidence
    for (const auto& armor : armors_) {
        cv::putText(img, armor.classification_result, armor.right_light.bottom, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);
    }
}
