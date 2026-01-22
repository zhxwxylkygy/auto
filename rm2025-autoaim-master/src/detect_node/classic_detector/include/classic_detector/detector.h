#pragma once

#include <classic_detector/light_corner_corrector.h>
#include <classic_detector/number_classifier.h>
#include <fmt/core.h>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "common.h"

namespace vpie {

// Armor size, Unit: m
constexpr double SMALL_ARMOR_WIDTH = 133.0 / 1000.0;  // 135
constexpr double SMALL_ARMOR_HEIGHT = 50.0 / 1000.0;  // 55
constexpr double LARGE_ARMOR_WIDTH = 225.0 / 1000.0;
constexpr double LARGE_ARMOR_HEIGHT = 50.0 / 1000.0;  // 55

// 15 degree in rad
constexpr double FIFTTEN_DEGREE_RAD = 15 * CV_PI / 180;

struct Detector {
    struct LightParams {
        // width / height
        int binary_thres;
        double min_ratio;
        double max_ratio;
        // vertical angle
        double max_angle;
        // judge color
        int color_diff_thresh;
        int min_pixel_to_edge;
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

    struct TimeStamp {
        using Tp = std::chrono::steady_clock::time_point;
        Tp start;
        Tp after_pi;
        Tp after_fl;
        Tp after_ml;
        Tp after_nc;

        static Tp GetTimePoint() { return std::chrono::steady_clock::now(); }
    };
    TimeStamp time_stamp{};

    EnemyColor detect_color;
    LightParams light_params;
    ArmorParams armor_params;
    double light_bright_ratio = 1.0;

    std::unique_ptr<NumberClassifier> classifier;
    std::unique_ptr<LightCornerCorrector> corner_corrector;

    // Debug msgs
    cv::Mat binary_img;
    cv::Mat gray_img_;

    std::vector<Light> lights_;
    std::vector<Armor> armors_;

    Detector(const EnemyColor& color, const LightParams& l, const ArmorParams& a);

    cv::Mat preprocessImage(const cv::Mat& rgb_img) noexcept;

    std::vector<Armor> detect(const cv::Mat& input) noexcept;
    bool isLight(const Light& light) const noexcept;

    std::vector<Light> findLights(const cv::Mat& rgb_img, const cv::Mat& binary_img) const noexcept;

    std::vector<Armor> matchLights(const std::vector<Light>& lights) const noexcept;
    // Check if there is another light in the boundingRect formed by the 2 lights
    bool containLight(int i, int j, const std::vector<Light>& lights) const noexcept;

    ArmorType isArmor(const Light& light_1, const Light& light_2) const noexcept;
    void drawResults(cv::Mat& img) const noexcept;
};
}  // namespace vpie