#include <chrono>
#include <classic_detector/detector.h>
#include <fmt/core.h>
#include <execution>
#include <opencv2/imgproc.hpp>
#include "classic_detector/armor.h"
#include "classic_detector/common.h"

vpie::Detector::Detector(const EnemyColor& color, const LightParams& l, const ArmorParams& a)
    : detect_color(color), light_params(l), armor_params(a) {}

cv::Mat vpie::Detector::preprocessImage(const cv::Mat& rgb_img) noexcept {
    cv::cvtColor(rgb_img, gray_img_, cv::COLOR_RGB2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img_, binary_img, light_params.binary_thres, 255, cv::THRESH_BINARY);
    return binary_img;
}

std::vector<vpie::Armor> vpie::Detector::detect(const cv::Mat& input) noexcept {
    time_stamp.start = TimeStamp::GetTimePoint();
    // 1. Preprocess the image
    binary_img = preprocessImage(input);
    time_stamp.after_pi = TimeStamp::GetTimePoint();
    // 2. Find lights
    lights_ = findLights(input, binary_img);
    time_stamp.after_fl = TimeStamp::GetTimePoint();
    // 3. Match lights to armors
    armors_ = matchLights(lights_);
    time_stamp.after_ml = TimeStamp::GetTimePoint();
    if (!armors_.empty() && classifier != nullptr) {
        // Parallel processing
        std::for_each(std::execution::par_unseq, armors_.begin(), armors_.end(), [this, &input](Armor& armor) {
            // // 4. Extract the number image
            // armor.number_img = classifier->extractNumber(input, armor);
            // // 5. Do classification
            // classifier->classify(input, armor);

            classifier->Run(input, armor);

            // 6. Correct the corners of the armor
            if (corner_corrector != nullptr) {
                corner_corrector->correctCorners(armor, gray_img_);
            }
        });

        // 7. Erase the armors with ignore classes
        classifier->eraseIgnoreClasses(armors_);
        // fmt::print("======\n");
    }
    time_stamp.after_nc = TimeStamp::GetTimePoint();

    return armors_;
}

bool vpie::Detector::isLight(const Light& light) const noexcept {
    // The ratio of light (short side / long side)
    float ratio = static_cast<float>(light.width) / static_cast<float>(light.length);
    bool ratio_ok = light_params.min_ratio < ratio && ratio < light_params.max_ratio;

    bool angle_ok = light.tilt_angle < light_params.max_angle;
    bool area_ok = cv::contourArea(light.contour_) / (light.length * light.width) > 0.6;

    // bool is_light = ratio_ok && angle_ok && area_ok;
    bool is_light = ratio_ok && angle_ok;

    return is_light;
}

std::vector<vpie::Light> vpie::Detector::findLights(const cv::Mat& rgb_img, const cv::Mat& binary_img) const noexcept {
    using std::vector;
    static constexpr bool is_tl = false;
    static constexpr int color_thresh = 120;

    auto binary_img_ = binary_img;

    if constexpr (is_tl) {
        cv::Mat channel[3];
        cv::split(rgb_img, channel);
        cv::Mat single_color;
        if (detect_color == EnemyColor::BLUE) {
            single_color = channel[0] - channel[2];
            cv::imshow("single_color", single_color);

            cv::threshold(single_color, single_color, color_thresh, 255, cv::THRESH_BINARY);

        } else {
            single_color = channel[2] - channel[0];
            cv::imshow("single_color", single_color);

            cv::threshold(single_color, single_color, color_thresh, 255, cv::THRESH_BINARY);
        }
        binary_img_ = (binary_img & single_color);
        cv::medianBlur(binary_img_, binary_img_, 3);
        cv::imshow("and", binary_img_);
    }

    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img_, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    vector<Light> lights;

    for (const auto& contour : contours) {
        if (contour.size() < 6)
            continue;

        auto light = Light(contour);

        if (isLight(light)) {
            if (is_tl)
                light.color = detect_color;
            else {
                int sum_r = 0, sum_b = 0;
                for (const auto& point : contour) {
                    sum_r += rgb_img.at<cv::Vec3b>(point.y, point.x)[2];
                    sum_b += rgb_img.at<cv::Vec3b>(point.y, point.x)[0];
                }
                if (std::abs(sum_r - sum_b) / static_cast<int>(contour.size()) > light_params.color_diff_thresh) {
                    light.color = sum_r > sum_b ? EnemyColor::RED : EnemyColor::BLUE;
                }
            }
            
            if ((abs(light.top.x - 0) < light_params.min_pixel_to_edge ||
                 abs(light.top.x - rgb_img.cols) < light_params.min_pixel_to_edge ||
                 abs(light.top.y - 0) < light_params.min_pixel_to_edge ||
                 abs(light.top.y - rgb_img.rows) < light_params.min_pixel_to_edge) ||
                abs(light.bottom.x - 0) < light_params.min_pixel_to_edge ||
                abs(light.bottom.x - rgb_img.cols) < light_params.min_pixel_to_edge ||
                abs(light.bottom.y - 0) < light_params.min_pixel_to_edge ||
                abs(light.bottom.y - rgb_img.rows) < light_params.min_pixel_to_edge)
                continue;
            lights.emplace_back(light);
        }
    }
    std::sort(lights.begin(), lights.end(), [](const Light& l1, const Light& l2) { return l1.center.x < l2.center.x; });
    return lights;
}

std::vector<vpie::Armor> vpie::Detector::matchLights(const std::vector<Light>& lights) const noexcept {
    std::vector<Armor> armors;
    // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
        if (light_1->color != detect_color)
            continue;

        double max_iter_width = light_1->length * armor_params.max_large_center_distance;

        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
            if (light_2->color != detect_color)
                continue;
            if (containLight(static_cast<int>(light_1 - lights.begin()), static_cast<int>(light_2 - lights.begin()),
                             lights)) {
                continue;
            }
            if (light_2->center.x - light_1->center.x > max_iter_width)
                break;

            auto type = isArmor(*light_1, *light_2);
            if (type != ArmorType::INVALID) {
                auto armor = Armor(*light_1, *light_2);
                armor.type = type;
                armors.emplace_back(armor);
            }
        }
    }

    return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool vpie::Detector::containLight(int i, int j, const std::vector<Light>& lights) const noexcept {
    const Light &light_1 = lights.at(i), light_2 = lights.at(j);
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);
    double avg_length = (light_1.length + light_2.length) / 2.0;
    double avg_width = (light_1.width + light_2.width) / 2.0;
    // Only check lights in between
    for (int k = i + 1; k < j; k++) {
        const Light& test_light = lights.at(k);

        // 防止数字干扰
        if (test_light.width > 2 * avg_width) {
            continue;
        }
        // 防止红点准星或弹丸干扰
        if (test_light.length < 0.5 * avg_length) {
            continue;
        }

        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center)) {
            return true;
        }
    }
    return false;
}

vpie::ArmorType vpie::Detector::isArmor(const Light& light_1, const Light& light_2) const noexcept {
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio =
        light_1.length < light_2.length ? light_1.length / light_2.length : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > armor_params.min_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok = (armor_params.min_small_center_distance <= center_distance &&
                               center_distance < armor_params.max_small_center_distance) ||
                              (armor_params.min_large_center_distance <= center_distance &&
                               center_distance < armor_params.max_large_center_distance);

    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < armor_params.max_angle;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

    // Judge armor type
    ArmorType type;
    if (is_armor) {
        type = center_distance > armor_params.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
        // fmt::print("center_distance: {}\n", center_distance);
    } else {
        type = ArmorType::INVALID;
    }

    return type;
}

void vpie::Detector::drawResults(cv::Mat& img) const noexcept {
    // Draw Lights

    for (const auto& light : lights_) {
        auto line_color = light.color == EnemyColor::RED ? cv::Scalar(255, 255, 0) : cv::Scalar(0, 255, 255);
        // cv::ellipse(img, light, line_color, 2);
        cv::line(img, light.top, light.bottom, line_color, 2);
    }

    // Draw armors
    for (const auto& armor : armors_) {
        if (!armor.valid)
            continue;
        cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 1);
        cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1);

        cv::circle(img, armor.left_light.top, 2, cv::Scalar(0, 0, 255), -1);
        cv::circle(img, armor.left_light.bottom, 2, cv::Scalar(0, 0, 255), -1);
        cv::circle(img, armor.right_light.top, 2, cv::Scalar(0, 0, 255), -1);
        cv::circle(img, armor.right_light.bottom, 2, cv::Scalar(0, 0, 255), -1);
        // cv::line(img, armor.left_light.top, armor.left_light.bottom,
        // cv::Scalar(0, 255, 0), 1, cv::LINE_AA); cv::line(img,
        // armor.right_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0),
        // 1, cv::LINE_AA); cv::line(img, armor.left_light.top,
        // armor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        // cv::line(img, armor.right_light.bottom, armor.left_light.bottom,
        // cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    // Show numbers and confidence
    for (const auto& armor : armors_) {
        if (!armor.valid)
            continue;
        std::string text = fmt::format("{} {} {}", armorTypeToString(armor.type), Color2String(armor.left_light.color),
                                       armor.classfication_result);
        cv::putText(img, text, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    }
}