#include <bits/chrono.h>
#include <classic_detector/number_classifier.h>
// #include <tbb/tbb.h>
#include <algorithm>
#include <execution>
#include <opencv2/imgproc.hpp>
#include "classic_detector/armor.h"
#include "classic_detector/common.h"
#include "fstream"

vpie::NumberClassifier::NumberClassifier(const Params& params) : params_(params) {
    net_ = cv::dnn::readNetFromONNX(params_.model_path);
    std::ifstream label_file(params_.label_path);
    std::string line;
    while (std::getline(label_file, line)) {
        class_names_.push_back(line);
    }
}

void vpie::NumberClassifier::Run(const cv::Mat& src, Armor& armor) {
    if (!params_.enable_size_corrector) {
        auto number_image = extractNumber(src, armor, armor.type);
        auto classify_result = classify(number_image);
        armor.SetClassifyResult(classify_result);
        armor.number_img = number_image;
        return;
    }

    cv::Mat large_img;
    cv::Mat small_img;
    Armor::ClassifyResult large_classify_result;
    Armor::ClassifyResult small_classify_result;
    small_img = extractNumber(src, armor, ArmorType::SMALL);
    small_classify_result = classify(small_img);
    large_img = extractNumber(src, armor, ArmorType::LARGE);
    large_classify_result = classify(large_img);

    // tbb::parallel_invoke(
    //     [&]() {

    //     },
    //     [&]() {

    //     });

    if ((large_classify_result.Is("1") && small_classify_result.As("1")) ||
        (large_classify_result.Is("base") && small_classify_result.As("base"))) {
        if (armor.type != ArmorType::LARGE)
            armor.is_corrected = true;
        armor.type = ArmorType::LARGE;
        armor.SetClassifyResult(large_classify_result);
        armor.number_img = large_img;
    } else if ((small_classify_result.Is("2") && large_classify_result.As("2")) ||
               (small_classify_result.Is("sentry") && large_classify_result.As("sentry")) ||
               (small_classify_result.Is("outpost") && large_classify_result.As("outpost"))) {
        if (armor.type != ArmorType::SMALL)
            armor.is_corrected = true;
        armor.type = ArmorType::SMALL;
        armor.SetClassifyResult(small_classify_result);
        armor.number_img = small_img;
    } else {
        if (armor.type == ArmorType::SMALL) {
            armor.SetClassifyResult(small_classify_result);
            armor.number_img = small_img;
        } else if (armor.type == ArmorType::LARGE) {
            armor.SetClassifyResult(large_classify_result);
            armor.number_img = large_img;
        }
    }
}

cv::Mat vpie::NumberClassifier::extractNumber(const cv::Mat& src, const Armor& armor, ArmorType type) const noexcept {
    // Light length in image
    static const int light_length = 12;
    // Image size after warp
    static const int warp_height = 28;
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;
    // Number ROI size
    static const cv::Size roi_size(20, 28);
    static const cv::Size middle_size(56, 56);
    static const cv::Size input_size(28, 28);

    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
                                      armor.right_light.bottom};

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));
    armor.original_img = number_image.clone();

    // Get ROI
    number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));
    cv::resize(armor.original_img, armor.original_img, input_size);

    // Binarize
    cv::Mat gray_tmp;
    cv::cvtColor(number_image, gray_tmp, cv::COLOR_RGB2GRAY);
    if (params_.blur_kernel_size > 0)
        if (params_.use_bilateral_filter)
            cv::bilateralFilter(gray_tmp, number_image, params_.blur_kernel_size, 75, 75);
        else
            cv::GaussianBlur(gray_tmp, number_image, cv::Size(params_.blur_kernel_size, params_.blur_kernel_size), 0);
    else
        gray_tmp.copyTo(number_image);
    if (params_.adaptive_binary_kernel_size == 0)
        cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    else
        cv::adaptiveThreshold(number_image, number_image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY,
                              params_.adaptive_binary_kernel_size, 0);
    cv::resize(number_image, number_image, input_size);
    return number_image;
}

// Classify the number of the armor
vpie::Armor::ClassifyResult vpie::NumberClassifier::classify(const cv::Mat& number_img) noexcept {
    // Normalize
    cv::Mat input = number_img / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob);

    // Set the input blob for the neural network
    mutex_.lock();
    net_.setInput(blob);

    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward().clone();
    mutex_.unlock();

    // Decode the output
    double confidence;
    cv::Point class_id_point;
    minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    return {confidence, class_names_[label_id]};
}

// Erase the ignore classes
void vpie::NumberClassifier::eraseIgnoreClasses(std::vector<Armor>& armors) noexcept {
    std::for_each(std::execution::par_unseq, armors.begin(), armors.end(), [this](Armor& armor) {
        if (armor.confidence < params_.confidence_threshold) {
            armor.valid = false;
            return;
        }

        if (std::find(std::execution::par_unseq, params_.ignore_classes.begin(), params_.ignore_classes.end(),
                      armor.number) != params_.ignore_classes.end()) {
            armor.valid = false;
            return;
        }

        if (armor.type == ArmorType::LARGE) {
            armor.valid = armor.number != "outpost" && armor.number != "2" && armor.number != "sentry";
        } else if (armor.type == ArmorType::SMALL) {
            armor.valid = armor.number != "1" && armor.number != "base";
        }
    });
}