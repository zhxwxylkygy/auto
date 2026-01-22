// Copyright 2022 Chen Jun
// Licensed under the MIT License.

// OpenCV
#include <string>
#include <vector>
#include "detect_node/interface/rm_interface.h"
#include "opencv2/opencv.hpp"

#include "detect_node/interface/interface.h"
#include "param_loader.hpp"
#include "detect_node/rv_detector/rv_number_classifier.h"

using namespace FP;
using namespace FP::RVD;

RVNumberClassifier::RVNumberClassifier(const std::string& model_path,
                                       const double thre,
                                       const std::vector<RM::ArmorId>& ignore_classes)
    : threshold(thre), ignore_classes_(ignore_classes) {
    net_ = cv::dnn::readNetFromONNX(model_path);
}

void RVNumberClassifier::extractNumbers(const cv::Mat& src, std::vector<RVDArmor>& armors) {
    // Light length in image
    const int light_length = 12;
    // Image size after warp
    const int warp_height = 28;
    const int small_armor_width = 32;
    const int large_armor_width = 54;
    // Number ROI size
    const cv::Size roi_size(20, 28);

    for (auto& armor : armors) {
        // Warp perspective transform
        cv::Point2f lights_vertices[4] = {armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
                                          armor.right_light.bottom};

        const int top_light_y = (warp_height - light_length) / 2 - 1;
        const int bottom_light_y = top_light_y + light_length;
        const int warp_width = armor.type == RM::ArmorSize::SMALL ? small_armor_width : large_armor_width;
        cv::Point2f target_vertices[4] = {
            cv::Point(0, bottom_light_y),
            cv::Point(0, top_light_y),
            cv::Point(warp_width - 1, top_light_y),
            cv::Point(warp_width - 1, bottom_light_y),
        };
        cv::Mat number_image;
        auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
        cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

        // Get ROI
        number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

        // Binarize
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);

        GaussianBlur(number_image, number_image, cv::Size(5, 5), 0);
        adaptiveThreshold(number_image, number_image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 7, 0);

        // cv::threshold(number_image, number_image, ParamLoader::FP::RVD::DNN_BINARY_THRESHOLD, 255,
        //               cv::THRESH_BINARY | cv::THRESH_OTSU);
        //            cv::Mat kernel_3 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
        //            cv::morphologyEx(number_image, number_image, cv::MORPH_ERODE, kernel_3);
        //            cv::Mat kernel_5 = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
        //            cv::morphologyEx(number_image, number_image, cv::MORPH_DILATE, kernel_5);

        armor.number_img = number_image;
    }
}

void RVNumberClassifier::classify(std::vector<RVDArmor>& armors) {
    std::size_t index = 0;
    for (auto& armor : armors) {
        cv::Mat image = armor.number_img.clone();

        // Normalize
        image = image / 255.0;

        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob);

        // Set the input blob for the neural network
        net_.setInput(blob);
        // Forward pass the image blob through the model
        cv::Mat outputs = net_.forward();

        // Do softmax
        float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double confidence;
        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int id_class = class_id_point.x;

        armor.confidence = confidence;

        //            std::cout << id_class << " ";
        switch (id_class) {
            case 0:
                armor.number = RM::ArmorId::ONE;
                break;
            case 1:
                armor.number = RM::ArmorId::TWO;
                break;
            case 2:
                armor.number = RM::ArmorId::THREE;
                break;
            case 3:
                armor.number = RM::ArmorId::FOUR;
                break;
            case 4:
                armor.number = RM::ArmorId::FIVE;
                break;
            case 5:
                armor.number = RM::ArmorId::OUTPOST;
                break;
            case 6:
                armor.number = RM::ArmorId::SENTRY;
                break;
            case 7:
                armor.number = RM::ArmorId::BASE;
                break;
            case 8:
                armor.number = RM::ArmorId::INVALID;
                break;
        }

        if (confidence < threshold)
            armor.number = RM::ArmorId::INVALID;

        std::stringstream result_ss;
        result_ss << class_names_[id_class];
        result_ss << ": " << std::fixed << std::setprecision(1) << armor.confidence * 100.0 << "%";
        armor.classification_result = result_ss.str();

        // cv::imshow(std::to_string(index++), armor.number_img);
    }

    //        std::cout << std::endl;

    // armors.erase(std::remove_if(armors.begin(), armors.end(),
    //                             [this](const RVDArmor& armor) {
    //                                 if (armor.confidence < threshold) {
    //                                     return true;
    //                                 } else {
    //                                     return false;
    //                                 }

    //                                 // for (const auto& ignore_class : ignore_classes_) {
    //                                 //     if (armor.number == ignore_class) {
    //                                 //         return true;
    //                                 //     }
    //                                 // }

    //                                 // bool mismatch_armor_type = false;
    //                                 // if (armor.type == RM::ArmorSize::LARGE) {
    //                                 //     mismatch_armor_type = armor.number == RM::ArmorId::OUTPOST ||
    //                                 //                           armor.number == RM::ArmorId::TWO ||
    //                                 //                           armor.number == RM::ArmorId::SENTRY;
    //                                 // } else if (armor.type == RM::ArmorSize::SMALL) {
    //                                 //     mismatch_armor_type = armor.number == RM::ArmorId::ONE;
    //                                 // }
    //                                 // return mismatch_armor_type;
    //                             }),
    //              armors.end());
}
