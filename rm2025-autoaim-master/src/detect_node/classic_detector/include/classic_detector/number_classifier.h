#pragma once

#include <classic_detector/armor.h>
#include <fmt/core.h>
#include <filesystem>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
namespace vpie {
// Class used to classify the number of the armor, based on the MLP model
class NumberClassifier {
   public:
    struct Params {
        int blur_kernel_size;
        int adaptive_binary_kernel_size;
        std::filesystem::path model_path;
        std::filesystem::path label_path;
        double confidence_threshold;
        std::vector<std::string> ignore_classes;
        bool enable_size_corrector;
        bool use_bilateral_filter;
    };

    explicit NumberClassifier(const Params& params);

    // Extract the roi image of number from the src
    cv::Mat extractNumber(const cv::Mat& src, const Armor& armor, ArmorType type) const noexcept;

    // Classify the number of the armor
    Armor::ClassifyResult classify(const cv::Mat& number_img) noexcept;

    // Erase the ignore classes
    void eraseIgnoreClasses(std::vector<Armor>& armors) noexcept;

    void Run(const cv::Mat& src, Armor& armor);

   private:
    std::mutex mutex_;
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    Params params_;
};
}  // namespace vpie