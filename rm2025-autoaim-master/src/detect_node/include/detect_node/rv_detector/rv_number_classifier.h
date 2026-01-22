// Copyright 2022 Chen Jun

#ifndef NUMBER_CLASSIFIER_HPP_
#define NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "detect_node/interface/interface.h"
#include "detect_node/interface/rm_interface.h"

namespace FP::RVD {
class RVNumberClassifier {
   public:
    RVNumberClassifier() = delete;

    RVNumberClassifier(const std::string& model_path,
                       double threshold,
                       const std::vector<RM::ArmorId>& ignore_classes = {});

    void extractNumbers(const cv::Mat& src, std::vector<RVDArmor>& armors);

    void classify(std::vector<RVDArmor>& armors);

    double threshold;

   private:
    std::array<std::string, 9> class_names_{"one",     "two",    "three", "four",   "five",
                                            "outpost", "sentry", "base",  "invalid"};
    cv::dnn::Net net_;
    std::vector<RM::ArmorId> ignore_classes_;
};
}  // namespace FP::RVD

#endif  // ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_
