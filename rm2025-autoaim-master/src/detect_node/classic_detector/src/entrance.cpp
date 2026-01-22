#include <classic_detector/entrance.h>
#include <memory>
#include "classic_detector/common.h"


std::shared_ptr<vpie::Detector> vpie::initDetector(vpie::EnemyColor color,
                                                   const vpie::Detector::LightParams& l_params,
                                                   const vpie::Detector::ArmorParams& a_params,
                                                   const vpie::NumberClassifier::Params& nc_params
                                                   ) {
    bool use_pca = true;
    auto detector = std::make_shared<vpie::Detector>(color, l_params, a_params);

    // Init classifier
    detector->classifier = std::make_unique<vpie::NumberClassifier>(nc_params);

    // Init Corrector
    if (use_pca) {
        detector->corner_corrector = std::make_unique<vpie::LightCornerCorrector>();
    }

    return detector;
}

std::vector<vpie::Armor> vpie::detectArmors(const std::shared_ptr<vpie::Detector>& detector_, const cv::Mat& img) {
    return detector_->detect(img);
}
