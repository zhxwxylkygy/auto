#include <classic_detector/detector.h>
#include <memory>
namespace vpie {


std::shared_ptr<Detector> initDetector(vpie::EnemyColor color,
                                       const Detector::LightParams& l_params,
                                       const Detector::ArmorParams& a_params,
                                       const NumberClassifier::Params& nc_params);

std::vector<Armor> detectArmors(const std::shared_ptr<Detector>& detector_, const cv::Mat& img);
}  // namespace vpie