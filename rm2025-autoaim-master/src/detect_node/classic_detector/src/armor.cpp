#include <classic_detector/armor.h>

vpie::Armor::Armor(const Light& l1, const Light& l2) {
    if (l1.center.x < l2.center.x) {
        left_light = l1, right_light = l2;
    } else {
        left_light = l2, right_light = l1;
    }

    center = (left_light.center + right_light.center) / 2;
}

template <typename PointType>
std::vector<PointType> vpie::Armor::buildObjectPoints(const double& w, const double& h) {
    if constexpr (N_LANDMARKS == 4) {
        return {PointType(0, w / 2, -h / 2), PointType(0, w / 2, h / 2), PointType(0, -w / 2, h / 2),
                PointType(0, -w / 2, -h / 2)};
    } else {
        return {PointType(0, w / 2, -h / 2), PointType(0, w / 2, 0),  PointType(0, w / 2, h / 2),
                PointType(0, -w / 2, h / 2), PointType(0, -w / 2, 0), PointType(0, -w / 2, -h / 2)};
    }
}

std::vector<cv::Point2f> vpie::Armor::landmarks() const {
    if constexpr (N_LANDMARKS == 4) {
        return {left_light.bottom, left_light.top, right_light.top, right_light.bottom};
    } else {
        return {left_light.bottom, left_light.center,  left_light.top,
                right_light.top,   right_light.center, right_light.bottom};
    }
}

void vpie::Armor::SetClassifyResult(const ClassifyResult& cr) {
    confidence = cr.confidence;
    number = cr.number;
    classfication_result = cr.classfication_result;
}
