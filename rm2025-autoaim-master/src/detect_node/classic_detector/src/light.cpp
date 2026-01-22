#include "classic_detector/light.h"
#include <numeric>
#include <opencv2/imgproc.hpp>

vpie::Light::Light(const std::vector<cv::Point>& contour)
    : cv::RotatedRect(cv::minAreaRect(contour)), color(EnemyColor::WHITE),contour_(contour) {
    center = std::accumulate(contour.begin(), contour.end(), cv::Point2f(0, 0),
                             [n = static_cast<float>(contour.size())](const cv::Point2f& a, const cv::Point& b) {
                                 return a + cv::Point2f(static_cast<float>(b.x), static_cast<float>(b.y)) / n;
                             });

    cv::Point2f p[4];
    this->points(p);
    std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    // Calculate the tilt angle
    // The angle is the angle between the light bar and the horizontal line
    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = static_cast<float>(tilt_angle / CV_PI * 180);
}
