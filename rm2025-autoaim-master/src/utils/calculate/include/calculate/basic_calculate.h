//
// Created by wpie on 23-11-1.
//

#pragma once
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <Eigen/Eigen>

//??umber?????????
template<typename T>
inline constexpr T LoopFormatNumber(const T number, const T min, const T max) {
    while (number > max || number < min) {
        if (number > max)number -= max - min;
        if (number < min)number += max - min;
    }
    return number;
}

//??umber????????
template<typename T>
inline constexpr T FormatNumber(const T number, const T min, const T max) {
    if (number > max || number < min) {
        if (number > max)number -= max - min;
        if (number < min)number += max - min;
    }
    return number;
}

//??????
template<typename T>
inline constexpr T LimitNumber(const T num, const T min, const T max) {
    if (num > max)return max;
    else if (num < min)return min;
    else return num;
}

//???number???????????
template<typename T>
inline constexpr bool IsInRange(const T number, const T min, const T max) {
    if (number >= min && number <= max)return true;
    else return false;
}


/* @function ????????????? */
inline constexpr double Get4PointArea( std::vector<cv::Point2f> points);

/**
 * @brief ??????????????????+9*87 *
 * @param pts ????????
 * @return float ???
 */
inline double CalcTriangleArea(const std::array<cv::Point2f, 3> &pts) {
    auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
    auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
    auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));

    auto p = (a + b + c) / 2.f;

    return sqrt(p * (p - a) * (p - b) * (p - c));
}

/**
 * @brief ???????????
 *
 * @param pts ????????
 * @return float ???
 */
inline double CalcTetragonArea(std::array<cv::Point2f, 4> pts) {
    return CalcTriangleArea({pts[0], pts[1], pts[2]}) +
           CalcTriangleArea({pts[1], pts[2], pts[3]});
}

template<typename T>
concept IsValue = std::is_same_v<T, double> || std::is_same_v<T, int> || std::is_same_v<T, float>;

template<typename T> requires IsValue<T>
constexpr T GetDistance(const T x , const T y, const T z){
    return sqrt(x * x + y * y + z * z);
}
template<typename T>
constexpr T GetDistance(const T x, const T y){
    return sqrt(x * x + y * y);
}
template<typename T>
constexpr T GetYaw(const T x, const T y){
    return atan2(y , x);
}

template<typename T>
constexpr auto GetVec(const T norm, const T yaw){
    cv::Point_<T>output(
            norm * cos(yaw),
            norm * sin(yaw)
    );
    return output;
}

template <typename T>
[[nodiscard]] cv::Point_<T> TransformPoint(const cv::Point_<T> p1, const cv::Point_<T> translation, double rotation) {
    // 1. ???????????????????????
    T translatedX = p1.x - translation.x;
    T translatedY = p1.y - translation.y;

    // 2. ??????????????
    T cosTheta = cos(rotation);
    T sinTheta = sin(rotation);
    T rotatedX = cosTheta * translatedX - sinTheta * translatedY;
    T rotatedY = sinTheta * translatedX + cosTheta * translatedY;

    return {rotatedX, rotatedY};
}

[[nodiscard]] inline Eigen::Vector3d XYZ2DTP(const Eigen::Vector3d &xyz){
    double distance = sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y() + xyz.z() * xyz.z());
    double phi = atan2(xyz.y(), xyz.x());
    double theta= acos(xyz.z() / distance);
    Eigen::Vector3d  output;
    output.x() = distance;
    output.y() = theta;
    output.z() = phi;
    return output;
}
[[nodiscard]] inline Eigen::Vector3d DTP2XYZ(const Eigen::Vector3d &dtp){
    double x = dtp.x() * sin(dtp.y()) * cos(dtp.z());
    double y = dtp.x() * sin(dtp.y()) * sin(dtp.z());
    double z = dtp.x() * cos(dtp.y());
    Eigen::Vector3d output;
    output.x() = x;
    output.y() = y;
    output.z() = z;
    return output;
}

template<typename T>
[[nodiscard]] T GetXOYNorm(cv::Point3_<T> p){
    return GetDistance(p.x, p.y);
}

template<typename T>
[[nodiscard]] T GetXOYYaw(cv::Point3_<T> p){
    return GetYaw(p.x, p.y);
}

template<typename T>
[[nodiscard]] inline bool IsPointInImage(cv::Point_<T> pt, const cv::Mat & input_image){
    return pt.x >= 0 && pt.x <= input_image.cols - 1 && pt.y >= 0 && pt.y <= input_image.rows - 1;
}