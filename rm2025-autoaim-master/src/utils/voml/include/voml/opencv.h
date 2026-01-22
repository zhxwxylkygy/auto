#pragma once

#include <opencv2/opencv.hpp>

#include <sys/types.h>
#include <voml/details.h>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/traits.hpp>
#include <string>

namespace toml {
template <>
struct from<cv::Mat> {
    template <typename C, template <typename...> class M, template <typename...> class A>
    static cv::Mat from_toml(const basic_value<C, M, A>& v) {
        switch (find<std::string>(v, "dt")[0]) {
            case 'u':
                return voml::Toml2Mat<uchar>(v);
            case 'c':
                return voml::Toml2Mat<char>(v);
            case 'w':
                return voml::Toml2Mat<ushort>(v);
            case 's':
                return voml::Toml2Mat<short>(v);
            case 'i':
                return voml::Toml2Mat<int>(v);
            case 'f':
                return voml::Toml2Mat<float>(v);
            case 'd':
                return voml::Toml2Mat<double>(v);
            default:
                throw voml::CastException{"Unknow dt"};
        }
    }
};

template <>
struct into<cv::Mat> {
    static toml::value into_toml(const cv::Mat& mat) {
        switch (mat.type()) {
            case 0:
                return voml::Mat2Toml<uchar>(mat);
            case 1:
                return voml::Mat2Toml<char>(mat);
            case 2:
                return voml::Mat2Toml<ushort>(mat);
            case 3:
                return voml::Mat2Toml<short>(mat);
            case 4:
                return voml::Mat2Toml<int>(mat);
            case 5:
                return voml::Mat2Toml<float>(mat);
            case 6:
                return voml::Mat2Toml<double>(mat);
            default:
                throw voml::CastException{"Unknow type"};
        }
    }
};

template <typename Tp>
struct from<cv::Size_<Tp>> {
    template <typename C, template <typename...> class M, template <typename...> class A>
    static cv::Size_<Tp> from_toml(const basic_value<C, M, A>& v) {
        switch (find<std::string>(v, "dt")[0]) {
            case 'u':
                return voml::Toml2Size<uchar>(v);
            case 'c':
                return voml::Toml2Size<char>(v);
            case 'w':
                return voml::Toml2Size<ushort>(v);
            case 's':
                return voml::Toml2Size<short>(v);
            case 'i':
                return voml::Toml2Size<int>(v);
            case 'f':
                return voml::Toml2Size<float>(v);
            case 'd':
                return voml::Toml2Size<double>(v);
            default:
                throw voml::CastException{"Unknow dt"};
        }
    }
};

template <typename Tp>
struct into<cv::Size_<Tp>> {
    static toml::value into_toml(const cv::Size_<Tp>& size) { return voml::Size2Toml<Tp>(size); }
};

template <typename Tp>
struct from<cv::Point_<Tp>> {
    template <typename C, template <typename...> class M, template <typename...> class A>
    static cv::Point_<Tp> from_toml(const basic_value<C, M, A>& v) {
        switch (find<std::string>(v, "dt")[0]) {
            case 'u':
                return voml::Toml2Point<uchar>(v);
            case 'c':
                return voml::Toml2Point<char>(v);
            case 'w':
                return voml::Toml2Point<ushort>(v);
            case 's':
                return voml::Toml2Point<short>(v);
            case 'i':
                return voml::Toml2Point<int>(v);
            case 'f':
                return voml::Toml2Point<float>(v);
            case 'd':
                return voml::Toml2Point<double>(v);
            default:
                throw voml::CastException{"Unknow dt"};
        }
    }
};

template <typename Tp>
struct into<cv::Point_<Tp>> {
    static toml::value into_toml(const cv::Point_<Tp>& point) { return voml::Point2Toml(point); }
};

template <typename Tp>
struct from<cv::Point3_<Tp>> {
    template <typename C, template <typename...> class M, template <typename...> class A>
    static cv::Point3_<Tp> from_toml(const basic_value<C, M, A>& v) {
        switch (find<std::string>(v, "dt")[0]) {
            case 'u':
                return voml::Toml2Point3<uchar>(v);
            case 'c':
                return voml::Toml2Point3<char>(v);
            case 'w':
                return voml::Toml2Point3<ushort>(v);
            case 's':
                return voml::Toml2Point3<short>(v);
            case 'i':
                return voml::Toml2Point3<int>(v);
            case 'f':
                return voml::Toml2Point3<float>(v);
            case 'd':
                return voml::Toml2Point3<double>(v);
            default:
                throw voml::CastException{"Unknow dt"};
        }
    }
};

template <typename Tp>
struct into<cv::Point3_<Tp>> {
    static toml::value into_toml(const cv::Point3_<Tp>& point) { return voml::Point32Toml(point); }
};
}  // namespace toml