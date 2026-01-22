#pragma once

#include <sys/types.h>
#include <exception>
#include <opencv2/opencv.hpp>

#include <string>
#include <string_view>
#include <type_traits>
#include <vector>
#include <toml.hpp>

namespace voml {
class CastException : public std::exception {
   public:
    explicit CastException(std::string_view content) : content_(content) {}

    [[nodiscard]] const char* what() const noexcept override { return content_.c_str(); }

   private:
    std::string content_{};
};
template <typename T>
cv::Mat Vector2Mat(const std::vector<T>& data, const std::vector<int>& shape) {
    int ndims = static_cast<int>(shape.size());
    if (ndims == 0) {
        throw CastException{"Error: shape must have at least one dimension."};
    }

    if (shape.size() != 2) {
        throw CastException{"Error: all dimensions in shape must be positive."};
    }
    int totalSize = shape[0] * shape[1];

    if (totalSize != data.size()) {
        throw CastException{"Error: data size does not match shape."};
    }

    cv::Mat mat(ndims, shape.data(), cv::DataType<T>::type);
    memcpy(mat.data, data.data(), totalSize * sizeof(T));

    return mat;
}

template <typename T, typename C, template <typename...> class M, template <typename...> class A>
cv::Mat Toml2Mat(const toml::basic_value<C, M, A>& v) {
    int rows = toml::find<int>(v, "rows");
    int cols = toml::find<int>(v, "cols");
    std::vector<int> shape{rows, cols};
    return voml::Vector2Mat(toml::find<std::vector<T>>(v, "data"), shape);
}

template <typename Tp, typename C, template <typename...> class M, template <typename...> class A>
cv::Size_<Tp> Toml2Size(const toml::basic_value<C, M, A>& v) {
    return cv::Size_<Tp>{toml::find<Tp>(v, "width"), toml::find<Tp>(v, "height")};
}

template <typename Tp, typename C, template <typename...> class M, template <typename...> class A>
cv::Point_<Tp> Toml2Point(const toml::basic_value<C, M, A>& v) {
    return cv::Point_<Tp>{toml::find<Tp>(v, "x"), toml::find<Tp>(v, "y")};
}

template <typename Tp, typename C, template <typename...> class M, template <typename...> class A>
cv::Point3_<Tp> Toml2Point3(const toml::basic_value<C, M, A>& v) {
    return cv::Point3_<Tp>{toml::find<Tp>(v, "x"), toml::find<Tp>(v, "y"), toml::find<Tp>(v, "z")};
}
template <typename Tp>
std::string MapCvType() {
    if constexpr (std ::is_same_v<Tp, uchar>) {
        return "u";
    } else if constexpr (std ::is_same_v<Tp, char>) {
        return "c";
    } else if constexpr (std ::is_same_v<Tp, ushort>) {
        return "w";
    } else if constexpr (std ::is_same_v<Tp, short>) {
        return "s";
    } else if constexpr (std ::is_same_v<Tp, int>) {
        return "i";
    } else if constexpr (std ::is_same_v<Tp, float>) {
        return "f";
    } else if constexpr (std ::is_same_v<Tp, double>) {
        return "d";
    } else {
        return "?";
    }
}

template <typename T>
toml::value Mat2Toml(cv::Mat mat) {
    int rows = mat.rows;
    int cols = mat.cols;
    auto totalSize = rows * cols;
    std::vector<T> data(totalSize);
    memcpy(data.data(), mat.data, totalSize * sizeof(T));

    return toml::value{{"rows", rows}, {"cols", cols}, {"dt", MapCvType<T>()}, {"data", data}};
}

template <typename Tp>
toml::value Size2Toml(cv::Size_<Tp> size) {
    Tp width = size.width;
    Tp height = size.height;

    return toml::value{{"width", width}, {"height", height}, {"dt", MapCvType<Tp>()}};
}

template <typename Tp>
toml::value Point2Toml(cv::Point_<Tp> point) {
    Tp x = point.x;
    Tp y = point.y;

    return toml::value{{"x", x}, {"y", y}, {"dt", MapCvType<Tp>()}};
}

template <typename Tp>
toml::value Point32Toml(cv::Point3_<Tp> point) {
    Tp x = point.x;
    Tp y = point.y;
    Tp z = point.z;

    return toml::value{{"x", x}, {"y", y}, {"z", z}, {"dt", MapCvType<Tp>()}};
}

}  // namespace voml