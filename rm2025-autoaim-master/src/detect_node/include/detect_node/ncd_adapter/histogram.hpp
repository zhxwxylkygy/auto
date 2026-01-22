#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

cv::Mat CalcHist(const cv::Mat& image) {
    cv::Mat image_gray, hist;
    cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);  // 灰度化
    imshow(" image_gray", image_gray);                // 显示灰度图像

    // 获取图像直方图
    int histsize = 256;
    float ranges[] = {0, 256};
    const float* histRanges = {ranges};
    calcHist(&image_gray, 1, 0, cv::Mat(), hist, 1, &histsize, &histRanges, true, false);

    // 创建直方图显示图像
    int hist_h = 300;                                                 // 直方图的图像的高
    int hist_w = 512;                                                 // 直方图的图像的宽
    int bin_w = hist_w / histsize;                                    // 直方图的等级
    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));  // 绘制直方图显示的图像

    // 绘制并显示直方图
    normalize(hist, hist, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());  // 归一化直方图
    for (int i = 1; i < histsize; i++) {
        line(histImage, cv::Point((i - 1) * bin_w, hist_h - cvRound(hist.at<float>(i - 1))),
             cv::Point((i)*bin_w, hist_h - cvRound(hist.at<float>(i))), cv::Scalar(255, 0, 0), 2, 8, 0);
    }
    return histImage;
}

auto AvgGrey(const cv::Mat& image) {
    cv::Mat image_gray;
    cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);  // 灰度化
    return cv::sum(image_gray)[0] / image.rows / image.cols;
}