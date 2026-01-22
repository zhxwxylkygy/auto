//
// Created by wpie on 23-11-1.
//

#pragma once
#include <iostream>
#include "opencv2/opencv.hpp"
#include "interface.h"
#include "mcu.h"

namespace FP{

    class IDetector{
    public:

        IDetector() = default;

        virtual ~IDetector() = default;

        [[nodiscard]] virtual std::vector<std::shared_ptr<IFeature>> Run(const cv::Mat &img, const MCU::MCU::Orders& input_order_data, cv::Mat& result_img) = 0;
    };

}
