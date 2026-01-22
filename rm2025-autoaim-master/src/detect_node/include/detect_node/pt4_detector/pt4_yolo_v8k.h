#pragma once

#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <openvino/openvino.hpp>
#include <string>

class Pt4YoloV8K {
   public:
    struct Config {
        float confidence_threshold;
        float nms_threshold;
        float score_threshold;
        int input_width;
        int input_height;
        std::string onnx_path;
    };
    struct Armor {
        int class_id;
        float confidence;
        cv::Rect box;
        std::vector<cv::Point> kpt;
    };

    Pt4YoloV8K() = delete;

    explicit Pt4YoloV8K(const Config& config);

    std::vector<Pt4YoloV8K::Armor> Run(const cv::Mat& frame, bool enable_debug_show);

   private:
    struct Resize {
        cv::Mat resized_image;
        int dw;
        int dh;
    };

    float color_list_[80][3] = {
        {0.000, 0.447, 0.741}, {0.850, 0.325, 0.098}, {0.929, 0.694, 0.125}, {0.494, 0.184, 0.556},
        {0.466, 0.674, 0.188}, {0.301, 0.745, 0.933}, {0.635, 0.078, 0.184},
    };

    std::array<std::string, 24> coco_name_ = {"BG", "B1",  "B2",  "B3",  "B4", "B5",  "BO",  "Bs",
                                              "Bb", "BL3", "BL4", "BL5", "RG", "R1",  "R2",  "R3",
                                              "R4", "R5",  "RO",  "Rs",  "Rb", "RL3", "RL4", "RL5"};

    // const char* coconame[] = {
    //     "BG", "B1", "B2", "B3", "B4", "B5", "BO", "Bs", "Bb", "RG", "R1", "R2", "R3", "R4", "R5", "RO", "Rs", "Rb",
    // };

    void InitModule();

    float confidence_threshold_{};
    float nms_threshold_{};
    float score_threshold_{};
    int input_width_{};
    int input_Height_{};
    float resized_image_ratio_x{};  // the width ratio of original image and resized image
    float resized_image_ratio_y{};  // the height ratio of original image and resized image
    std::string onnx_path_{};
    Resize resize_{};
    ov::Tensor input_tensor_{};
    ov::InferRequest infer_request_{};
    ov::CompiledModel compiled_model_{};

    void PreProcessImg(const cv::Mat& frame);

    std::vector<Pt4YoloV8K::Armor> PostPrecessImg(float* detections,
                                                  ov::Shape& output_shape,
                                                  cv::Mat input_image,
                                                  bool enable_debug_show);
};