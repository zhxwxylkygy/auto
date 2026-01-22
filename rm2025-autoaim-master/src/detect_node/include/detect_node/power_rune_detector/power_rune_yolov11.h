#pragma once

#include<string>
#include<iostream>
#include<opencv2/dnn.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<openvino/openvino.hpp>


class PowerRuneYOLOv11 {
public:
    struct Config {
        float confidence_threshold; // 置信度
        float nms_threshold;        // NMS阈值
        float score_threshold;      // 分数阈值
        std::string model_path;     // 模型路径
    };
    struct Fan {
        struct LightSurface {
            cv::Point left;
            cv::Point top;
            cv::Point center;
            cv::Point bottom;
            cv::Point right;
        } lights_surface;
        cv::Point bull_point;
        int class_id;
        float confidence;
        cv::Rect box;
    };

    PowerRuneYOLOv11() = delete;
    explicit PowerRuneYOLOv11(const Config &config)
        : confidence_threshold_(config.confidence_threshold),
          nms_threshold_(config.nms_threshold),
          score_threshold_(config.score_threshold),
          model_path_(config.model_path) {
      InitModule();
    }
  
    std::vector<PowerRuneYOLOv11::Fan> Run(const cv::Mat &frame,
                                             bool enable_debug_show);

private:
struct Resize {
    cv::Mat resized_image;
    int dw;
    int dh;
  };

  std::array<std::string, 2> coco_name_ = {
      "BLUE", "RED"};

  float confidence_threshold_{};     // 置信度阈值
  float nms_threshold_{};            // NMS阈值
  float score_threshold_{};          // 分数阈值
  int input_width_{};                // 输入图像宽度
  int input_height_{};               // 输入图像高度
  float resized_image_ratio_x{};     // 缩放后图像与原始图像的比例
  float resized_image_ratio_y{};     // 缩放后图像与原始图像的比例
  std::string model_path_{};         // 模型路径
  Resize resize_{};                  // 缩放图像
  ov::Tensor input_tensor_{};        // 模型输入张量
  ov::InferRequest infer_request_{}; // 推理请求
  ov::CompiledModel compiled_model_{}; // 构建模型

  void InitModule();                        // 模型初始化
  void PreProcessImg(const cv::Mat &frame); // 图像预处理
  std::vector<PowerRuneYOLOv11::Fan>
  PostPrecessImg(float *detections, ov::Shape &output_shape,
                 cv::Mat input_image,
                 bool enable_debug_show); // 后处理
};