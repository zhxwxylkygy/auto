#pragma once

#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <openvino/openvino.hpp>
#include <string>

class YoloPoseDetector {
public:
  // 模型配置
  struct Config {
    float confidence_threshold; // 置信度
    float nms_threshold;        // NMS阈值
    float score_threshold;      // 分数阈值
    std::string model_path;     // 模型路径
  };
  // 结果结构
  struct Armor {
    int class_id;
    float confidence;
    cv::Rect box;
    std::vector<cv::Point> kpt;
  };

  // 构造函数
  YoloPoseDetector() = delete;
  explicit YoloPoseDetector(const Config &config)
      : confidence_threshold_(config.confidence_threshold),
        nms_threshold_(config.nms_threshold),
        score_threshold_(config.score_threshold),
        model_path_(config.model_path) {
    InitModule();
  }

  std::vector<YoloPoseDetector::Armor> Run(const cv::Mat &frame,
                                           bool enable_debug_show);

private:
  // 缩放图像
  struct Resize {
    cv::Mat resized_image;
    int dw;
    int dh;
  };

  std::array<std::string, 24> coco_name_ = {
      "BG", "B1",  "B2",  "B3",  "B4", "B5",  "BO",  "Bs",
      "Bb", "BL3", "BL4", "BL5", "RG", "R1",  "R2",  "R3",
      "R4", "R5",  "RO",  "Rs",  "Rb", "RL3", "RL4", "RL5"};

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
  std::vector<YoloPoseDetector::Armor>
  PostPrecessImg(float *detections, ov::Shape &output_shape,
                 cv::Mat input_image,
                 bool enable_debug_show); // 后处理
};