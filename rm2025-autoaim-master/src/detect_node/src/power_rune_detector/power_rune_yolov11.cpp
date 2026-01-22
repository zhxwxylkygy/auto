#include "detect_node/power_rune_detector/power_rune_yolov11.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <ostream>
#include <fstream>

std::vector<PowerRuneYOLOv11::Fan> PowerRuneYOLOv11::Run(const cv::Mat& frame, bool enable_debug_show) {
    PreProcessImg(frame); // 预处理

    //auto start = std::chrono::high_resolution_clock::now(); // 记录开始时间
    infer_request_.start_async();                           // 开始推理
    infer_request_.wait();                                  // 等待推理结束
    //auto end = std::chrono::high_resolution_clock::now();   // 记录结束时间
    //std::chrono::duration<double, std::milli> infer_time = end - start;
    //std::cout << infer_time.count() << " ms" << std::endl;
    // std::ofstream txtFile("../output.txt",
    //                       std::ios::app); // 使用std::ios::app以追加模式打开文件
    // txtFile << infer_time.count() << std::endl;
    // txtFile.close();
  
    const ov::Tensor &output_tensor =
        infer_request_.get_output_tensor();             // 推理结果张量
    ov::Shape output_shape = output_tensor.get_shape(); // 结果形状
    auto *detections = output_tensor.data<float>();     // 结果数据
  
    auto output =
        this->PostPrecessImg(detections, output_shape, frame,
                             enable_debug_show); // montana: 这步加工了返回值
    return output;
}

void PowerRuneYOLOv11::InitModule() {
    ov::Core core; // 初始化vino核心
  core.set_property("CPU", ov::streams::num(ov::streams::NUMA));
  // 读取模型
  std::shared_ptr<ov::Model> model = core.read_model(this->model_path_);
  input_height_ = model->input().get_shape()[2];
  input_width_ = model->input().get_shape()[3];
  std::cout << "model input: " << model->input() << std::endl;
  std::cout << "model output: " << model->output() << std::endl;

  // 模型输入输出/预处理后处理配置
  ov::preprocess::PrePostProcessor ppp =
      ov::preprocess::PrePostProcessor(model);

  ppp.input()
      .tensor()
      .set_element_type(ov::element::u8) // 设置输入张量的数据类型为 u8
      .set_layout("NHWC")                // 设置输入张量的布局
      .set_color_format(
          ov::preprocess::ColorFormat::BGR); // opencv输入图像为BGR

  ppp.input()
      .preprocess()
      .convert_element_type(ov::element::f16)
      .convert_color(
          ov::preprocess::ColorFormat::RGB) // 输入图像的颜色格式转换为RGB
      .convert_layout("NCHW")               // 将输入的布局转为 NCHW
      .scale(
          {255, 255,
           255}); // 对输入数据进行缩放，将像素值从范围 [0, 255] 缩放到 [0, 1]

  ppp.output().tensor().set_element_type(
      ov::element::f32); // 设置输出张量的数据类型为 f32（32位浮点数）

  std::cout << ppp << std::endl;
  model = ppp.build();

  // 将编译后的模型对象存储在类的成员变量 compiled_model_ 中
  this->compiled_model_ = core.compile_model(
      model, "CPU",
      ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

  // 将创建的推理请求对象存储在类的成员变量 infer_request_ 中
  this->infer_request_ = compiled_model_.create_infer_request();
}

void PowerRuneYOLOv11::PreProcessImg(const cv::Mat &frame) {
    float width = frame.cols;
    float height = frame.rows;
    cv::Size new_shape = cv::Size(input_width_, input_height_); // 期望形状
    auto r = float(new_shape.width / std::max(width, height));  // 缩放比例
    int new_unpadW = int(std::round(width * r));
    int new_unpadH = int(std::round(height * r));
  
    // 缩放处理，使用灰色进行填充
    cv::resize(frame, resize_.resized_image, cv::Size(new_unpadW, new_unpadH), 0,
               0, cv::INTER_AREA);
    resize_.resized_image = resize_.resized_image;
    resize_.dw = new_shape.width - new_unpadW;
    resize_.dh = new_shape.height - new_unpadH;
    cv::Scalar color = cv::Scalar(100, 100, 100);
    cv::copyMakeBorder(resize_.resized_image, resize_.resized_image, 0,
                       resize_.dh, 0, resize_.dw, cv::BORDER_CONSTANT, color);
  
    // 缩放比例（用于推理结果还原）
    this->resized_image_ratio_x =
        (float)frame.cols / (float)(resize_.resized_image.cols - resize_.dw);
    this->resized_image_ratio_y =
        (float)frame.rows / (float)(resize_.resized_image.rows - resize_.dh);
  
    auto *input_data =
        (float *)resize_.resized_image.data; // 最终输入模型的数据指针
    // 构建输入张量，并添加到推理请求中
    input_tensor_ = ov::Tensor(compiled_model_.input().get_element_type(),
                               compiled_model_.input().get_shape(), input_data);
    infer_request_.set_input_tensor(input_tensor_);
}


std::vector<PowerRuneYOLOv11::Fan> PowerRuneYOLOv11::PostPrecessImg(float* detections,
                                                                  ov::Shape& output_shape,
                                                                  const cv::Mat input_image,
                                                                  bool enable_debug_show) {
    std::vector<cv::Rect> boxes;              // 框
    std::vector<int> class_ids;               // 类
    std::vector<float> confidences;           // 置信度
    std::vector<cv::Point> kpt_point;         // 单个关键点
    std::vector<std::vector<cv::Point>> kpts; // 关键点们
  
    // 根据输出结果创建矩阵
    int out_rows = output_shape[1];
    int out_cols = output_shape[2];
    const cv::Mat det_output(out_rows, out_cols, CV_32F, (float *)detections);
  
    for (int i = 0; i < det_output.cols; ++i) {
      const cv::Mat classes_scores = det_output.col(i).rowRange(4, 6);
      cv::Point class_id;
      double max_class_score;
      cv::minMaxLoc(classes_scores, nullptr, &max_class_score, nullptr,
                    &class_id);
      if (max_class_score > 0.3) {
        const float cx = det_output.at<float>(0, i);
        const float cy = det_output.at<float>(1, i);
        const float ow = det_output.at<float>(2, i);
        const float oh = det_output.at<float>(3, i);
        cv::Rect box;
        box.x = static_cast<int>((cx - 0.5 * ow));
        box.y = static_cast<int>((cy - 0.5 * oh));
        box.width = static_cast<int>(ow);
        box.height = static_cast<int>(oh);
  
        std::vector<float> landmark{};
        for (int k = 6; k < 16; k += 2) {
          landmark.emplace_back(det_output.at<float>(k, i));
          landmark.emplace_back(det_output.at<float>(k + 1, i));
        }
        for (int k = 0; k < 10; k += 2) {
          landmark[k] = resized_image_ratio_x * landmark[k];
          landmark[k + 1] = resized_image_ratio_y * landmark[k + 1];
        }
  
        boxes.push_back(box);
        for (int k = 0; k < landmark.size(); k += 2) {
          kpt_point.push_back(cv::Point(landmark[k], landmark[k + 1]));
        }
        kpts.push_back(kpt_point);
        class_ids.push_back(class_id.y);
        confidences.push_back(max_class_score);
        kpt_point.clear();
      }
    }
  
    // 非极大值抑制（NMS）
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, this->score_threshold_,
                      this->nms_threshold_, nms_result);
  
    // 遍历NMS结果并构建输出
    std::vector<Fan> output;
    for (int i = 0; i < nms_result.size(); i++) {
        Fan result;
        int idx = nms_result[i];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        boxes[idx].x = this->resized_image_ratio_x * boxes[idx].x;
        boxes[idx].y = this->resized_image_ratio_y * boxes[idx].y;
        boxes[idx].width = this->resized_image_ratio_x * boxes[idx].width;
        boxes[idx].height = this->resized_image_ratio_y * boxes[idx].height;
        result.box = boxes[idx];
        result.lights_surface = Fan::LightSurface{kpts[0][0], kpts[0][1], kpts[0][2], kpts[0][3], kpts[0][4]};
        output.push_back(result);
    }

    if (enable_debug_show) {
      cv::Mat debug_image;
      input_image.copyTo(debug_image);
      for (int i = 0; i < output.size(); i++) {
        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;
        // if (classId != 0) continue;
        auto confidence = detection.confidence;
        std::vector<cv::Point> Point{detection.lights_surface.left, detection.lights_surface.top, detection.lights_surface.center, detection.lights_surface.bottom, detection.lights_surface.right};
  
        // box.x = this->resized_image_ratio_x * box.x;
        // box.y = this->resized_image_ratio_y * box.y;
        // box.width = this->resized_image_ratio_x * box.width;
        // box.height = this->resized_image_ratio_y * box.height;
        float xmax = box.x + box.width;
        float ymax = box.y + box.height;
  
        cv::Scalar color;
        if (classId > 11) {
          color = cv::Scalar(0, 0, 255); // 蓝色
        } else {
          color = cv::Scalar(255, 0, 0); // 红色
        }
  
        cv::Scalar txt_color = cv::Scalar(255, 255, 255);
  
        cv::rectangle(debug_image, cv::Point(box.x, box.y), cv::Point(xmax, ymax),
                      color, 2);
        for (int i = 0; i < 4; i++) {
          circle(debug_image, Point[i], 3, cv::Scalar(0, 255, 0), -1);
        }
        int baseLine = 0;
        char text[256];
        sprintf(text, "%s %0.1f%%", coco_name_[classId].c_str(),
                confidence * 100);
        cv::Size label_size =
            cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
        cv::Scalar txt_bk_color = color * 0.7;
        cv::rectangle(
            debug_image,
            cv::Rect(cv::Point(box.x, box.y),
                     cv::Size(label_size.width, label_size.height + baseLine)),
            txt_bk_color, -1);
        cv::putText(debug_image, text,
                    cv::Point(box.x, box.y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);
      }
      cv::imshow("power_rune_v11 debug", debug_image);
    }
  
    return output;
}

