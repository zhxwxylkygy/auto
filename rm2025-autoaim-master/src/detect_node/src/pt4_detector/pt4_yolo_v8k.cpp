
#include "detect_node/pt4_detector/pt4_yolo_v8k.h"
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <string>

Pt4YoloV8K::Pt4YoloV8K(const Config& config) {
    this->confidence_threshold_ = config.confidence_threshold;
    this->nms_threshold_ = config.nms_threshold;
    this->score_threshold_ = config.score_threshold;
    this->input_width_ = config.input_width;
    this->input_Height_ = config.input_height;
    this->onnx_path_ = config.onnx_path;
    this->InitModule();
}

std::vector<Pt4YoloV8K::Armor> Pt4YoloV8K::Run(const cv::Mat& frame, bool enable_debug_show) {
    PreProcessImg(frame);
    //auto start = std::chrono::high_resolution_clock::now();   // 记录结束时间
    infer_request_.start_async();
    infer_request_.wait();
    //auto end = std::chrono::high_resolution_clock::now();   // 记录结束时间
    //std::chrono::duration<double, std::milli> infer_time = end - start;
    //std::cout << infer_time.count() << " ms" << std::endl;
    const ov::Tensor& output_tensor = infer_request_.get_output_tensor();
    ov::Shape output_shape = output_tensor.get_shape();

    auto* detections = output_tensor.data<float>();
    //    std::cout << sizeof(output_tensor.data<float>()) << std::endl;

    auto output =
        this->PostPrecessImg(detections, output_shape, frame, enable_debug_show);  // montana: 这步加工了返回值

    return output;
}

void Pt4YoloV8K::InitModule() {
    ov::Core core;  // openvino
//    core.set_property("CPU", ov::streams::num(ov::streams::AUTO));
    core.set_property("CPU", ov::streams::num(ov::streams::NUMA));
    std::shared_ptr<ov::Model> model = core.read_model(this->onnx_path_);//(this->xml_path, this->bin_path);
    ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
    ppp.input()
        .tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::RGB);
    ppp.input()
        .preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({255, 255, 255});  // .scale({ 112, 112, 112 });
    ppp.input().model().set_layout("NCHW");
    ppp.output().tensor().set_element_type(ov::element::f32);
    model = ppp.build();

    this->compiled_model_ = core.compile_model(model, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));//);//
    this->infer_request_ = compiled_model_.create_infer_request();
}

void Pt4YoloV8K::PreProcessImg(const cv::Mat& frame) {
    float width = frame.cols;
    float height = frame.rows;
    cv::Size new_shape = cv::Size(input_width_, input_Height_);
    auto r = float(new_shape.width / std::max(width, height));
    int new_unpadW = int(std::round(width * r));
    int new_unpadH = int(std::round(height * r));

    cv::resize(frame, resize_.resized_image, cv::Size(new_unpadW, new_unpadH), 0, 0, cv::INTER_AREA);
    resize_.resized_image = resize_.resized_image;
    resize_.dw = new_shape.width - new_unpadW;
    resize_.dh = new_shape.height - new_unpadH;
    cv::Scalar color = cv::Scalar(100, 100, 100);
    cv::copyMakeBorder(resize_.resized_image, resize_.resized_image, 0, resize_.dh, 0, resize_.dw, cv::BORDER_CONSTANT,
                       color);
    cv::cvtColor(resize_.resized_image, resize_.resized_image, cv::COLOR_BGR2RGB);

    this->resized_image_ratio_x = (float)frame.cols / (float)(resize_.resized_image.cols - resize_.dw);
    this->resized_image_ratio_y = (float)frame.rows / (float)(resize_.resized_image.rows - resize_.dh);
    auto* input_data = (float*)resize_.resized_image.data;
    input_tensor_ =
        ov::Tensor(compiled_model_.input().get_element_type(), compiled_model_.input().get_shape(), input_data);
    infer_request_.set_input_tensor(input_tensor_);
}

std::vector<Pt4YoloV8K::Armor> Pt4YoloV8K::PostPrecessImg(float* detections,
                                                        ov::Shape& output_shape,
                                                        const cv::Mat input_image,
                                                        bool enable_debug_show) {
    std::vector<cv::Rect> boxes;
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Point> kpt_point;
    std::vector<std::vector<cv::Point>> kpts;
    int out_rows = output_shape[1];
    int out_cols = output_shape[2];
    const cv::Mat det_output(out_rows, out_cols, CV_32F, (float*)detections);

    for (int i = 0; i < det_output.cols; ++i) {
        const cv::Mat classes_scores = det_output.col(i).rowRange(4, 28);
        cv::Point class_id;
        double max_class_score;
        cv::minMaxLoc(classes_scores, nullptr, &max_class_score, nullptr, &class_id);
        if (max_class_score > 0.3) {
            const float cx = det_output.at<float>(0, i);
            const float cy = det_output.at<float>(1, i);
            const float ow = det_output.at<float>(2, i);
            const float oh = det_output.at<float>(3, i);
            cv::Rect box;
            box.x = static_cast<int>((cx - 0.5 * ow) );
            box.y = static_cast<int>((cy - 0.5 * oh));
            box.width = static_cast<int>(ow);
            box.height = static_cast<int>(oh);

            std::vector<float> landmark{};
                for (int k = 28; k < 36; k += 2) {
                    landmark.emplace_back(det_output.at<float>(k, i));
                    landmark.emplace_back(det_output.at<float>(k + 1, i));
                }
                for (int k = 0; k < 8; k += 2) {
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
    // std::cout << " 1   " << std::endl;
    // for(int q = 0 ; q < kpts.size() ; q++ ){
    // std::cout << kpts[q] << std::endl;
    // }
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, this->score_threshold_, this->nms_threshold_, nms_result);

    //_____________________________________________________________________________
    std::vector<Armor> output;
    for (int i = 0; i < nms_result.size(); i++) {
        Armor result;
        int idx = nms_result[i];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        boxes[idx].x = this->resized_image_ratio_x * boxes[idx].x;
        boxes[idx].y = this->resized_image_ratio_y * boxes[idx].y;
        boxes[idx].width = this->resized_image_ratio_x * boxes[idx].width;
        boxes[idx].height = this->resized_image_ratio_y * boxes[idx].height;
        result.box = boxes[idx];
        result.kpt = kpts[idx];
        output.push_back(result);
    }

    //______________________________________________________________________________

    if (enable_debug_show) {
        cv::Mat debug_image;
        input_image.copyTo(debug_image);
        for (int i = 0; i < output.size(); i++) {
            auto detection = output[i];
            auto box = detection.box;
            auto classId = detection.class_id;
            // if (classId != 0) continue;
            auto confidence = detection.confidence;
            auto Point = detection.kpt;

            // box.x = this->resized_image_ratio_x * box.x;
            // box.y = this->resized_image_ratio_y * box.y;
            // box.width = this->resized_image_ratio_x * box.width;
            // box.height = this->resized_image_ratio_y * box.height;
            float xmax = box.x + box.width;
            float ymax = box.y + box.height;

            cv::Scalar color = cv::Scalar(color_list_[classId][0], color_list_[classId][1], color_list_[classId][2]);
            float c_mean = cv::mean(color)[0];
            cv::Scalar txt_color;
            if (c_mean > 0.5) {
                txt_color = cv::Scalar(0, 0, 0);
            } else {
                txt_color = cv::Scalar(255, 255, 255);
            }
            cv::rectangle(debug_image, cv::Point(box.x, box.y), cv::Point(xmax, ymax), color * 255, 2);
            for (int i = 0; i < 4; i++) {
                circle(debug_image, Point[i], 3, cv::Scalar(0, 255, 0), -1);
            }
            int baseLine = 0;
            char text[256];
            sprintf(text, "%s %0.1f%%", coco_name_[classId].c_str(), confidence * 100);
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
            cv::Scalar txt_bk_color = color * 0.7 * 255;
            cv::rectangle(debug_image,
                          cv::Rect(cv::Point(box.x, box.y), cv::Size(label_size.width, label_size.height + baseLine)),
                          txt_bk_color, -1);
            cv::putText(debug_image, text, cv::Point(box.x, box.y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        txt_color, 1);
        }
        cv::imshow("pt4 debug", debug_image);
    }
    return output;
}
