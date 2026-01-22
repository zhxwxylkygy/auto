#include <bits/chrono.h>
#include <classic_detector/entrance.h>
#include <fmt/core.h>
#include <opencv2/core/hal/interface.h>
#include <algorithm>
#include <execution>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logger.hpp>
#include "classic_detector/common.h"
#include "classic_detector/detector.h"
#include "filesystem"
#include "fmt/chrono.h"
#include "histogram.hpp"
#include "detect_node/interface/i_detector.h"
#include "detect_node/interface/interface.h"
#include "detect_node/interface/rm_interface.h"
#include "param_loader.hpp"

namespace vpie {
class NewClassicDetector : public FP::IDetector {
    struct DebugParams {
        bool is_enable;
        bool is_bright_hist_image;
        bool is_binary_image;
        bool is_classifier_image;
        bool is_result_image;
        bool is_classifier_result_log;
        bool is_size_corrector_log;
        bool is_time_log;
        bool is_save_classifier_dataset;
    };
    DebugParams debug_params{};
    std::shared_ptr<vpie::Detector> detector_{};
    std::filesystem::path ds_dir{};
    bool is_log_time;

    void InitDatasetSave() {
        auto root_path = std::filesystem::current_path().parent_path() / "ncd_dataset";

        if (!std::filesystem::exists(root_path)) {
            std::filesystem::create_directory(root_path);
        }
        using namespace std::chrono_literals;
        auto ds_dir_name = fmt::format("{:%Y-%m-%d_%H:%M:%S}", std::chrono::system_clock::now() + 8h);
        ds_dir = root_path / ds_dir_name;
        if (!std::filesystem::exists(ds_dir)) {
            std::filesystem::create_directory(ds_dir);
        }
    }

    void SaveDataset() {
        static long long index = 0;
        for (const auto& armor : detector_->armors_) {
            cv::imwrite(ds_dir / fmt::format("{}-{}.png", armor.number, index++), armor.number_img);
        }
    }

   public:
    NewClassicDetector() = delete;

    explicit NewClassicDetector(RM::Color input_color) {
        vpie::Detector::LightParams l_params {
            .binary_thres = ParamLoader::GetInstance().GetParam<int>("ncd", "light", "binary_thresh"),
            .min_ratio = ParamLoader::GetInstance().GetParam<double>("ncd", "light", "min_ratio"),
            .max_ratio = ParamLoader::GetInstance().GetParam<double>("ncd", "light", "max_ratio"),
            .max_angle = ParamLoader::GetInstance().GetParam<double>("ncd", "light", "max_angle"),
            .color_diff_thresh = ParamLoader::GetInstance().GetParam<int>("ncd", "light", "color_diff_thresh"),
            .min_pixel_to_edge = ParamLoader::GetInstance().GetParam<int>("ncd", "light", "min_pixel_to_edge")
        };
        vpie::Detector::ArmorParams a_params{
            .min_light_ratio = ParamLoader::GetInstance().GetParam<double>("ncd", "armor", "min_light_ratio"),
            .min_small_center_distance =
                ParamLoader::GetInstance().GetParam<double>("ncd", "armor", "min_small_center_distance"),
            .max_small_center_distance =
                ParamLoader::GetInstance().GetParam<double>("ncd", "armor", "small_large_center_threshold"),
            .min_large_center_distance =
                ParamLoader::GetInstance().GetParam<double>("ncd", "armor", "small_large_center_threshold"),
            .max_large_center_distance =
                ParamLoader::GetInstance().GetParam<double>("ncd", "armor", "max_large_center_distance"),
            .max_angle = ParamLoader::GetInstance().GetParam<double>("ncd", "armor", "max_angle"),
        };
        vpie::NumberClassifier::Params nc_params{
            .blur_kernel_size = ParamLoader::GetInstance().GetParam<int>("ncd", "classifier", "blur_kernel_size"),
            .adaptive_binary_kernel_size =
                ParamLoader::GetInstance().GetParam<int>("ncd", "classifier", "adaptive_binary_kernel_size"),
            .model_path = ParamLoader::GetInstance().GetParam<std::string>("ncd", "classifier", "model_path"),
            .label_path = ParamLoader::GetInstance().GetParam<std::string>("ncd", "classifier", "label_path"),
            .confidence_threshold =
                ParamLoader::GetInstance().GetParam<double>("ncd", "classifier", "confidence_threshold"),
            .ignore_classes =
                ParamLoader::GetInstance().GetParam<std::vector<std::string>>("ncd", "classifier", "ignore_class"),
            .enable_size_corrector =
                ParamLoader::GetInstance().GetParam<bool>("ncd", "classifier", "enable_size_corrector"),
            .use_bilateral_filter =
                ParamLoader::GetInstance().GetParam<bool>("ncd", "classifier", "use_bilateral_filter"),
        };
        debug_params = DebugParams{
            .is_enable = ParamLoader::GetInstance().GetParam<bool>("ncd", "debug", "enable"),
            .is_bright_hist_image = ParamLoader::GetInstance().GetParam<bool>("ncd", "debug", "bright_hist_image"),
            .is_binary_image = ParamLoader::GetInstance().GetParam<bool>("ncd", "debug", "binary_image"),
            .is_classifier_image = ParamLoader::GetInstance().GetParam<bool>("ncd", "debug", "classifier_image"),
            .is_result_image = ParamLoader::GetInstance().GetParam<bool>("ncd", "debug", "result_image"),
            .is_classifier_result_log =
                ParamLoader::GetInstance().GetParam<bool>("ncd", "debug", "classifier_result_log"),
            .is_size_corrector_log = ParamLoader::GetInstance().GetParam<bool>("ncd", "debug", "size_corrector_log"),
            .is_time_log = ParamLoader::GetInstance().GetParam<bool>("ncd", "debug", "time_log"),
            .is_save_classifier_dataset =
                ParamLoader::GetInstance().GetParam<bool>("ncd", "debug", "save_classifier_dataset")};

        if (debug_params.is_enable && debug_params.is_save_classifier_dataset) {
            InitDatasetSave();
        }
        
        EnemyColor color = input_color == RM::Color::BLUE  ? vpie::EnemyColor::BLUE
                           : input_color == RM::Color::RED ? vpie::EnemyColor::RED
                                                           : vpie::EnemyColor::WHITE;

        detector_ = initDetector(color, l_params, a_params, nc_params);

        is_log_time = ParamLoader::GetInstance().GetParam<bool>("DETECT", "DEBUG");
    }

    void Debug(cv::Mat image) {
        if (debug_params.is_bright_hist_image)
            cv::imshow("hist", CalcHist(image));
        if (debug_params.is_binary_image)
            cv::imshow("binary", detector_->binary_img);

        if (debug_params.is_classifier_image) {
            for (auto&& it = detector_->armors_.cbegin(); it != detector_->armors_.cend(); ++it) {
                cv::Mat concat(28, 56, CV_8UC3);
                cv::Mat tmp;
                cv::cvtColor(it->number_img, tmp, cv::COLOR_GRAY2BGR);

                cv::hconcat(it->original_img, tmp, concat);
                cv::copyMakeBorder(concat, concat, 0, 25, 0, 250, cv::BORDER_CONSTANT);
                cv::putText(concat, it->classfication_result, {70, 10}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 255, 255), 2);
                imshow(fmt::format("{}", it - detector_->armors_.cbegin()), concat);
            }
        }

        if (debug_params.is_result_image) {
            detector_->drawResults(image);
            cv::imshow("ncd_result", image);
        }

        if (debug_params.is_classifier_result_log) {
            fmt::print(">>>>>>>>>>\n");
            for (const auto& armor : detector_->armors_) {
                fmt::print("{}: {}\n", armor.classfication_result, armor.confidence);
            }
            fmt::print("<<<<<<<<<<\n");
        }

        if (debug_params.is_time_log) {
            auto time_stamp = detector_->time_stamp;
            fmt::print("{{{{{{{{{{\n");
            fmt::print("ncd: {} ms\n", static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                               time_stamp.after_nc - time_stamp.start)
                                                               .count()) /
                                           1000.);
            fmt::print("pi: {} ms\n", static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                              time_stamp.after_pi - time_stamp.start)
                                                              .count()) /
                                          1000.);
            fmt::print("fl: {} ms\n", static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                              time_stamp.after_fl - time_stamp.after_pi)
                                                              .count()) /
                                          1000.);
            fmt::print("ml: {} ms\n", static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                              time_stamp.after_ml - time_stamp.after_fl)
                                                              .count()) /
                                          1000.);
            fmt::print("nc: {} ms\n", static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                              time_stamp.after_nc - time_stamp.after_ml)
                                                              .count()) /
                                          1000.);
            fmt::print("}}}}}}}}}}\n");
        }

        if (debug_params.is_save_classifier_dataset) {
            SaveDataset();
        }

        if (debug_params.is_size_corrector_log) {
            fmt::print("[[[[[[[[[[\n");
            for (const auto& armor : detector_->armors_) {
                if (armor.is_corrected)
                    fmt::print("{} corrected\n", armor.number);
            }
            fmt::print("]]]]]]]]]]\n");
        }
    }

    std::vector<std::shared_ptr<FP::IFeature>> Run(
        const cv::Mat& img_,
        [[maybe_unused]] const  MCU::Orders& input_order_data,
        cv::Mat& result_img) final {
        tt.after_fl = tt.GetTimePoint();
        auto input_scale = ParamLoader::GetInstance().GetParam<double>("ncd", "input_scale");
        //cv::Mat img;
        //cv::resize(img_, img, {static_cast<int>(img_.cols * input_scale), static_cast<int>(img_.rows * input_scale)}, 0, 0, cv::INTER_AREA);
        tt.after_ml = tt.GetTimePoint();

        auto primal_armors = detectArmors(detector_, img_);

        
        //std::cout << " up " << std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1).count() << std::endl;
        if (debug_params.is_enable) {
            Debug(img_.clone());
        }
        tt.after_nc = tt.GetTimePoint();
        std::vector<std::shared_ptr<FP::IFeature>> armors(primal_armors.size());
        tt.after_pi = tt.GetTimePoint();
        std::transform(std::execution::par_unseq, primal_armors.begin(), primal_armors.end(), armors.begin(),
                       [&](const Armor& primal_armor) -> std::shared_ptr<FP::IFeature> {
                           if (!primal_armor.valid)
                               return nullptr;
                           std::shared_ptr<FP::ArmorInfo> armor = std::make_shared<FP::ArmorInfo>();
                           armor->is_dart_armor = false;
                           armor->image_coord_armor_points.l_top = primal_armor.left_light.top / input_scale;
                           armor->image_coord_armor_points.l_bottom = primal_armor.left_light.bottom / input_scale;
                           armor->image_coord_armor_points.r_top = primal_armor.right_light.top / input_scale;
                           armor->image_coord_armor_points.r_bottom = primal_armor.right_light.bottom / input_scale;

                           armor->armor_id = primal_armor.number == "1"         ? RM::ArmorId::ONE
                                             : primal_armor.number == "2"       ? RM::ArmorId::TWO
                                             : primal_armor.number == "3"       ? RM::ArmorId::THREE
                                             : primal_armor.number == "4"       ? RM::ArmorId::FOUR
                                             : primal_armor.number == "5"       ? RM::ArmorId::FIVE
                                             : primal_armor.number == "outpost" ? RM::ArmorId::OUTPOST
                                             : primal_armor.number == "sentry"  ? RM::ArmorId::SENTRY
                                             : primal_armor.number == "base"    ? RM::ArmorId::BASE
                                                                                : RM::ArmorId::INVALID;

                           armor->armor_size = primal_armor.type == vpie::ArmorType::SMALL   ? RM::ArmorSize::SMALL
                                               : primal_armor.type == vpie::ArmorType::LARGE ? RM::ArmorSize::LARGE
                                                                                             : RM::ArmorSize::INVALID;
                           return armor;
                       });
        armors.erase(std::remove(armors.begin(), armors.end(), nullptr), armors.end());
        tt.start = tt.GetTimePoint();
        //LogTime();
        return armors;
    }
    
    Detector::TimeStamp tt;
    void LogTime() const{
        if (is_log_time) {
            auto time_stamp = tt;
            fmt::print(" resize : {} ms\n", static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                               time_stamp.after_ml - time_stamp.after_fl)
                                                               .count()) /
                                           1000.);
            fmt::print("detect : {} ms\n", static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                              time_stamp.after_nc - time_stamp.after_ml)
                                                              .count()) /
                                          1000.);
            fmt::print("copy: {} ms\n", static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                              time_stamp.after_pi - time_stamp.after_nc)
                                                              .count()) /
                                          1000.);
            fmt::print("transfrom: {} ms\n", static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                              time_stamp.start - time_stamp.after_pi)
                                                              .count()) /
                                          1000.);
        }
    }
};
}  // namespace vpie