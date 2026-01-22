//
// Created by wpie on 23-7-18.
//

#ifndef GXCAMERA_TEST_GXCAMERA_H
#define GXCAMERA_TEST_GXCAMERA_H

#include <unistd.h>
#include <builtin_interfaces/msg/detail/time__struct.hpp>
#include <condition_variable>
#include <cstdint>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <thread>
#include <variant>
#include "inc/DxImageProc.h"
#include "inc/GxIAPI.h"

// Show error message
#define GX_VERIFY(emStatus)              \
    if (emStatus != GX_STATUS_SUCCESS) { \
        GetErrorString(emStatus);        \
        return emStatus == 0 ? 0 : 1;    \
    }

#define GX_VERIFY_WITH_UNINIT(emStatus)  \
    if (emStatus != GX_STATUS_SUCCESS) { \
        GetErrorString(emStatus);        \
        GXCloseLib();                    \
        return emStatus == 0 ? 0 : 1;    \
    }

class GxCamera {
   public:
    using Timestamp = unsigned long int;

    struct FrameData {
        std::shared_ptr<cv::Mat> img;
        std::shared_ptr<Timestamp> timestamp;
        std::shared_ptr<std::mutex> m;
        std::shared_ptr<std::condition_variable> cv;
        std::shared_ptr<sensor_msgs::msg::Image> img_msg;
    };

    enum class ExternalTriggerIndex { OC_LINE0, GPIO_LINE2, GPIO_LINE3 };
    enum class ExternalTriggerActivation { RAISING_EDGE, FALLING_EDGE, ANY_EDGE, LEVEL_HIGH, LEVEL_LOW };

    GxCamera() = default;

    ~GxCamera() = default;

    //       配置部分
    void LoadCameraSN(const char* CameraSN);

    void LoadExposureGainParam(bool AutoExposure,
                               bool AutoGain,
                               double ExposureTime,
                               double AutoExposureTimeMin,
                               double AutoExposureTimeMax,
                               double Gain,
                               double AutoGainMin,
                               double AutoGainMax,
                               int64_t GrayValue);

    void LoadRoiParam(int64_t Width, int64_t Height, int64_t OffsetX, int64_t OffsetY);

    void LoadWhiteBalanceParam(bool WhiteBalanceOn, GX_AWB_LAMP_HOUSE_ENTRY lightSource);

    int InitWithParam();

    int ExternalInitTimestamp();

    //          运行部分
    int Run(FrameData& frame_data, ExternalTriggerIndex eti, ExternalTriggerActivation eta);

    int Run(FrameData& frame_data, std::chrono::milliseconds trigger_time_ms);

    int Run(FrameData& frame_data);

    int ActivateTrigger();

    int Stop();

    static int UnInit();

    static int WaitForFrame(cv::Mat& dst_mat,
                            GxCamera::FrameData& input_frame_data,
                            std::chrono::milliseconds time_out_ms);

    static int WaitForImgmsg(sensor_msgs::msg::Image& img_msg,
                             sensor_msgs::msg::CameraInfo& camera_info,
                             GxCamera::FrameData& input_frame_data,
                             std::chrono::milliseconds time_out_ms);

    [[nodiscard]] bool CanPubulishImg();

   private:
    enum class RunMode { STOP, EXTERNAL_TRIGGER, TIME_SOFTWARE_TRIGGER, BOOL_SOFTWARE_TRIGGER };

    struct RunServer {};
    struct TSTRunServer : RunServer {
        std::condition_variable cv{};
        std::mutex m{};
        bool need_exit = false;
        bool successful_exit = false;
        std::chrono::milliseconds trigger_wait_ms{};
    };
    struct ASTRunServer : RunServer {
        bool has_get = false;
    };
    // maybe use enum class to instead these bool flag variable, it would include a NO_SIGNAL member

    struct Roi {
        int64_t width = 640;    ///< Roi width  640
        int64_t height = 480;   ///< Roi height 480
        int64_t offset_x = 80;  ///< OffsetX 80  和Min的差必须是16的倍数
        int64_t offset_y = 60;  ///< OffsetY 60   和Min的差必须是2的倍数
    };

    struct ExposureGain {
        bool use_auto_exposure = true;  ///< Exposure is auto mode or not
        bool use_auto_gain = true;      ///< Gain is auto mode or not

        double exposure_time = 2000;            ///< 2000us Exposure Time
        double auto_exposure_time_max = 10000;  ///< 5000us Maximum exposure time when using AutoExporsureTime mode
        double auto_exposure_time_min = 500;    ///< 1000us Minimum exposure time when using AutoExporsureTime mode

        double gain = 6;            ///< Gain (Maxium 16dB)
        double auto_gain_max = 10;  ///< Maximum gain when using AutoGain mode
        double auto_gain_min = 5;   ///< Minimum gain when using AutoGain mode

        int64_t gray_value = 200;  ///< Expected gray value
    };

    struct WhiteBalance {
        bool use_white_balance = false;                                     ///< Auto WhiteBalance is applied ?
        GX_AWB_LAMP_HOUSE_ENTRY light_source = GX_AWB_LAMP_HOUSE_ADAPTIVE;  ///< The lamp type of environment
    };

    struct CaptureCallbackUserParam {
        std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>();
        std::shared_ptr<uint64_t> timestamp = std::make_shared<uint64_t>();
        std::shared_ptr<std::mutex> m = std::make_shared<std::mutex>();
        std::shared_ptr<std::condition_variable> cv = std::make_shared<std::condition_variable>();
        std::shared_ptr<sensor_msgs::msg::Image> img_msg = std::make_shared<sensor_msgs::msg::Image>();
    };

    static void GetErrorString(GX_STATUS);

    GX_STATUS SetRoi();

    GX_STATUS SetExposureGain();

    GX_STATUS SetWhiteBalance();

    static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);

    GX_DEV_HANDLE device_handle_ = nullptr;  ///< Device handle
    char* device_sn_ = nullptr;              ///< Device SN number
    ExposureGain exposure_gain_;             ///< Camera exposure and gain param
    Roi roi_;                                ///< Camera roi_ and resolution param
    WhiteBalance white_balance_;             ///< Camera whitebalance param

    CaptureCallbackUserParam capture_callback_user_param_;

    RunMode run_mode_ = RunMode::STOP;

    std::variant<std::monostate, TSTRunServer, ASTRunServer> run_server_var_{};  // 保留更多线程服务结构体的接口
};

#endif  // GXCAMERA_TEST_GXCAMERA_H
