//
// Created by wpie on 23-7-18.
//

#include <camera_node/gx_camera.h>
#include <bits/chrono.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <opencv2/imgproc.hpp>
#include <param_loader.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>
#include <thread>
#include <vector>
#include <camera_node/inc/DxImageProc.h>
#include <fmt/core.h>
#include "opencv2/highgui.hpp"
#include <memory>
/// For client
/// Set camera exposure and gain params
void GxCamera::LoadExposureGainParam(bool AutoExposure,
                                     bool AutoGain,
                                     double ExposureTime,
                                     double AutoExposureTimeMin,
                                     double AutoExposureTimeMax,
                                     double Gain,
                                     double AutoGainMin,
                                     double AutoGainMax,
                                     int64_t GrayValue) {
    exposure_gain_.use_auto_exposure = AutoExposure;
    exposure_gain_.use_auto_gain = AutoGain;
    exposure_gain_.exposure_time = ExposureTime;
    exposure_gain_.auto_exposure_time_min = AutoExposureTimeMin;
    exposure_gain_.auto_exposure_time_max = AutoExposureTimeMax;
    exposure_gain_.gain = Gain;
    exposure_gain_.auto_gain_min = AutoGainMin;
    exposure_gain_.auto_gain_max = AutoGainMax;
    exposure_gain_.gray_value = GrayValue;
}

/// Set camera exposure and gain
GX_STATUS GxCamera::SetExposureGain() {
    GX_STATUS status;

    // Set Exposure
    if (exposure_gain_.use_auto_exposure) {
        status = GXSetEnum(device_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
        GX_VERIFY(status)
        status = GXSetFloat(device_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, exposure_gain_.auto_exposure_time_max);
        GX_VERIFY(status)
        status = GXSetFloat(device_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, exposure_gain_.auto_exposure_time_min);
        GX_VERIFY(status)
    } else {
        status = GXSetEnum(device_handle_, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
        GX_VERIFY(status)
        status = GXSetEnum(device_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
        GX_VERIFY(status)
        status = GXSetFloat(device_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_gain_.exposure_time);
        GX_VERIFY(status)
    }

    // Set Gain
    if (exposure_gain_.use_auto_gain) {
        status = GXSetEnum(device_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
        GX_VERIFY(status)
        status = GXSetFloat(device_handle_, GX_FLOAT_AUTO_GAIN_MAX, exposure_gain_.auto_gain_max);
        GX_VERIFY(status)
        status = GXSetFloat(device_handle_, GX_FLOAT_AUTO_GAIN_MIN, exposure_gain_.auto_gain_min);
        GX_VERIFY(status)
    } else {
        status = GXSetEnum(device_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
        GX_VERIFY(status)
        status = GXSetEnum(device_handle_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        GX_VERIFY(status)
        status = GXSetFloat(device_handle_, GX_FLOAT_GAIN, exposure_gain_.gain);
        GX_VERIFY(status)
    }

    // Set Expected Gray Value
    status = GXSetInt(device_handle_, GX_INT_GRAY_VALUE, exposure_gain_.gray_value);
    GX_VERIFY(status)

    return GX_STATUS_SUCCESS;
}

/// For client
/// Set camera roi_ params
void GxCamera::LoadRoiParam(int64_t Width, int64_t Height, int64_t OffsetX, int64_t OffsetY) {
    roi_.width = Width;
    roi_.height = Height;
    roi_.offset_x = OffsetX;
    roi_.offset_y = OffsetY;
}

/// Set camera roi_
GX_STATUS GxCamera::SetRoi() {
    GX_STATUS status;
    int64_t w, h, sw, sh;
    status = GXGetInt(device_handle_, GX_INT_WIDTH_MAX, &w);
    GX_VERIFY(status)
    status = GXGetInt(device_handle_, GX_INT_HEIGHT_MAX, &h);
    GX_VERIFY(status)
    status = GXGetInt(device_handle_, GX_INT_SENSOR_WIDTH, &sw);
    GX_VERIFY(status)
    status = GXGetInt(device_handle_, GX_INT_SENSOR_HEIGHT, &sh);
    GX_VERIFY(status)

    int64_t last_width, last_height;
    status = GXGetInt(device_handle_, GX_INT_WIDTH, &last_width);
    GX_VERIFY(status)
    status = GXGetInt(device_handle_, GX_INT_HEIGHT, &last_height);
    GX_VERIFY(status)

    if (last_width < roi_.width) {
        status = GXSetInt(device_handle_, GX_INT_OFFSET_X, roi_.offset_x);
        GX_VERIFY(status)
        status = GXSetInt(device_handle_, GX_INT_WIDTH, roi_.width);
        GX_VERIFY(status)
    } else {
        status = GXSetInt(device_handle_, GX_INT_WIDTH, roi_.width);
        GX_VERIFY(status)
        status = GXSetInt(device_handle_, GX_INT_OFFSET_X, roi_.offset_x);
        GX_VERIFY(status)
    }
    if (last_height < roi_.height) {
        status = GXSetInt(device_handle_, GX_INT_OFFSET_Y, roi_.offset_y);
        GX_VERIFY(status)
        status = GXSetInt(device_handle_, GX_INT_HEIGHT, roi_.height);
        GX_VERIFY(status)
    } else {
        status = GXSetInt(device_handle_, GX_INT_HEIGHT, roi_.height);
        GX_VERIFY(status)
        status = GXSetInt(device_handle_, GX_INT_OFFSET_Y, roi_.offset_y);
        GX_VERIFY(status)
    }
    return GX_STATUS_SUCCESS;
}

/// For client
/// Set camera white balance params
void GxCamera::LoadWhiteBalanceParam(bool WhiteBalanceOn, GX_AWB_LAMP_HOUSE_ENTRY lightSource) {
    // 自动白平衡光照环境
    //  GX_AWB_LAMP_HOUSE_ADAPTIVE 自适应
    //  GX_AWB_LAMP_HOUSE_FLUORESCENCE 荧光灯
    //  GX_AWB_LAMP_HOUSE_INCANDESCENT 白炽灯
    //  GX_AWB_LAMP_HOUSE_U30 光源温度3000k
    //  GX_AWB_LAMP_HOUSE_D50 光源温度5000k
    //  GX_AWB_LAMP_HOUSE_D65 光源温度6500k
    //  GX_AWB_LAMP_HOUSE_D70 光源温度7000k
    white_balance_.use_white_balance = WhiteBalanceOn;
    white_balance_.light_source = lightSource;
}

/// Set camera WhiteBalance
GX_STATUS GxCamera::SetWhiteBalance() {
    // 选择白平衡通道
    GX_STATUS status;

    if (white_balance_.use_white_balance) {
        status = GXSetEnum(device_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
        GX_VERIFY(status)
        // status = GXSetEnum(hDevice_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
        // status = GXSetEnum(hDevice_, GX_ENUM_BALANCE_RAT IO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
        status = GXSetInt(device_handle_, GX_INT_AWBROI_OFFSETX, 0);
        GX_VERIFY(status)
        status = GXSetInt(device_handle_, GX_INT_AWBROI_OFFSETY, 0);
        GX_VERIFY(status)
        // 设置自动白平衡感兴趣区域(整个roi)
        status = GXSetInt(device_handle_, GX_INT_AWBROI_WIDTH, roi_.width);
        GX_VERIFY(status)
        status = GXSetInt(device_handle_, GX_INT_AWBROI_HEIGHT, roi_.height);
        GX_VERIFY(status)

        // 自动白平衡设置
        status = GXSetEnum(device_handle_, GX_ENUM_AWB_LAMP_HOUSE, white_balance_.light_source);
        GX_VERIFY(status)

        // 设置连续自动白平衡
        status = GXSetEnum(device_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        GX_VERIFY(status)
    } else {
        status = GXSetEnum(device_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
        GX_VERIFY(status)
    }

    return GX_STATUS_SUCCESS;
}

void GxCamera::LoadCameraSN(const char* CameraSN) {
    device_sn_ = const_cast<char*>(CameraSN);
}

int GxCamera::InitWithParam() {
    printf("GxCamera Initializing......");
    printf("\n\n");

    GX_STATUS status;
    uint32_t ui32DeviceNum = 0;

    // Initialize library
    status = GXInitLib();
    GX_VERIFY(status)
    if (device_sn_ == nullptr) {
        // if(true) {
        // Get device enumerated number
        status = GXUpdateDeviceList(&ui32DeviceNum, 1000);
        GX_VERIFY(status)

        // If no device found, app exit
        if (ui32DeviceNum <= 0) {
            printf("<No device found>\n");
            GXCloseLib();
            return status;
        }

        // Open first device enumerated
        status = GXOpenDeviceByIndex(1, &device_handle_);
        GX_VERIFY_WITH_UNINIT(status)
    } else {
        // InitWithParam OpenParam , Open device in exclusive mode by SN
        GX_OPEN_PARAM stOpenParam;
        stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
        stOpenParam.openMode = GX_OPEN_SN;
        stOpenParam.pszContent = device_sn_;

        // Open device
        status = GXOpenDevice(&stOpenParam, &device_handle_);
        GX_VERIFY_WITH_UNINIT(status)
    }

    // Get Device Info
    printf("***********************************************\n");
    std::cout << "GxCamera start running" << std::endl;
    std::cout << " " << std::endl;

    size_t nSize = 0;
    // Get string length of Model name
    status = GXGetStringLength(device_handle_, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    GX_VERIFY_WITH_UNINIT(status)
    // Alloc memory for Model name
    char* pszModelName = new char[nSize];
    // Get Model name
    status = GXGetString(device_handle_, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    if (status != GX_STATUS_SUCCESS) {
        delete[] pszModelName;
        pszModelName = nullptr;
        GX_VERIFY_WITH_UNINIT(status)
    }

    printf("<Model Name : %s>\n", pszModelName);
    // Release memory for Model name
    delete[] pszModelName;
    pszModelName = nullptr;

    // Get string length of USBHID number
    status = GXGetStringLength(device_handle_, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    GX_VERIFY_WITH_UNINIT(status)
    // Alloc memory for USBHID number
    char* pszSerialNumber = new char[nSize];
    // Get USBHID Number
    status = GXGetString(device_handle_, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    if (status != GX_STATUS_SUCCESS) {
        delete[] pszSerialNumber;
        pszSerialNumber = nullptr;
        GX_VERIFY_WITH_UNINIT(status)
    }
    printf("<USBHID Number : %s>\n", pszSerialNumber);
    // Release memory for USBHID number
    delete[] pszSerialNumber;
    pszSerialNumber = nullptr;

    // Get string length of Device version
    status = GXGetStringLength(device_handle_, GX_STRING_DEVICE_VERSION, &nSize);
    GX_VERIFY_WITH_UNINIT(status)
    char* pszDeviceVersion = new char[nSize];
    // Get Device Version
    status = GXGetString(device_handle_, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    if (status != GX_STATUS_SUCCESS) {
        delete[] pszDeviceVersion;
        pszDeviceVersion = nullptr;
        GX_VERIFY(status)
    }

    printf("<Device Version : %s>\n", pszDeviceVersion);
    // Release memory for Device version
    delete[] pszDeviceVersion;
    pszDeviceVersion = nullptr;
    printf("***********************************************\n");

    // Get the type of Bayer conversion. whether is a color camera.
    bool bColorFilter_;
    status = GXIsImplemented(device_handle_, GX_ENUM_PIXEL_COLOR_FILTER, &bColorFilter_);
    GX_VERIFY_WITH_UNINIT(status)

    // This app only support color cameras
    if (!bColorFilter_) {
        printf("<This app only support color cameras! App Exit!>\n");
        GXCloseDevice(device_handle_);
        device_handle_ = nullptr;
        GXCloseLib();
        return 0;
    }

    //    status = GXGetInt(hDevice_, GX_INT_PAYLOAD_SIZE, &nPayloadSize_);
    //    GX_VERIFY(status)

    status = SetRoi();
    GX_VERIFY_WITH_UNINIT(status)
    status = SetExposureGain();
    GX_VERIFY_WITH_UNINIT(status)
    status = SetWhiteBalance();
    GX_VERIFY_WITH_UNINIT(status)

    return GX_STATUS_SUCCESS;
}

// int GxCamera::ExternalInitTimestamp() {
//     GX_STATUS status;
//     status = GXSetEnum(device_handle_, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);
//     GX_VERIFY(status)
//     status = GXSetEnum(device_handle_, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_INPUT);
//     GX_VERIFY(status)

//     bool GPIO_Line2;
//     auto start_t = std::chrono::high_resolution_clock::now();
//     while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t)
//                    .count() <= 1000 &&
//            GPIO_Line2 == 1) {
//         status = GXGetBool(device_handle_, GX_BOOL_LINE_STATUS, &GPIO_Line2);
//         GX_VERIFY(status)
//     }
//     if (GPIO_Line2 == 1) {
//         std::cout << "camera syn with c board failed!  time out" << std::endl;
//         return false;
//     } else {
//         status = GXSendCommand(device_handle_, GX_COMMAND_TIMESTAMP_LATCH);
//         GX_VERIFY(status)
//     }
//     std::cout << "camera syn with c board successful" << std::endl;

//     return GX_STATUS_SUCCESS;
// }

int GxCamera::Run(FrameData& frame_data, ExternalTriggerIndex eti, ExternalTriggerActivation eta) {
    run_mode_ = RunMode::EXTERNAL_TRIGGER;
    capture_callback_user_param_.img->create(static_cast<int>(roi_.height), static_cast<int>(roi_.width), CV_8UC3);
    frame_data.cv = capture_callback_user_param_.cv;
    frame_data.img = capture_callback_user_param_.img;
    frame_data.m = capture_callback_user_param_.m;
    frame_data.timestamp = capture_callback_user_param_.timestamp;
    frame_data.img_msg = capture_callback_user_param_.img_msg;
    capture_callback_user_param_.img_msg->encoding = "rgb8";
    capture_callback_user_param_.img_msg->width = static_cast<int>(roi_.width);
    capture_callback_user_param_.img_msg->height = static_cast<int>(roi_.height);
    capture_callback_user_param_.img_msg->is_bigendian = false;
    capture_callback_user_param_.img_msg->step = static_cast<int>(roi_.width) * 1 * 3;
    capture_callback_user_param_.img_msg->data.resize(static_cast<int>(roi_.width) * static_cast<int>(roi_.height)  *
                                                      sizeof(uint8_t));

    GX_STATUS status;
    int gx_line_selector;
    int gx_trigger_source_line;
    int gx_trigger_activation;
    switch (eti) {
        case ExternalTriggerIndex::OC_LINE0: {
            gx_line_selector = GX_ENUM_LINE_SELECTOR_LINE0;
            gx_trigger_source_line = GX_TRIGGER_SOURCE_LINE0;
            break;
        }
        case ExternalTriggerIndex::GPIO_LINE2: {
            gx_line_selector = GX_ENUM_LINE_SELECTOR_LINE2;
            gx_trigger_source_line = GX_TRIGGER_SOURCE_LINE2;
            break;
        }
        case ExternalTriggerIndex::GPIO_LINE3: {
            gx_line_selector = GX_ENUM_LINE_SELECTOR_LINE3;
            gx_trigger_source_line = GX_TRIGGER_SOURCE_LINE3;
            break;
        }
    }

    switch (eta) {
        case ExternalTriggerActivation::RAISING_EDGE: {
            gx_trigger_activation = GX_TRIGGER_ACTIVATION_RISINGEDGE;
            break;
        }
        case ExternalTriggerActivation::FALLING_EDGE: {
            gx_trigger_activation = GX_TRIGGER_ACTIVATION_FALLINGEDGE;
            break;
        }
        case ExternalTriggerActivation::ANY_EDGE: {
            gx_trigger_activation = GX_TRIGGER_ACTIVATION_ANYEDGE;
            break;
        }
        case ExternalTriggerActivation::LEVEL_HIGH: {
            gx_trigger_activation = GX_TRIGGER_ACTIVATION_LEVELHIGH;
            break;
        }
        case ExternalTriggerActivation::LEVEL_LOW: {
            gx_trigger_activation = GX_TRIGGER_ACTIVATION_LEVELLOW;
            break;
        }
    }

    status = GXSetEnum(device_handle_, GX_ENUM_LINE_SELECTOR, gx_line_selector);
    GX_VERIFY(status)

    // 设置触发模式为 ON
    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    GX_VERIFY(status)

    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_ACTIVATION, gx_trigger_activation);
    GX_VERIFY(status)
    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_SOURCE, gx_trigger_source_line);
    GX_VERIFY(status)
    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START);
    GX_VERIFY(status)
    // 注册图像处理回调函数
    status = GXRegisterCaptureCallback(device_handle_, &capture_callback_user_param_, OnFrameCallbackFun);
    GX_VERIFY(status)
    // 发送开采命令
    status = GXSendCommand(device_handle_, GX_COMMAND_ACQUISITION_START);
    GX_VERIFY(status)
    GXSetInt(device_handle_, GX_INT_LINE_FILTER_RAISING_EDGE, 1000);

    return GX_STATUS_SUCCESS;
}

int GxCamera::Run(FrameData& frame_data, std::chrono::milliseconds trigger_time_ms) {
    run_mode_ = RunMode::TIME_SOFTWARE_TRIGGER;
    run_server_var_.emplace<TSTRunServer>();
    std::get<TSTRunServer>(run_server_var_).trigger_wait_ms = trigger_time_ms;

    capture_callback_user_param_.img->create(static_cast<int>(roi_.height), static_cast<int>(roi_.width), CV_8UC3);


    frame_data.cv = capture_callback_user_param_.cv;
    frame_data.img = capture_callback_user_param_.img;
    frame_data.m = capture_callback_user_param_.m;
    frame_data.timestamp = capture_callback_user_param_.timestamp;
    frame_data.img_msg = capture_callback_user_param_.img_msg;
    capture_callback_user_param_.img_msg->encoding = "rgb8";
    capture_callback_user_param_.img_msg->width = static_cast<int>(roi_.width);
    capture_callback_user_param_.img_msg->height = static_cast<int>(roi_.height);
    capture_callback_user_param_.img_msg->is_bigendian = false;
    capture_callback_user_param_.img_msg->step = static_cast<int>(roi_.width) * 1 * 3;
    capture_callback_user_param_.img_msg->data.resize(static_cast<int>(roi_.width) * static_cast<int>(roi_.height) *
                                                      sizeof(uint8_t));

    GX_STATUS status;
    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    GX_VERIFY(status)
    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
    GX_VERIFY(status)
    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START);
    GX_VERIFY(status)

    status = GXRegisterCaptureCallback(device_handle_, &capture_callback_user_param_, OnFrameCallbackFun);
    GX_VERIFY(status)

    status = GXSendCommand(device_handle_, GX_COMMAND_ACQUISITION_START);
    GX_VERIFY(status)

    std::thread t1([this]() {
        GX_STATUS status;
        auto& thread_server = std::get<TSTRunServer>(run_server_var_);

        while (true) {
            std::unique_lock run_lk(thread_server.m);
            thread_server.cv.wait_for(run_lk, thread_server.trigger_wait_ms);
            if (thread_server.need_exit) {
                break;
            } else {
                status = GXSendCommand(device_handle_, GX_COMMAND_TRIGGER_SOFTWARE);
                if (status != GX_STATUS_SUCCESS) {
                    GetErrorString(status);
                    std::cout << "TST thread: software trigger failed" << std::endl;
                    break;
                } else {
                    // std::cout << "TST thread: software trigger success" << std::endl;
                }
            }
        }
        std::cout << "TST thread: software trigger thread normal exit" << std::endl;
        {
            std::unique_lock exit_lk(thread_server.m);
            thread_server.successful_exit = true;
        }
        thread_server.cv.notify_all();
    });

    t1.detach();

    return GX_STATUS_SUCCESS;
}

int GxCamera::Run(FrameData& frame_data) {
    run_mode_ = RunMode::BOOL_SOFTWARE_TRIGGER;
    run_server_var_.emplace<ASTRunServer>();

    capture_callback_user_param_.img->create(static_cast<int>(roi_.height), static_cast<int>(roi_.width), CV_8UC3);
    frame_data.cv = capture_callback_user_param_.cv;
    frame_data.img = capture_callback_user_param_.img;
    frame_data.m = capture_callback_user_param_.m;
    frame_data.timestamp = capture_callback_user_param_.timestamp;
    frame_data.img_msg = capture_callback_user_param_.img_msg;
    capture_callback_user_param_.img_msg->encoding = "rgb8";
    capture_callback_user_param_.img_msg->width = static_cast<int>(roi_.width);
    capture_callback_user_param_.img_msg->height = static_cast<int>(roi_.height);
    capture_callback_user_param_.img_msg->is_bigendian = false;
    capture_callback_user_param_.img_msg->step = static_cast<int>(roi_.width) * 1 * 3;
    capture_callback_user_param_.img_msg->data.resize(static_cast<int>(roi_.width) * static_cast<int>(roi_.height) *
                                                      sizeof(uint8_t));

    [[maybe_unused]] GX_STATUS status;
    // 设置触发模式为 ON
    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    GX_VERIFY(status)
    // 设置触发激活方式为上升沿

    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
    GX_VERIFY(status)
    status = GXSetEnum(device_handle_, GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START);
    GX_VERIFY(status)
    status = GXRegisterCaptureCallback(device_handle_, &capture_callback_user_param_, OnFrameCallbackFun);
    GX_VERIFY(status)
    status = GXSendCommand(device_handle_, GX_COMMAND_ACQUISITION_START);
    GX_VERIFY(status)

    GXSetInt(device_handle_, GX_INT_LINE_FILTER_RAISING_EDGE, 1000);

    return GX_STATUS_SUCCESS;
}

void GxCamera::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame) {

    auto img = ((CaptureCallbackUserParam*)pFrame->pUserParam)->img;
    auto timestamp = ((CaptureCallbackUserParam*)pFrame->pUserParam)->timestamp;
    auto m = ((CaptureCallbackUserParam*)pFrame->pUserParam)->m;
    auto cv = ((CaptureCallbackUserParam*)pFrame->pUserParam)->cv;
    auto img_msg = ((CaptureCallbackUserParam*)pFrame->pUserParam)->img_msg;


    //*timestamp = pFrame->nTimestamp;
    *timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count() - 8000000;

    if (pFrame->status != GX_FRAME_STATUS_SUCCESS) {
        std::cout << "not success" << std::endl;
        cv->notify_all();
        return;
    }
    memcpy(img_msg->data.data(), (void*)pFrame->pImgBuf, 2048 * 1536);
    
    cv->notify_all();


}

int GxCamera::ActivateTrigger() {
    if (run_mode_ != RunMode::BOOL_SOFTWARE_TRIGGER)
        std::cout << "camera is not work on software trigger" << std::endl;
    assert(run_mode_ == RunMode::BOOL_SOFTWARE_TRIGGER);

    GX_STATUS status;
    status = GXSendCommand(device_handle_, GX_COMMAND_TRIGGER_SOFTWARE);
    if (status != GX_STATUS_SUCCESS) {
        GetErrorString(status);
        //        std::cout << "software trigger thread exit by error" << std::endl;
    } else {
        //        std::cout << "software success trigger" << std::endl;
    }
    return GX_STATUS_SUCCESS;
}

int GxCamera::Stop() {
    GX_STATUS status;

    rclcpp::shutdown();
    switch (run_mode_) {
        case RunMode::EXTERNAL_TRIGGER: {
            run_server_var_.emplace<std::monostate>();

            break;
        }
        case RunMode::TIME_SOFTWARE_TRIGGER: {
            auto& thread_server = std::get<TSTRunServer>(run_server_var_);
            {
                std::unique_lock run_lk(thread_server.m);
                thread_server.need_exit = true;
            }
            thread_server.cv.notify_all();

            std::unique_lock exit_lk(thread_server.m);
            thread_server.cv.wait(exit_lk, [&thread_server] { return thread_server.successful_exit; });

            std::cout << "server thread successful exit" << std::endl;
            run_server_var_.emplace<std::monostate>();
            break;
        }

        case RunMode::BOOL_SOFTWARE_TRIGGER: {
            run_server_var_.emplace<std::monostate>();
            break;
        }
        case RunMode::STOP: {
            std::cout << "error , no camera has been run" << std::endl;
            break;
        }
    }

    status = GXSendCommand(device_handle_, GX_COMMAND_ACQUISITION_STOP);
    GX_VERIFY(status)
    status = GXUnregisterCaptureCallback(device_handle_);
    GX_VERIFY(status)
    status = GXCloseDevice(device_handle_);
    GX_VERIFY(status)

    run_mode_ = RunMode::STOP;
    return GX_STATUS_SUCCESS;
}

int GxCamera::UnInit() {
    GX_STATUS status;
    // Release library
    status = GXCloseLib();
    GX_VERIFY(status)
    rclcpp::shutdown();
    return GX_STATUS_SUCCESS;
}

//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code

\return void
*/
//----------------------------------------------------------------------------------
void GxCamera::GetErrorString(GX_STATUS emErrorStatus) {
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, nullptr, &size);
    if (emStatus != GX_STATUS_SUCCESS) {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }

    auto error_info = (char*)std::malloc(size);
    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS) {
        printf("<Error when calling GXGetLastError>\n");
    } else {
        printf("%s\n", error_info);
    }
    delete[] error_info;
}

int GxCamera::WaitForFrame(cv::Mat& dst_mat,
                           GxCamera::FrameData& input_frame_data,
                           std::chrono::milliseconds time_out_ms) {
    {
        std::unique_lock lk(*input_frame_data.m);
        auto status = input_frame_data.cv->wait_for(lk, time_out_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (status == std::cv_status::no_timeout) {
            input_frame_data.img->copyTo(dst_mat);
            return 0;
        } else {
            return -1;
        }
    }
}

int GxCamera::WaitForImgmsg(sensor_msgs::msg::Image& img_msg,
                            sensor_msgs::msg::CameraInfo& camera_info,
                            GxCamera::FrameData& input_frame_data,
                            std::chrono::milliseconds time_out_ms) {
    {
        std::unique_lock lk(*input_frame_data.m);
        auto status = input_frame_data.cv->wait_for(lk, time_out_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (status == std::cv_status::no_timeout) {
            //std:: cout << "trigger_time_in_wait" << *input_frame_data.timestamp << std::endl;
            img_msg = *input_frame_data.img_msg;
            //std::cout<< "wait for/ time:" << *input_frame_data.timestamp<<std::endl;
            img_msg.header.stamp.nanosec = *input_frame_data.timestamp % 1000000000;
            img_msg.header.stamp.sec = *input_frame_data.timestamp / 1000000000;

            //std::cout << "dafsasfdsafadsfsda"  << *input_frame_data.timestamp << std::endl;

            camera_info.header.stamp.nanosec = *input_frame_data.timestamp % 1000000000;
            camera_info.header.stamp.sec = *input_frame_data.timestamp / 1000000000;
            return 0;
        } else {
            return -1;
        }
    }
}
