#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <pthread.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>

using namespace std;

// 共享内存时间戳结构
struct TimeStamp {
    int64_t high;
    int64_t low;
};

static volatile bool exit_flag = false;
static TimeStamp* shared_timestamp_ptr = nullptr;
static image_transport::Publisher pub;
static float image_scale = 1.0;
static int trigger_enable = 1;

enum PixelFormat : unsigned int {
    RGB8 = 0x02180014,
    BayerRG8 = 0x01080009,
    BayerRG12Packed = 0x010C002B,
    BayerGB12Packed = 0x010C002C,
    BayerGB8 = 0x0108000A
};
static std::vector<PixelFormat> PIXEL_FORMAT = { RGB8, BayerRG8, BayerRG12Packed, BayerGB12Packed, BayerGB8 };

// 信号处理：捕捉 Ctrl+C
void SignalHandler(int signal) {
    if (signal == SIGINT) {
        std::cerr << "\nReceived Ctrl+C, exiting..." << std::endl;
        exit_flag = true;
    }
}

void SetupSignalHandler() {
    struct sigaction sa{};
    sa.sa_handler = SignalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
}

// 打印设备信息
bool PrintDeviceInfo(MV_CC_DEVICE_INFO* devInfo) {
    if (!devInfo) {
        cerr << "Null device info pointer!" << endl;
        return false;
    }
    if (devInfo->nTLayerType == MV_GIGE_DEVICE) {
        int ip1 = (devInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24;
        int ip2 = (devInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16;
        int ip3 = (devInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8;
        int ip4 = (devInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        cout << "Device Model Name: " << devInfo->SpecialInfo.stGigEInfo.chModelName << "\n";
        cout << "Current IP: " << ip1 << "." << ip2 << "." << ip3 << "." << ip4 << "\n";
        cout << "Serial Number: " << devInfo->SpecialInfo.stGigEInfo.chSerialNumber << "\n";
    } else if (devInfo->nTLayerType == MV_USB_DEVICE) {
        cout << "Device Model Name: " << devInfo->SpecialInfo.stUsb3VInfo.chModelName << "\n";
        cout << "Serial Number: " << devInfo->SpecialInfo.stUsb3VInfo.chSerialNumber << "\n";
    } else {
        cout << "Unsupported device type.\n";
        return false;
    }
    return true;
}

// 设置相机参数
bool SetCameraParams(void* handle, const cv::FileStorage& params) {
    int ret;
    int exposureAutoMode = (int)params["ExposureAutoMode"];
    int exposureTimeLower = (int)params["AutoExposureTimeLower"];
    int exposureTimeUpper = (int)params["AutoExposureTimeUpper"];
    int exposureTime = (int)params["ExposureTime"];
    int gainAuto = (int)params["GainAuto"];
    float gain = (float)params["Gain"];
    int gammaSelector = (int)params["GammaSelector"];
    float gamma = (float)params["Gamma"];

    static const std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
    static const std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
    static const std::string GammaSelectorStr[3] = {"User", "sRGB", "Off"};

    // 曝光模式
    ret = MV_CC_SetExposureAutoMode(handle, exposureAutoMode);
    if (ret == MV_OK) {
        ROS_INFO_STREAM("ExposureAutoMode set to " << ExposureAutoStr[exposureAutoMode]);
    } else {
        ROS_WARN_STREAM("Failed to set ExposureAutoMode");
    }

    // 自动曝光上下限
    if (exposureAutoMode == 2) {
        ret = MV_CC_SetAutoExposureTimeLower(handle, exposureTimeLower);
        if (ret != MV_OK) ROS_ERROR("Failed to set AutoExposureTimeLower");
        ret = MV_CC_SetAutoExposureTimeUpper(handle, exposureTimeUpper);
        if (ret != MV_OK) ROS_ERROR("Failed to set AutoExposureTimeUpper");
    }

    // 固定曝光时间
    if (exposureAutoMode == 0) {
        ret = MV_CC_SetExposureTime(handle, exposureTime);
        if (ret != MV_OK) ROS_ERROR("Failed to set ExposureTime");
    }

    // 增益自动模式
    ret = MV_CC_SetEnumValue(handle, "GainAuto", gainAuto);
    if (ret == MV_OK) {
        ROS_INFO_STREAM("GainAuto set to " << GainAutoStr[gainAuto]);
    } else {
        ROS_ERROR("Failed to set GainAuto");
    }

    // 固定增益值
    if (gainAuto == 0) {
        ret = MV_CC_SetGain(handle, gain);
        if (ret != MV_OK) ROS_ERROR("Failed to set Gain");
    }

    // Gamma设置
    ret = MV_CC_SetGammaSelector(handle, gammaSelector);
    if (ret == MV_OK) {
        ROS_INFO_STREAM("GammaSelector set to " << GammaSelectorStr[gammaSelector]);
    } else {
        ROS_ERROR("Failed to set GammaSelector");
    }
    ret = MV_CC_SetGamma(handle, gamma);
    if (ret != MV_OK) ROS_ERROR("Failed to set Gamma");

    return true;
}

// 初始化相机：打开设备、设置参数等
bool InitCamera(void*& handle, const std::string& serialNumber, int pixelFormatIdx, const cv::FileStorage& params) {
    int ret;
    MV_CC_DEVICE_INFO_LIST deviceList;
    memset(&deviceList, 0, sizeof(deviceList));
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (ret != MV_OK) {
        ROS_ERROR("Enum devices failed");
        return false;
    }
    if (deviceList.nDeviceNum == 0) {
        ROS_ERROR("No camera devices found");
        return false;
    }

    // 查找序列号匹配设备索引
    int selectedIdx = -1;
    for (unsigned int i = 0; i < deviceList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* dev = deviceList.pDeviceInfo[i];
        string sn;
        if (dev->nTLayerType == MV_USB_DEVICE) {
            sn = string((char*)dev->SpecialInfo.stUsb3VInfo.chSerialNumber);
        } else if (dev->nTLayerType == MV_GIGE_DEVICE) {
            sn = string((char*)dev->SpecialInfo.stGigEInfo.chSerialNumber);
        }
        if (sn == serialNumber) {
            selectedIdx = i;
            break;
        }
    }
    if (selectedIdx == -1) {
        ROS_ERROR_STREAM("Can't find camera with serial number: " << serialNumber);
        return false;
    }

    // 创建句柄
    ret = MV_CC_CreateHandle(&handle, deviceList.pDeviceInfo[selectedIdx]);
    if (ret != MV_OK) {
        ROS_ERROR("Create handle failed");
        return false;
    }

    // 打开设备
    ret = MV_CC_OpenDevice(handle);
    if (ret != MV_OK) {
        ROS_ERROR("Open device failed");
        MV_CC_DestroyHandle(handle);
        return false;
    }

    // 设置像素格式
    ret = MV_CC_SetEnumValue(handle, "PixelFormat", PIXEL_FORMAT[pixelFormatIdx]);
    if (ret != MV_OK) {
        ROS_ERROR("Set PixelFormat failed");
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    // 关闭帧率限制
    ret = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
    if (ret != MV_OK) {
        ROS_WARN("Failed to disable frame rate limit");
    }

    // 设置触发模式
    ret = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
    if (ret != MV_OK) {
        ROS_ERROR("Set TriggerMode failed");
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    ret = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
    if (ret != MV_OK) {
        ROS_ERROR("Set TriggerSource failed");
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    // 设置相机参数
    if (!SetCameraParams(handle, params)) {
        ROS_WARN("Set camera parameters failed");
    }

    // 启动抓图
    ret = MV_CC_StartGrabbing(handle);
    if (ret != MV_OK) {
        ROS_ERROR("Start grabbing failed");
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    return true;
}

// 图像采集线程
void* CaptureThread(void* pUser) {
    void* handle = pUser;
    int ret;

    MVCC_INTVALUE payloadSizeParam;
    ret = MV_CC_GetIntValue(handle, "PayloadSize", &payloadSizeParam);
    if (ret != MV_OK) {
        cerr << "Get PayloadSize failed: " << hex << ret << endl;
        return nullptr;
    }

    unsigned int payloadSize = payloadSizeParam.nCurValue;

    unsigned char* buffer = new unsigned char[payloadSize];
    unsigned char* buffer_converted = new unsigned char[payloadSize * 3]; // 3倍大小以防

    while (!exit_flag && ros::ok()) {
        MV_FRAME_OUT_INFO_EX frameInfo;
        memset(&frameInfo, 0, sizeof(frameInfo));

        ret = MV_CC_GetOneFrameTimeout(handle, buffer, payloadSize, &frameInfo, 1000);
        if (ret == MV_OK) {
            ros::Time timestamp;
            if (trigger_enable && shared_timestamp_ptr != MAP_FAILED && shared_timestamp_ptr->low != 0) {
                double ts_sec = shared_timestamp_ptr->low / 1e9;
                timestamp = ros::Time(ts_sec);
            } else {
                timestamp = ros::Time::now();
            }

            MV_CC_PIXEL_CONVERT_PARAM convertParam{};
            convertParam.nWidth = frameInfo.nWidth;
            convertParam.nHeight = frameInfo.nHeight;
            convertParam.pSrcData = buffer;
            convertParam.nSrcDataLen = payloadSize;
            convertParam.enSrcPixelType = frameInfo.enPixelType;
            convertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            convertParam.pDstBuffer = buffer_converted;
            convertParam.nDstBufferSize = payloadSize * 3;

            ret = MV_CC_ConvertPixelType(handle, &convertParam);
            if (ret != MV_OK) {
                cerr << "ConvertPixelType failed: " << hex << ret << endl;
                continue;
            }

            cv::Mat img(frameInfo.nHeight, frameInfo.nWidth, CV_8UC3, buffer_converted);

            if (image_scale > 0.1 && image_scale != 1.0) {
                cv::resize(img, img, cv::Size(img.cols * image_scale, img.rows * image_scale));
            }

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img).toImageMsg();
            msg->header.stamp = timestamp;
            pub.publish(msg);

            // 调试窗口，可选打开
            // cv::imshow("Camera", img);
            // cv::waitKey(1);
        }
    }

    delete[] buffer;
    delete[] buffer_converted;
    return nullptr;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mvs_trigger");
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <params.yaml>" << endl;
        return -1;
    }
    std::string params_file = argv[1];

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    SetupSignalHandler();

    cv::FileStorage params(params_file, cv::FileStorage::READ);
    if (!params.isOpened()) {
        ROS_ERROR_STREAM("Failed to open params file: " << params_file);
        return -1;
    }

    trigger_enable = (int)params["TriggerEnable"];
    std::string serial_number = (std::string)params["SerialNumber"];
    std::string topic_name = (std::string)params["TopicName"];
    int pixelFormatIdx = (int)params["PixelFormat"];
    image_scale = (float)params["image_scale"];
    if (image_scale < 0.1) image_scale = 1.0f;

    pub = it.advertise(topic_name, 1);

    // 共享内存映射
    const char* home_path = getenv("HOME");
    if (!home_path) {
        ROS_ERROR("HOME environment variable not set");
        return -1;
    }
    std::string shm_path = std::string(home_path) + "/timeshare";

    int shm_fd = open(shm_path.c_str(), O_RDWR);
    if (shm_fd < 0) {
        ROS_ERROR_STREAM("Failed to open shared memory: " << shm_path);
        return -1;
    }

    shared_timestamp_ptr = (TimeStamp*)mmap(NULL, sizeof(TimeStamp),
                                            PROT_READ | PROT_WRITE,
                                            MAP_SHARED, shm_fd, 0);
    if (shared_timestamp_ptr == MAP_FAILED) {
        ROS_ERROR("mmap failed for shared memory");
        close(shm_fd);
        return -1;
    }

    void* handle = nullptr;
    if (!InitCamera(handle, serial_number, pixelFormatIdx, params)) {
        munmap(shared_timestamp_ptr, sizeof(TimeStamp));
        close(shm_fd);
        return -1;
    }

    // 启动采集线程
    pthread_t capture_thread;
    int ret = pthread_create(&capture_thread, nullptr, CaptureThread, handle);
    if (ret != 0) {
        ROS_ERROR("Failed to create capture thread");
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        munmap(shared_timestamp_ptr, sizeof(TimeStamp));
        close(shm_fd);
        return -1;
    }

    ros::Rate rate(10);
    while (ros::ok() && !exit_flag) {
        ros::spinOnce();
        rate.sleep();
    }

    pthread_join(capture_thread, nullptr);
    ROS_INFO("Capture thread joined");

    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);

    munmap(shared_timestamp_ptr, sizeof(TimeStamp));
    close(shm_fd);

    return 0;
}
