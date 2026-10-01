#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp" // 用于参数回调
#include "MvCameraControl.h" // 海康 SDK 头文件

#include <thread>
#include <cstring>
#include <memory>
#include <functional>
#include <vector>

class HikrobotCameraNode : public rclcpp::Node
{
public:
    HikrobotCameraNode() : Node("hikrobot_camera_node")
    {
        // 1. 声明参数
        this->declare_parameter("exposure_time", 5000.0);
        this->declare_parameter("gain", 10.0);
        this->declare_parameter("frame_rate", 30.0);

        // 2. 注册参数回调，用于动态响应 ros2 param set 命令
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&HikrobotCameraNode::parametersCallback, this, std::placeholders::_1)
        );

        // 3. 创建发布者
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

        // 4. 初始化海康相机
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "相机初始化失败！请检查虚拟机USB直通或网络设置。");
        } else {
            RCLCPP_INFO(this->get_logger(), "相机初始化成功！开始取流...");
            grab_thread_ = std::thread(&HikrobotCameraNode::grabImageLoop, this);
        }
    }

    ~HikrobotCameraNode()
    {
        // 5. 退出时清理资源
        if (grab_thread_.joinable()) {
            grab_thread_.join();
        }
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
        }
        MV_CC_Finalize();
    }

private:
    // ================= 参数回调与设置逻辑 =================
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "参数设置成功";

        for (const auto &param : parameters) {
            if (param.get_name() == "exposure_time") {
                double exposure = param.as_double();
                if (exposure < 10.0 || exposure > 1000000.0) {
                    result.successful = false;
                    result.reason = "曝光时间超出范围 (10 - 1000000)";
                    return result;
                }
                if (!setCameraParam("ExposureTime", exposure)) {
                    result.successful = false;
                    result.reason = "SDK设置曝光失败，请检查相机连接状态";
                    return result;
                }
            }
            else if (param.get_name() == "gain") {
                double gain = param.as_double();
                if (gain < 0.0 || gain > 100.0) {
                    result.successful = false;
                    result.reason = "增益超出范围 (0 - 100)";
                    return result;
                }
                if (!setCameraParam("Gain", gain)) {
                    result.successful = false;
                    result.reason = "SDK设置增益失败，请检查相机连接状态";
                    return result;
                }
            }
            else if (param.get_name() == "frame_rate") {
                double rate = param.as_double();
                if (rate <= 0.0 || rate > 100.0) {
                    result.successful = false;
                    result.reason = "帧率超出范围 (1 - 100)";
                    return result;
                }
                if (!setCameraParam("AcquisitionFrameRate", rate)) {
                    result.successful = false;
                    result.reason = "SDK设置帧率失败，请检查相机连接状态";
                    return result;
                }
            }
        }
        return result;
    }

    // 封装设置参数的函数（无相机时安全返回失败，避免崩溃）
    bool setCameraParam(const std::string &param_name, double value) {
        if (handle_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "当前无相机连接，参数 [%s] 无法应用", param_name.c_str());
            return false; 
        }

        int nRet = MV_OK;
        if (param_name == "ExposureTime") {
            MV_CC_SetEnumValue(handle_, "ExposureAuto", 0); // 必须关闭自动曝光
            nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", value);
        } 
        else if (param_name == "Gain") {
            MV_CC_SetEnumValue(handle_, "GainAuto", 0); // 必须关闭自动增益
            nRet = MV_CC_SetFloatValue(handle_, "Gain", value);
        }
        else if (param_name == "AcquisitionFrameRate") {
            MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true); // 开启帧率控制
            nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", value);
        }

        if (nRet != MV_OK) {
            RCLCPP_ERROR(this->get_logger(), "设置参数 [%s] 失败，错误码 [0x%x]", param_name.c_str(), nRet);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "设置参数 [%s] 成功，值: %.2f", param_name.c_str(), value);
        return true;
    }

    // ================= 相机初始化与图像抓取逻辑 =================
    bool initCamera() {
        int nRet = MV_OK;

        nRet = MV_CC_Initialize();
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "初始化SDK失败! 错误码 [0x%x]", nRet);
            return false;
        }

        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE | MV_GENTL_XOC_DEVICE, &stDeviceList);
        if (MV_OK != nRet || stDeviceList.nDeviceNum == 0) {
            RCLCPP_ERROR(this->get_logger(), "未发现任何海康相机设备！");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "发现 %d 个设备，选择第一个连接", stDeviceList.nDeviceNum);
        unsigned int nIndex = 0;

        nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "创建句柄失败! 错误码 [0x%x]", nRet);
            return false;
        }

        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "打开设备失败! 错误码 [0x%x]", nRet);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return false;
        }

        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
            if (nPacketSize > 0) {
                MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", nPacketSize);
            }
        }

        nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
        if (MV_OK != nRet) {
            RCLCPP_WARN(this->get_logger(), "关闭触发模式失败，相机可能处于外部触发模式");
        }

        return true;
    }

    void grabImageLoop() {
        MV_FRAME_OUT stImageInfo;
        memset(&stImageInfo, 0, sizeof(stImageInfo));

        while (rclcpp::ok()) {
            int nRet = MV_CC_GetImageBuffer(handle_, &stImageInfo, 1000);

            if (nRet == MV_OK) {
                auto msg = std::make_unique<sensor_msgs::msg::Image>();
                msg->header.stamp = this->now();
                msg->header.frame_id = "camera_link";
                msg->height = stImageInfo.stFrameInfo.nHeight;
                msg->width = stImageInfo.stFrameInfo.nWidth;
                msg->is_bigendian = false;

                // 假设相机输出的是 BGR8 格式（彩色）。如果是黑白相机，改为 "mono8"，step = width
                msg->encoding = "bgr8"; 
                msg->step = stImageInfo.stFrameInfo.nWidth * 3; 
                msg->data.resize(msg->step * msg->height);

                memcpy(msg->data.data(), stImageInfo.pBufAddr, msg->data.size());
                image_pub_->publish(std::move(msg));
                MV_CC_FreeImageBuffer(handle_, &stImageInfo);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "获取图像失败或相机未连接: 0x%x", nRet);
            }
        }
    }

    // ================= 成员变量 =================
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::thread grab_thread_;
    void* handle_ = nullptr;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikrobotCameraNode>());
    rclcpp::shutdown();
    return 0;
}