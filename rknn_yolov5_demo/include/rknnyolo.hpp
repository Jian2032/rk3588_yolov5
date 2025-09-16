#ifndef RKNNYOLO_HPP
#define RKNNYOLO_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "rkYolov5s.hpp"
#include "rknnPool.hpp"
#include "postprocess.h"

#include "robot_msgs/msg/target_info.hpp"
#include "robot_msgs/msg/target_array.hpp"

#include <array>
#include <vector>

constexpr int MAX_RESULTS = 32;

// 识别目标信息
typedef struct _TargetInfo
{
    // 当前是第几个目标
    int index;
    // 中心点坐标
    int center_u;
    int center_v;
    // 距离
    int distance;
} TargetInfo;

// 多个目标信息
typedef struct _TargetGroup
{
    int total_count;
    std::array<TargetInfo, 10> targets; // 固定上限
} TargetGroup;

// RknnYoloNode类继承rclcpp::Node
class RknnYoloNode : public rclcpp::Node
{
public:
    // 单例模式，explicit防止参数构造函数被隐式调用
    explicit RknnYoloNode(const std::string &model_path);
    // 获取类的单例指针
    static RknnYoloNode *getInstance()
    {
        return instance_;
    }
    // 获取像素点距离
    int getDistance(int u, int v);
    // 目标点排序添加
    void addTarget(detect_result_group_t *group);

    ~RknnYoloNode();

private:
    // 相机图像回调
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // 深度图像回调
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    // 静态指针成员，属于类本身，所有对象共享一份
    static RknnYoloNode *instance_;
    // rknnPool对象
    std::shared_ptr<rknnPool<rkYolov5s, cv::Mat, cv::Mat>> pool_;
    // 消息订阅 彩色图像 深度图
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_raw_, sub_depth_;
    // 消息发布 图像信息 目标信息
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
    rclcpp::Publisher<robot_msgs::msg::TargetArray>::SharedPtr pub_targets_;
    // 统计帧数
    int frames_;
    // 记录上次处理时间
    double last_time_;
    // 储存检测结果
    TargetGroup target_;
    // 保存深度图像信息
    cv::Mat depth_image;

    std::thread worker_;
    std::atomic<bool> running_;

    void processThread();
};

#endif