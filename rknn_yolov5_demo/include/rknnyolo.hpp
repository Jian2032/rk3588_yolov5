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

constexpr int MAX_RESULTS = 32;

typedef struct _TargetInfo
{
    // 总目标数
    int total_count;   
    // 当前是第几个目标
    int index;         
    // 中心点坐标
    int center_x;
    int center_y;
    // 距离
    int distance;
}TargetInfo;

typedef struct _TargetGroup
{
    int total_count;            
    TargetInfo targets[10];    // 固定上限
} TargetGroup;

class RknnYoloNode : public rclcpp::Node
{
public:
    explicit RknnYoloNode(const std::string &model_path);

    static RknnYoloNode* getInstance(){
        return instance_;
    }
    int getDistance(int u, int v);
    void addTarget(detect_result_group_t* group);

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    static RknnYoloNode *instance_;
    
    std::shared_ptr<rknnPool<rkYolov5s, cv::Mat, cv::Mat>> pool_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_raw_,sub_depth_ ;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Publisher<robot_msgs::msg::TargetArray>::SharedPtr pub_targets_;
    int frames_;
    double last_time_;

    TargetGroup target_;

    cv::Mat depth_image;
};

#endif