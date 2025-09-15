#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/target_array.hpp"
#include "robot_msgs/msg/arm_control.hpp"
#include "robot_msgs/msg/arm_info.hpp"
#include <array>
#include <vector>
#include <chrono>

constexpr int MAX_RESULTS = 32;

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

typedef struct _TargetGroup
{
    int total_count;
    std::array<TargetInfo, 4> targets; // 固定上限
} TargetGroup;

typedef struct _PointInfo
{
    int index;
    int u;
    int v;
    int distance;
} PointInfo;

class RobotControl : public rclcpp::Node
{
public:
    RobotControl();
    ~RobotControl();

    void controlLoop();
    bool calculateMovement(const PointInfo &point, const TargetInfo &target);
    bool calculatePointMovement(const std::array<double, 4> current_tcp_, const std::array<double, 4> target_tcp_);

private:
    void targetCallback(const robot_msgs::msg::TargetArray::SharedPtr msg);
    void arminfoCallback(const robot_msgs::msg::ArmInfo::SharedPtr msg);
    rclcpp::Subscription<robot_msgs::msg::TargetArray>::SharedPtr sub_target_;
    rclcpp::Subscription<robot_msgs::msg::ArmInfo>::SharedPtr sub_arm_info_;
    rclcpp::Publisher<robot_msgs::msg::ArmControl>::SharedPtr pub_arm_ctl_;

    TargetGroup target_;
    // 目标像素点信息
    std::array<PointInfo, 4> points_;
    // 当前机械臂信息
    std::array<double, 4> current_tcp_;
    std::array<double, 4> current_joints_;
    std::array<std::array<double, 4>, 4> target_tcp_list_;

    rclcpp::TimerBase::SharedPtr timer_;

    // 工作阶段
    int work_phase = 0;
    // 调用目标信息点
    int index = 0;
};
#endif