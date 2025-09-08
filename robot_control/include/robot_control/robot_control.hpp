#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/target_array.hpp"
#include "robot_msgs/msg/arm_control.hpp"
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
}TargetInfo;

typedef struct _TargetGroup
{
    int total_count;            
    std::array<TargetInfo, 4> targets;    // 固定上限
} TargetGroup;

typedef struct _PointInfo {
    int index;
    int u;
    int v;
    int distance;
}PointInfo;

class RobotControl : public rclcpp::Node
{
public:
    RobotControl();

    void controlLoop();
    bool calculateMovement(const PointInfo &point, const TargetInfo &target);

private:
    void targetCallback(const robot_msgs::msg::TargetArray::SharedPtr msg);
    rclcpp::Subscription<robot_msgs::msg::TargetArray>::SharedPtr sub_target_;
    rclcpp::Publisher<robot_msgs::msg::ArmControl>::SharedPtr pub_arm_;

    TargetGroup target_;
    std::array<PointInfo, 4> points_;
    rclcpp::TimerBase::SharedPtr timer_;

    int index = 0;
};
#endif