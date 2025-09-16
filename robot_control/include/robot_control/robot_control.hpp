#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/target_array.hpp"
#include "robot_msgs/msg/arm_control.hpp"
#include "robot_msgs/msg/arm_info.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <array>
#include <vector>
#include <chrono>

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

// 目标位置像素点距离信息
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

private:
    void controlLoop();
    // 机械臂向量运动控制
    bool calculateMovement(const PointInfo &point, const TargetInfo &target);
    // 机械臂点位运动控制
    bool calculatePointMovement(const std::array<double, 4> current_tcp_, const std::array<double, 4> target_tcp_);
    // 控制继电器
    void setRcCtl(const std::array<uint8_t, 10>& input, std_msgs::msg::UInt8MultiArray &rc_ctl);
    // 视觉识别乳头位置回调
    void targetCallback(const robot_msgs::msg::TargetArray::SharedPtr msg);
    // 机械臂笛卡尔、关节坐标信息
    void arminfoCallback(const robot_msgs::msg::ArmInfo::SharedPtr msg);
    // 乳头目标信息、机械臂信息接收
    rclcpp::Subscription<robot_msgs::msg::TargetArray>::SharedPtr sub_target_;
    rclcpp::Subscription<robot_msgs::msg::ArmInfo>::SharedPtr sub_arm_info_;
    // 机械臂、继电器控制信息发布
    rclcpp::Publisher<robot_msgs::msg::ArmControl>::SharedPtr pub_arm_ctl_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_rc_ctl_;

    // 识别乳头信息
    TargetGroup target_;
    // 目标像素点信息
    std::array<PointInfo, 5> points_;
    // 当前机械臂信息
    std::array<double, 4> current_tcp_;
    std::array<double, 4> current_joints_;
    std::array<std::array<double, 4>, 4> target_tcp_list_;
    // 控制下位机继电器
    std_msgs::msg::UInt8MultiArray rc_ctl_;
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 工作阶段
    int work_phase = 0;
    // 调用目标信息点
    int index = 0;
    // 继电器控制消息
    std::array<uint8_t,10> rc_input;
};
#endif