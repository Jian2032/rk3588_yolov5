#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "rclcpp/rclcpp.hpp"                   // ROS2 节点与基础功能
#include "robot_msgs/msg/target_array.hpp"     // 自定义消息：识别到的乳头目标数组
#include "robot_msgs/msg/arm_control.hpp"      // 自定义消息：机械臂控制
#include "robot_msgs/msg/arm_info.hpp"         // 自定义消息：机械臂信息（位置、姿态等）
#include "std_msgs/msg/u_int8_multi_array.hpp" // ROS2 标准消息：继电器控制数组
#include <array>
#include <vector>
#include <chrono>

// 单个目标的信息（通常对应一个乳头）
typedef struct _TargetInfo
{
    int index;    // 当前目标序号（第几个目标）
    int center_u; // 目标在图像中的 u 坐标（横向像素）
    int center_v; // 目标在图像中的 v 坐标（纵向像素）
    int distance; // 与目标的距离（通常是相机/视觉估计得到的物理距离）
} TargetInfo;

// 一组目标信息（例如一次最多识别到 4 个乳头）
typedef struct _TargetGroup
{
    int total_count;                   // 实际检测到的目标数量
    std::array<TargetInfo, 4> targets; // 存放最多 4 个目标的信息（固定上限）
} TargetGroup;

// 单个像素点的目标信息（如用于运动控制的关键点）
typedef struct _PointInfo
{
    int index;    // 点的序号
    int u;        // 像素横坐标
    int v;        // 像素纵坐标
    int distance; // 与点的距离（毫米或厘米）
} PointInfo;

// ------------------ 机器人控制类 ------------------
class RobotControl : public rclcpp::Node
{
public:
    RobotControl();  // 构造函数：初始化节点
    ~RobotControl(); // 析构函数

private:
    // ------------------ 内部功能函数 ------------------
    void controlLoop(); // 主控制循环（由定时器周期调用）

    // 机械臂向量运动控制，根据当前点和目标点，计算并执行运动
    bool calculateMovement(const PointInfo &point, const TargetInfo &target);

    // 机械臂点位运动控制，根据当前位置和目标位置的 TCP 点进行运动
    bool calculatePointMovement(const std::array<double, 4> current_tcp_,
                                const std::array<double, 4> target_tcp_);

    // 控制继电器（生成继电器控制消息）
    // input：控制输入数组（14 个通道）
    // rc_ctl：输出的 ROS2 消息（UInt8MultiArray）
    void setRcCtl(const std::array<uint8_t, 14> &input,
                  std_msgs::msg::UInt8MultiArray &rc_ctl);

    // 视觉识别乳头位置回调（订阅 topic：TargetArray）
    void targetCallback(const robot_msgs::msg::TargetArray::SharedPtr msg);

    // 机械臂笛卡尔、关节坐标信息回调（订阅 topic：ArmInfo）
    void arminfoCallback(const robot_msgs::msg::ArmInfo::SharedPtr msg);

    // ------------------ ROS2 通信 ------------------
    rclcpp::Subscription<robot_msgs::msg::TargetArray>::SharedPtr sub_target_; // 订阅乳头目标信息
    rclcpp::Subscription<robot_msgs::msg::ArmInfo>::SharedPtr sub_arm_info_;   // 订阅机械臂位姿信息

    rclcpp::Publisher<robot_msgs::msg::ArmControl>::SharedPtr pub_arm_ctl_;   // 发布机械臂控制指令
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_rc_ctl_; // 发布继电器控制指令

    // ------------------ 内部数据缓存 ------------------
    TargetGroup target_;                                   // 当前识别到的乳头目标信息
    std::array<PointInfo, 6> points_;                      // 目标像素点信息（最多 6 个点）
    std::array<double, 4> current_tcp_;                    // 当前机械臂 TCP 位姿
    std::array<double, 4> current_joints_;                 // 当前机械臂关节角
    std::array<std::array<double, 4>, 4> target_tcp_list_; // 存放目标 TCP 列表

    std_msgs::msg::UInt8MultiArray rc_ctl_; // 控制下位机继电器的消息缓存

    rclcpp::TimerBase::SharedPtr timer_; // 定时器（周期调用 controlLoop）

    // 等待函数：检查是否需要等待指定时间（单位：秒）
    bool check_wait(int seconds);

    // ------------------ 状态变量 ------------------
    int work_phase;                                                    // 当前工作阶段（流程控制用，例如：定位、吸附、清洗等）
    int step;                                                         // 工作阶段中的步骤
    bool waiting;                                                      // 是否处于等待状态
    std::chrono::time_point<std::chrono::steady_clock> now, last_time; // 时间记录（用于等待控制）

    int index = 0; // 当前处理的目标点索引

    // 继电器控制输入定义（10 通道）
    // 0：奶杯号(1-4)
    // 1：拉绳(1=拉,0=松)
    // 2：奶托(1=斜,0=立)
    // 3：真空泵
    // 4：主奶管
    // 5：异奶管
    // 6：弃奶管
    // 7：乳刷
    // 8：支架
    // 9：药浴
    // 10~13:四个脉动器
    std::array<uint8_t, 14> rc_input;

    int cnt; // 循环计数器（用于继电器周期控制）
};

#endif
