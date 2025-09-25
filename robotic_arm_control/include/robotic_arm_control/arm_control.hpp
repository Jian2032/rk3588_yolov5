#ifndef ARM_CONTROL_HPP_
#define ARM_CONTROL_HPP_

// ROS2 C++ 节点开发基础头文件
#include "rclcpp/rclcpp.hpp"

// 常用标准库
#include <vector>   // 使用 std::vector 容器
#include <cmath>    // 数学函数
#include <signal.h> // 信号处理（如 Ctrl+C）
#include <memory>   // 智能指针
#include <iostream> // 输入输出流
#include <unistd.h> // UNIX 系统调用（如 sleep）
#include <chrono>   // 时间相关（定时器等）
#include <iomanip>  // 控制输出格式

// 机器人相关接口与消息头文件
#include "cpp_interface/nrc_interface.h"     // 机械臂驱动接口
#include "cpp_interface/nrc_queue_operate.h" // 机械臂任务队列操作接口
#include "cpp_interface/nrc_job_operate.h"   // 机械臂任务操作接口
#include "robot_msgs/msg/arm_control.hpp"    // 机械臂控制指令消息
#include "robot_msgs/msg/arm_info.hpp"       // 机械臂状态信息消息
#include "keyboard_reader.hpp"               // 键盘输入读取工具类

// 机械臂控制数据结构体
typedef struct _ArmControlData
{
    int8_t mode;       // 控制模式（例如：0-停止，1-点动，2-自动运行）
    int8_t work_phase; // 工作阶段/状态标记（例如：初始化、执行、完成）
    double position_x; // 末端位姿 X 坐标（单位：mm 或 m，取决于驱动）
    double position_y; // 末端位姿 Y 坐标
    double position_z; // 末端位姿 Z 坐标
    double position_u; // 姿态角度 U（一般指绕某个轴的旋转角度）

    int direction_x;    // 移动方向 X（1 正向 / -1 反向 / 0 不动）
    int direction_y;    // 移动方向 Y
    int direction_z;    // 移动方向 Z
    double direction_u; // 姿态角速度（旋转方向 + 角度）
} ArmControlData;

// 机械臂控制类 ArmControl
// 继承自 ROS2 节点 和 键盘输入类
class ArmControl : public rclcpp::Node, public KeyboardReader
{
public:
    ArmControl();  // 构造函数，初始化 ROS2 节点和相关资源
    ~ArmControl(); // 析构函数，释放资源

    // 机械臂主循环，定时执行控制逻辑
    void arm_control_loop();

private:
    // 上电函数：向机械臂控制器发送上电指令
    void power_on(int fd);

    // 等待机械臂运行结束（阻塞等待，带超时时间）
    void wait_for_running_over(int fd, int timeout_ms);

    // 点动控制：通过给定方向和速度对机械臂进行手动微调
    void arm_control_jog(ArmControlData data);

    // 装载位姿（目标点位、坐标系、速度、加速度、减速度）
    void loadpos(MoveCmd &m, std::vector<double> pos, int coord,
                 double velocity, double acc, double dcc);

    // ROS2 订阅回调函数：接收外部发送的 ArmControl 消息并更新控制数据
    void ArmControlCallback(const robot_msgs::msg::ArmControl::SharedPtr msg);

    // ROS2 通信
    rclcpp::Publisher<robot_msgs::msg::ArmInfo>::SharedPtr pub_arm_info_;      // 发布机械臂状态信息
    rclcpp::Subscription<robot_msgs::msg::ArmControl>::SharedPtr sub_control_; // 订阅机械臂控制指令

    // 机械臂底层接口与运行状态
    SOCKETFD fd;             // 与机械臂控制器通信的套接字文件描述符
    Result res;              // 机械臂操作返回结果（成功/失败等）
    ArmControlData arm_data; // 当前控制数据缓存（位姿、模式、方向等）

    // 运行辅助变量
    int cnt = 0;                            // 计数器，用于判断控制方向运动
    int key;                                // 键盘输入值
    std::vector<double> pos_tcp, pos_joint; // TCP 坐标（末端工具坐标）与关节坐标
    MoveCmd move_cmd;                       // 机械臂运动命令结构体
    rclcpp::TimerBase::SharedPtr timer_;    // ROS2 定时器（驱动循环调用）
};

#endif
