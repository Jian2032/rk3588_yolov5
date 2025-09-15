#include "robot_control/robot_control.hpp"


RobotControl::RobotControl() : Node("robot_control_node")
{
    // 初始化四个乳头目标位置
    points_.at(0) = {1, 625, 375, 350};
    points_.at(1) = {2, 815, 385, 345};
    points_.at(2) = {3, 490, 335, 290};
    points_.at(3) = {4, 910, 290, 280};
    // 初始化点位控制目标点
    target_tcp_list_[0] = {450.0, -25.0, -280.0, 2.26};
    target_tcp_list_[1] = {300.0, 345.0, -280.0, 2.10};
    target_tcp_list_[2] = {300.0, 345.0, -280.0, 2.10};
    target_tcp_list_[3] = {300.0, 345.0, -280.0, 2.10};

    sub_target_ = this->create_subscription<robot_msgs::msg::TargetArray>(
        "/target_info", 10,
        std::bind(&RobotControl::targetCallback, this, std::placeholders::_1));

    sub_arm_info_ = this->create_subscription<robot_msgs::msg::ArmInfo>(
        "/arm_info", 10,
        std::bind(&RobotControl::arminfoCallback, this, std::placeholders::_1));
    
    pub_arm_ctl_ = this->create_publisher<robot_msgs::msg::ArmControl>("arm_control", 10);
    
    // 创建定时器 (10ms 周期)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotControl::controlLoop, this));
}

RobotControl::~RobotControl()
{
    // 发布清零消息
    robot_msgs::msg::ArmControl msg{};

    pub_arm_ctl_->publish(msg);

}

void RobotControl::targetCallback(const robot_msgs::msg::TargetArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received TargetArray, total_count=%d", msg->total_count);
    target_.total_count = msg->total_count;

    // 遍历 targets 数组
    for (size_t i = 0; i < msg->targets.size(); i++)
    {
        auto &t = msg->targets[i];

        target_.targets[i].index = t.index;
        target_.targets[i].center_u = t.center_u;
        target_.targets[i].center_v = t.center_v;
        target_.targets[i].distance = t.distance;
        // RCLCPP_INFO(this->get_logger(),
        //             "Target %d: center=(%d,%d), distance=%d",
        //             target_.targets[i].index, target_.targets[i].center_u, target_.targets[i].center_v, target_.targets[i].distance);
    }
}

void RobotControl::arminfoCallback(const robot_msgs::msg::ArmInfo::SharedPtr msg)
{
    // 打印 TCP 位姿
    // RCLCPP_INFO(this->get_logger(),
    //     "TCP Position -> x: %.3f, y: %.3f, z: %.3f, u: %.3f",
    //     msg->tcp_position_x,
    //     msg->tcp_position_y,
    //     msg->tcp_position_z,
    //     msg->tcp_position_u);

    // 打印关节位置
    // RCLCPP_INFO(this->get_logger(),
    //     "Joint Position -> j1: %.3f, j2: %.3f, j3: %.3f, j4: %.3f",
    //     msg->joint_position_1,
    //     msg->joint_position_2,
    //     msg->joint_position_3,
    //     msg->joint_position_4);

    current_tcp_[0] = msg->tcp_position_x;
    current_tcp_[1] = msg->tcp_position_y;
    current_tcp_[2] = msg->tcp_position_z;
    current_tcp_[3] = msg->tcp_position_u;

    current_joints_[0] = msg->joint_position_1;
    current_joints_[1] = msg->joint_position_2;
    current_joints_[2] = msg->joint_position_3;
    current_joints_[3] = msg->joint_position_4;
}


bool RobotControl::calculateMovement(const PointInfo &point, const TargetInfo &target)
{
    robot_msgs::msg::ArmControl msg;

    msg.mode = 1;

    RCLCPP_INFO(this->get_logger(),"--------------------------------------------" );
    RCLCPP_INFO(this->get_logger(),
        "PointInfo  -> index: %d, u: %d, v: %d, distance: %d",
        point.index, point.u, point.v, point.distance);
    RCLCPP_INFO(this->get_logger(),
        "TargetInfo -> index: %d, u: %d, v: %d, distance: %d",
        target.index, target.center_u, target.center_v, target.distance);

    if(point.u > target.center_u){
        msg.direction_y = -1;
    }
    else{
        msg.direction_y = 1;
    }

    if(point.v > target.center_v){
        msg.direction_z = 1;
    }
    else{
        msg.direction_z = -1;
    }

    if(point.distance > target.distance){
        msg.direction_x = 1;
    }
    else{
        msg.direction_x = -1;
    }

    if(abs(point.u - target.center_u) <= 5)
        msg.direction_y = 0;
    if(abs(point.v - target.center_v) <= 5)
        msg.direction_z = 0;
    if(abs(point.distance - target.distance) <= 5)
        msg.direction_x = 0;

    if(target.center_u == 0 || target.center_v == 0 || target.distance == 0 
        || target.center_u > 1280 || target.center_v > 720 || target.distance >= 1000
        || target.center_u < 0 || target.center_v < 0 || target.distance < 0)
    {
        msg.direction_y = 0;
        msg.direction_z = 0;
        msg.direction_x = 0;
    }

    if(abs(point.u - target.center_u) <= 5 && abs(point.v - target.center_v) <= 5 && abs(point.distance - target.distance) <= 5)
    {
        msg.direction_y = 0;
        msg.direction_z = 0;
        msg.direction_x = 0;

        pub_arm_ctl_->publish(msg);

        return true;
    }

    pub_arm_ctl_->publish(msg);

    return false;
}

bool RobotControl::calculatePointMovement(const std::array<double, 4> current_tcp_, const std::array<double, 4> target_tcp_)
{
    robot_msgs::msg::ArmControl msg;
    msg.mode = 0;   // 自定义模式，比如 2 表示点位控制

    RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
    RCLCPP_INFO(this->get_logger(),
        "Current -> x: %.3f, y: %.3f, z: %.3f, u: %.3f",
        current_tcp_[0], current_tcp_[1],
        current_tcp_[2], current_tcp_[3]);
    RCLCPP_INFO(this->get_logger(),
        "Target  -> x: %.3f, y: %.3f, z: %.2f, u: %.3f",
        target_tcp_[0], target_tcp_[1],
        target_tcp_[2], target_tcp_[3]);

    msg.position_x = target_tcp_[0];
    msg.position_y = target_tcp_[1];
    msg.position_z = target_tcp_[2];
    msg.position_u = target_tcp_[3];
    msg.direction_x = 0;
    msg.direction_y = 0;
    msg.direction_z = 0;
    msg.direction_u = 0.0;

    // 判断是否到达目标
    if(fabs(target_tcp_[0] - current_tcp_[0]) <= 5.0 && fabs(target_tcp_[1] - current_tcp_[1]) <= 5.0
        && fabs(target_tcp_[2] - current_tcp_[2]) <= 5.0 && fabs(target_tcp_[3] - current_tcp_[3]) <= 0.1)
    {
        pub_arm_ctl_->publish(msg);
        return true;
    }

    // 发布控制消息
    pub_arm_ctl_->publish(msg);
    return false;
}

void RobotControl::controlLoop()
{
    static auto last_time = std::chrono::steady_clock::now();

    if (work_phase == 0) {
        calculatePointMovement(current_tcp_,target_tcp_list_[0]);
    }
    else if (work_phase == 1) {
        static bool waiting = false;
        // 非阻塞等待 10 秒
        auto now = std::chrono::steady_clock::now();
        if (waiting) {
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_time).count() >= 10) {
                RCLCPP_INFO(this->get_logger(), "Wait finished, continue moving...");
                waiting = false; // 解除等待
            } else {
                return; // 等待未结束，直接退出，不执行下面逻辑
            }
        }

        if (calculateMovement(points_[index], target_.targets[index])) {
            index++;
            if (index >= 3) index = 3;

            last_time = std::chrono::steady_clock::now();
            waiting = true; // 下一次循环进入等待状态
            RCLCPP_INFO(this->get_logger(), "Reached point %d, waiting 10 seconds...", index);
        }
    }

}