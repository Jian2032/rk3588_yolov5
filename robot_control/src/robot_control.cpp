#include "robot_control/robot_control.hpp"

RobotControl::RobotControl() : Node("robot_control_node")
{
    // 初始化四个乳头目标位置
    points_.at(0) = {1, 750, 395, 370};
    points_.at(1) = {2, 640, 370, 360};
    points_.at(2) = {3, 955, 315, 305};
    points_.at(3) = {4, 565, 315, 315};
    // 初始化药浴目标位置
    points_.at(4) = {5, 710, 390, 370};
    // 初始化滚刷目标位置
    points_.at(5) = {6, 700, 450, 480};
    // 初始化点位控制目标点
    target_tcp_list_[0] = {450.0, -25.0, -280.0, 2.26};
    target_tcp_list_[1] = {230.0, 425.0, -280.0, 1.61};
    target_tcp_list_[2] = {365.0, 215.0, -290.0, 3.14};
    target_tcp_list_[3] = {365.0, 215.0, -290.0, 3.14};
    // 初始化继电器控制信息长度
    rc_ctl_.data.resize(3, 0);
    // 初始化工作阶段
    work_phase = 0;
    // 初始化等待标志位
    waiting = false;
    // 初始化现在时间
    now = std::chrono::steady_clock::now();
    last_time = std::chrono::steady_clock::now();

    cnt = 1;

    sub_target_ = this->create_subscription<robot_msgs::msg::TargetArray>(
        "/target_info", 10,
        std::bind(&RobotControl::targetCallback, this, std::placeholders::_1));

    sub_arm_info_ = this->create_subscription<robot_msgs::msg::ArmInfo>(
        "/arm_info", 10,
        std::bind(&RobotControl::arminfoCallback, this, std::placeholders::_1));

    pub_arm_ctl_ = this->create_publisher<robot_msgs::msg::ArmControl>("arm_control", 10);
    pub_rc_ctl_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("relay_control", 10);

    // 创建定时器 (100ms 周期)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotControl::controlLoop, this));
}

RobotControl::~RobotControl()
{
    // 发布清零消息
    robot_msgs::msg::ArmControl arm_msg{};
    std_msgs::msg::UInt8MultiArray rc_msg{};

    pub_arm_ctl_->publish(arm_msg);
    pub_rc_ctl_->publish(rc_msg);
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
    // 笛卡尔坐标信息
    current_tcp_[0] = msg->tcp_position_x;
    current_tcp_[1] = msg->tcp_position_y;
    current_tcp_[2] = msg->tcp_position_z;
    current_tcp_[3] = msg->tcp_position_u;
    // 关节坐标信息
    current_joints_[0] = msg->joint_position_1;
    current_joints_[1] = msg->joint_position_2;
    current_joints_[2] = msg->joint_position_3;
    current_joints_[3] = msg->joint_position_4;
}

bool RobotControl::calculateMovement(const PointInfo &point, const TargetInfo &target)
{
    robot_msgs::msg::ArmControl msg;

    msg.mode = 1;

    msg.work_phase = work_phase;

    RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
    RCLCPP_INFO(this->get_logger(),
                "PointInfo  -> index: %d, u: %d, v: %d, distance: %d",
                point.index, point.u, point.v, point.distance);
    RCLCPP_INFO(this->get_logger(),
                "TargetInfo -> index: %d, u: %d, v: %d, distance: %d",
                target.index, target.center_u, target.center_v, target.distance);
    // 根据目标位置像素点和距离信息调整机械臂
    if (point.u > target.center_u)
    {
        msg.direction_y = -1;
    }
    else
    {
        msg.direction_y = 1;
    }

    if (point.v > target.center_v)
    {
        msg.direction_z = 1;
    }
    else
    {
        msg.direction_z = -1;
    }

    if (point.distance > target.distance)
    {
        msg.direction_x = 1;
    }
    else
    {
        msg.direction_x = -1;
    }

    if (abs(point.u - target.center_u) <= 5)
        msg.direction_y = 0;
    if (abs(point.v - target.center_v) <= 5)
        msg.direction_z = 0;
    if (abs(point.distance - target.distance) <= 5)
        msg.direction_x = 0;

    if (target.center_u == 0 || target.center_v == 0 || target.distance == 0 || target.center_u > 1280 || target.center_v > 720 || target.distance >= 1000 || target.center_u < 0 || target.center_v < 0 || target.distance < 0)
    {
        msg.direction_y = 0;
        msg.direction_z = 0;
        msg.direction_x = 0;
    }
    // 到达目标位置
    if (abs(point.u - target.center_u) <= 5 && abs(point.v - target.center_v) <= 5 && abs(point.distance - target.distance) <= 5)
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
    msg.mode = 0; // 自定义模式
    msg.work_phase = work_phase;

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
    if (fabs(target_tcp_[0] - current_tcp_[0]) <= 5.0 && fabs(target_tcp_[1] - current_tcp_[1]) <= 5.0 && fabs(target_tcp_[2] - current_tcp_[2]) <= 5.0 && fabs(target_tcp_[3] - current_tcp_[3]) <= 0.1)
    {
        pub_arm_ctl_->publish(msg);
        return true;
    }

    // 发布控制消息
    pub_arm_ctl_->publish(msg);
    return false;
}

bool RobotControl::check_wait(int seconds)
{
    if (waiting)
    {
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_time).count() >= seconds)
        {
            RCLCPP_INFO(this->get_logger(), "Wait %d seconds finished, continue moving...", seconds);
            waiting = false; // 自动解除等待
            return true;     // 表示等待完成
        }
        else
        {
            return false; // 还在等待
        }
    }
    return true; // 不在等待状态，直接继续
}

void RobotControl::setRcCtl(const std::array<uint8_t, 10> &input, std_msgs::msg::UInt8MultiArray &rc_ctl)
{
    if (input[0] < 1 || input[0] > 4)
    {
        throw std::invalid_argument("First element must be 1-4");
    }
    rc_ctl.data[0] = input[0];

    rc_ctl.data[1] = (input[1] & 1) << 0 |
                     (input[2] & 1) << 1 |
                     (input[3] & 1) << 2 |
                     (input[4] & 1) << 3 |
                     (input[5] & 1) << 4 |
                     (input[6] & 1) << 5;

    rc_ctl.data[2] = (input[7] & 1) << 0 |
                     (input[8] & 1) << 1 |
                     (input[9] & 1) << 2;

    pub_rc_ctl_->publish(rc_ctl);
}

void RobotControl::controlLoop()
{
    // 记录当前时间
    now = std::chrono::steady_clock::now();
    // 继电器消息清零
    rc_input = {};
    // 继电器消息赋值
    rc_input = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // work_phase = 3;
    // 测试
    if (work_phase == -1)
    {
        // rc_input = {4, 1, 1, 0, 0, 0, 0, 0, 0, 0};
        // rc_input = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
        // setRcCtl(rc_input, rc_ctl_);
    }
    // 阶段1：控制机械臂到初始位置
    else if (work_phase == 0)
    {
        if (calculatePointMovement(current_tcp_, target_tcp_list_[index]))
        {
            index++;
        }
        if (index == 2)
        {
            work_phase = 1;
            index = 0;
        }
        rc_input = {(uint8_t)cnt, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        setRcCtl(rc_input, rc_ctl_);
    }
    // 阶段2：开始药浴
    else if (work_phase == 1)
    {
        // 阻塞2s
        if (!check_wait(2))
        {
            return;
        }

        if (calculateMovement(points_[4], target_.targets[index]))
        {
            index++;

            last_time = std::chrono::steady_clock::now();
            waiting = true; // 下一次循环进入等待状态
            RCLCPP_INFO(this->get_logger(), "Reached point %d, waiting 2 seconds...", index);
        }

        if (index == 4)
        {
            work_phase = 2;
            index = 0;
        }

        rc_input = {(uint8_t)cnt, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        setRcCtl(rc_input, rc_ctl_);
    }
    // 阶段3：开始洗刷乳头
    else if (work_phase == 2)
    {
        // 阻塞5s
        if (!check_wait(5))
        {
            return;
        }

        if (calculateMovement(points_[5], target_.targets[index]))
        {
            index++;

            last_time = std::chrono::steady_clock::now();
            waiting = true; // 下一次循环进入等待状态
            RCLCPP_INFO(this->get_logger(), "Reached point %d, waiting 2 seconds...", index);
        }

        if (index == 4)
        {
            work_phase = 3;
            index = 0;
        }
        rc_input = {(uint8_t)cnt, 0, 1, 0, 0, 0, 0, 1, 0, 0};
        setRcCtl(rc_input, rc_ctl_);
    }
    // 阶段4：开始套杯
    else if (work_phase == 3)
    {
        // 非阻塞等待 5 秒
        if (!check_wait(5))
        {
            return;
        }

        if (calculateMovement(points_[index], target_.targets[index]))
        {
            index++;
            if (index > 3)
                index = 4;

            last_time = std::chrono::steady_clock::now();
            waiting = true; // 下一次循环进入等待状态
            RCLCPP_INFO(this->get_logger(), "Reached point %d, waiting 5 seconds...", index);
        }

        if (waiting == true)
        {
            rc_input = {(uint8_t)(index), 1, 0, 0, 0, 0, 0, 0, 0, 0};
            setRcCtl(rc_input, rc_ctl_);
        }
        else
        {
            rc_input = {(uint8_t)(index + 1), 0, 1, 0, 0, 0, 0, 0, 0, 0};
            setRcCtl(rc_input, rc_ctl_);
        }
    }

    // 继电器周期
    cnt++;
    if (cnt > 4)
        cnt = 1;
    RCLCPP_INFO(this->get_logger(), "work_phase = %d cnt = %d", work_phase, cnt);
}