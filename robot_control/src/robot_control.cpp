#include "robot_control/robot_control.hpp" // 引入 RobotControl 类头文件

// 构造函数，初始化 RobotControl 节点
RobotControl::RobotControl() : Node("robot_control_node")
{
    // 初始化四个乳头目标位置，PointInfo {index, u, v, distance}
    points_.at(0) = {1, 750, 395, 370};
    points_.at(1) = {2, 640, 370, 360};
    points_.at(2) = {3, 955, 315, 305};
    points_.at(3) = {4, 565, 315, 315};
    // 初始化药浴目标位置
    points_.at(4) = {5, 710, 390, 370};
    // 初始化滚刷目标位置
    points_.at(5) = {6, 700, 450, 480};
    // 初始化点位控制目标 TCP 列表 {x, y, z, u}
    target_tcp_list_[0] = {450.0, -25.0, -280.0, 2.26};
    target_tcp_list_[1] = {230.0, 425.0, -280.0, 1.61};
    target_tcp_list_[2] = {365.0, 215.0, -290.0, 3.14};
    target_tcp_list_[3] = {365.0, 215.0, -290.0, 3.14};
    // 初始化继电器控制信息长度为 3，并置 0
    rc_ctl_.data.resize(3, 0);
    // 初始化工作阶段
    work_phase = 0;
    // 初始化等待标志位
    waiting = false;
    // 初始化时间点，用于等待判断
    now = std::chrono::steady_clock::now();
    last_time = std::chrono::steady_clock::now();

    cnt = 1; // 继电器计数器

    // 创建目标信息订阅器，接收 "/target_info" 消息
    sub_target_ = this->create_subscription<robot_msgs::msg::TargetArray>(
        "/target_info", 10,
        std::bind(&RobotControl::targetCallback, this, std::placeholders::_1));

    // 创建机械臂信息订阅器，接收 "/arm_info" 消息
    sub_arm_info_ = this->create_subscription<robot_msgs::msg::ArmInfo>(
        "/arm_info", 10,
        std::bind(&RobotControl::arminfoCallback, this, std::placeholders::_1));

    // 创建机械臂控制发布器
    pub_arm_ctl_ = this->create_publisher<robot_msgs::msg::ArmControl>("arm_control", 10);
    // 创建继电器控制发布器
    pub_rc_ctl_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("relay_control", 10);

    // 创建定时器，周期 100ms，循环调用 controlLoop()
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotControl::controlLoop, this));
}

// 析构函数，发布清零消息
RobotControl::~RobotControl()
{
    robot_msgs::msg::ArmControl arm_msg{};   // 机械臂控制消息置零
    std_msgs::msg::UInt8MultiArray rc_msg{}; // 继电器控制消息置零

    pub_arm_ctl_->publish(arm_msg); // 发布机械臂清零消息
    pub_rc_ctl_->publish(rc_msg);   // 发布继电器清零消息
}

// 目标信息回调函数
void RobotControl::targetCallback(const robot_msgs::msg::TargetArray::SharedPtr msg)
{
    // 保存总目标数量
    target_.total_count = msg->total_count;

    // 遍历接收到的目标数组
    for (size_t i = 0; i < msg->targets.size(); i++)
    {
        auto &t = msg->targets[i];

        target_.targets[i].index = t.index;       // 目标索引
        target_.targets[i].center_u = t.center_u; // 像素 u 坐标
        target_.targets[i].center_v = t.center_v; // 像素 v 坐标
        target_.targets[i].distance = t.distance; // 距离信息
        // RCLCPP_INFO(this->get_logger(),
        //             "Target %d: center=(%d,%d), distance=%d",
        //             target_.targets[i].index, target_.targets[i].center_u, target_.targets[i].center_v, target_.targets[i].distance);
    }
}

// 机械臂信息回调函数
void RobotControl::arminfoCallback(const robot_msgs::msg::ArmInfo::SharedPtr msg)
{
    // 打印 TCP 位姿信息
    RCLCPP_INFO(this->get_logger(),
                "TCP Position -> x: %.3f, y: %.3f, z: %.3f, u: %.3f",
                msg->tcp_position_x,
                msg->tcp_position_y,
                msg->tcp_position_z,
                msg->tcp_position_u);

    // 打印关节位置信息
    RCLCPP_INFO(this->get_logger(),
                "Joint Position -> j1: %.3f, j2: %.3f, j3: %.3f, j4: %.3f",
                msg->joint_position_1,
                msg->joint_position_2,
                msg->joint_position_3,
                msg->joint_position_4);

    // 保存 TCP 坐标
    current_tcp_[0] = msg->tcp_position_x;
    current_tcp_[1] = msg->tcp_position_y;
    current_tcp_[2] = msg->tcp_position_z;
    current_tcp_[3] = msg->tcp_position_u;
    // 保存关节坐标
    current_joints_[0] = msg->joint_position_1;
    current_joints_[1] = msg->joint_position_2;
    current_joints_[2] = msg->joint_position_3;
    current_joints_[3] = msg->joint_position_4;
}

// 根据目标像素信息计算机械臂移动方向
bool RobotControl::calculateMovement(const PointInfo &point, const TargetInfo &target)
{
    robot_msgs::msg::ArmControl msg;
    msg.mode = 1;                // 设置模式为位置调节
    msg.work_phase = work_phase; // 当前工作阶段

    RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
    // 打印点位信息
    RCLCPP_INFO(this->get_logger(),
                "PointInfo  -> index: %d, u: %d, v: %d, distance: %d",
                point.index, point.u, point.v, point.distance);
    // 打印目标信息
    RCLCPP_INFO(this->get_logger(),
                "TargetInfo -> index: %d, u: %d, v: %d, distance: %d",
                target.index, target.center_u, target.center_v, target.distance);

    // 根据像素 u 坐标调整 y 方向
    if (point.u > target.center_u)
        msg.direction_y = -1;
    else
        msg.direction_y = 1;

    // 根据像素 v 坐标调整 z 方向
    if (point.v > target.center_v)
        msg.direction_z = 1;
    else
        msg.direction_z = -1;

    // 根据距离调整 x 方向
    if (point.distance > target.distance)
        msg.direction_x = 1;
    else
        msg.direction_x = -1;

    // 像素误差小于阈值，则方向置零
    if (abs(point.u - target.center_u) <= 5)
        msg.direction_y = 0;
    if (abs(point.v - target.center_v) <= 5)
        msg.direction_z = 0;
    if (abs(point.distance - target.distance) <= 5)
        msg.direction_x = 0;

    // 如果目标无效（越界或为零），则方向置零
    if (target.center_u == 0 || target.center_v == 0 || target.distance == 0 || target.center_u > 1280 || target.center_v > 720 || target.distance >= 1000 || target.center_u < 0 || target.center_v < 0 || target.distance < 0)
    {
        msg.direction_y = 0;
        msg.direction_z = 0;
        msg.direction_x = 0;
    }

    // 如果到达目标位置
    if (abs(point.u - target.center_u) <= 5 && abs(point.v - target.center_v) <= 5 && abs(point.distance - target.distance) <= 5)
    {
        msg.direction_y = 0;
        msg.direction_z = 0;
        msg.direction_x = 0;
        pub_arm_ctl_->publish(msg); // 发布机械臂控制消息
        return true;                // 到达目标
    }

    pub_arm_ctl_->publish(msg); // 发布控制消息
    return false;               // 未到达目标
}

// 根据 TCP 点位信息移动机械臂
bool RobotControl::calculatePointMovement(const std::array<double, 4> current_tcp_, const std::array<double, 4> target_tcp_)
{
    robot_msgs::msg::ArmControl msg;
    msg.mode = 0; // 自定义模式
    msg.work_phase = work_phase;

    // 打印当前与目标 TCP
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
    RCLCPP_INFO(this->get_logger(),
                "Current -> x: %.3f, y: %.3f, z: %.3f, u: %.3f",
                current_tcp_[0], current_tcp_[1],
                current_tcp_[2], current_tcp_[3]);
    RCLCPP_INFO(this->get_logger(),
                "Target  -> x: %.3f, y: %.3f, z: %.2f, u: %.3f",
                target_tcp_[0], target_tcp_[1],
                target_tcp_[2], target_tcp_[3]);

    // 填写目标位置信息
    msg.position_x = target_tcp_[0];
    msg.position_y = target_tcp_[1];
    msg.position_z = target_tcp_[2];
    msg.position_u = target_tcp_[3];
    // 方向置零
    msg.direction_x = 0;
    msg.direction_y = 0;
    msg.direction_z = 0;
    msg.direction_u = 0.0;

    // 判断是否到达目标 TCP
    if (fabs(target_tcp_[0] - current_tcp_[0]) <= 5.0 && fabs(target_tcp_[1] - current_tcp_[1]) <= 5.0 && fabs(target_tcp_[2] - current_tcp_[2]) <= 5.0 && fabs(target_tcp_[3] - current_tcp_[3]) <= 0.1)
    {
        pub_arm_ctl_->publish(msg); // 发布控制消息
        return true;                // 到达目标
    }

    pub_arm_ctl_->publish(msg); // 发布控制消息
    return false;               // 未到达目标
}

// 等待检查函数，非阻塞等待
bool RobotControl::check_wait(int seconds)
{
    if (waiting) // 如果处于等待状态
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_time).count();
        if (elapsed >= seconds)
        {
            RCLCPP_INFO(this->get_logger(), "Wait %d seconds finished, continue moving...", seconds);
            waiting = false; // 自动解除等待
            return true;     // 等待完成
        }
        else
        {
            int remaining = seconds - static_cast<int>(elapsed);
            RCLCPP_INFO(this->get_logger(), "Waiting... %d seconds remaining", remaining);
            return false; // 还在等待
        }
    }
    return true; // 不在等待状态，直接继续
}


// 设置继电器控制消息
void RobotControl::setRcCtl(const std::array<uint8_t, 10> &input, std_msgs::msg::UInt8MultiArray &rc_ctl)
{
    // 检查第一个元素是否合法
    if (input[0] < 1 || input[0] > 4)
    {
        throw std::invalid_argument("First element must be 1-4");
    }
    rc_ctl.data[0] = input[0]; // 设置第 0 个字节

    // 设置第 1 个字节，低 6 位对应 input[1]~input[6]
    rc_ctl.data[1] = (input[1] & 1) << 0 |
                     (input[2] & 1) << 1 |
                     (input[3] & 1) << 2 |
                     (input[4] & 1) << 3 |
                     (input[5] & 1) << 4 |
                     (input[6] & 1) << 5;

    // 设置第 2 个字节，低 3 位对应 input[7]~input[9]
    rc_ctl.data[2] = (input[7] & 1) << 0 |
                     (input[8] & 1) << 1 |
                     (input[9] & 1) << 2;

    // 发布继电器控制消息
    pub_rc_ctl_->publish(rc_ctl);
}

// 主控制循环
void RobotControl::controlLoop()
{
    now = std::chrono::steady_clock::now(); // 获取当前时间

    rc_input = {};                                        // 清空继电器输入数组
    rc_input = {(uint8_t)cnt, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 默认继电器状态

    // 测试模式（work_phase == -1）
    if (work_phase == -1)
    {
        // 可以手动设置 rc_input 测试继电器
    }
    // 阶段1：机械臂到初始位置
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
    // 阶段2：药浴
    else if (work_phase == 1)
    {
        if (!check_wait(2)) // 阻塞等待 2 秒
        {
            return;
        }

        if (calculateMovement(points_[4], target_.targets[index]))
        {
            index++;
            last_time = std::chrono::steady_clock::now();
            waiting = true; // 下一循环进入等待状态
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
    // 阶段3：洗刷乳头
    else if (work_phase == 2)
    {
        if (!check_wait(5)) // 阻塞等待 5 秒
        {
            return;
        }

        if (calculateMovement(points_[5], target_.targets[index]))
        {
            index++;
            last_time = std::chrono::steady_clock::now();
            waiting = true; // 下一循环进入等待状态
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
    // 阶段4：套杯
    else if (work_phase == 3)
    {
        if (!check_wait(5)) // 非阻塞等待 5 秒
        {
            return;
        }

        if (calculateMovement(points_[index], target_.targets[index]))
        {
            index++;
            if (index > 3)
                index = 4;

            last_time = std::chrono::steady_clock::now();
            waiting = true; // 下一循环进入等待状态
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

    // 继电器周期计数
    cnt++;
    if (cnt > 4)
        cnt = 1;

    RCLCPP_INFO(this->get_logger(), "work_phase = %d cnt = %d", work_phase, cnt); // 打印当前阶段与计数
}
