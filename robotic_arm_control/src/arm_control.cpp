#include "arm_control.hpp"

// 构造函数
ArmControl::ArmControl()
    : Node("robotic_arm_control_node"),  // 初始化 ROS2 节点名称
      KeyboardReader(),                  // 初始化键盘输入父类
      pos_tcp(7, 0.0), pos_joint(7, 0.0) // 初始化 TCP 坐标和关节坐标数组（7 维）
{
    RCLCPP_INFO(this->get_logger(), "ArmControl node initialized!");

    // 订阅机械臂控制话题 /arm_control
    try
    {
        sub_control_ = this->create_subscription<robot_msgs::msg::ArmControl>(
            "/arm_control", 10,
            std::bind(&ArmControl::ArmControlCallback, this, std::placeholders::_1));
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create subscription: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // 发布机械臂状态话题 /arm_info
    pub_arm_info_ = this->create_publisher<robot_msgs::msg::ArmInfo>("arm_info", 10);

    // 连接机械臂控制器
    fd = connect_robot("192.168.2.14", "6001");
    if (fd <= 0)
    {
        std::cout << "连接失败" << std::endl;
        return;
    }
    std::cout << "连接成功: " << fd << std::endl;

    // 设置示教模式（模式0：示教）
    set_current_mode(fd, 0);
    // 设置全局速度
    set_speed(fd, 50);
    // 上电（伺服电机使能）
    power_on(fd);

    // 切换到运行模式（模式1：运行）
    set_current_mode(fd, 1);

    // 获取当前 TCP（末端）位置
    get_current_position(fd, 1, pos_tcp);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "当前直角坐标：" << pos_tcp[0] << " " << pos_tcp[1] << " " << pos_tcp[2] << " "
              << pos_tcp[3] << " " << pos_tcp[4] << " " << pos_tcp[5] << " " << pos_tcp[6] << std::endl;

    // 创建定时器，100ms 执行一次 arm_control_loop()
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ArmControl::arm_control_loop, this));
}

// 析构函数
ArmControl::~ArmControl()
{
    // 发布空的 ArmInfo，表示节点结束
    robot_msgs::msg::ArmInfo msg{};
    pub_arm_info_->publish(msg);

    // 回到示教模式并下电
    set_current_mode(fd, 0);
    set_servo_poweroff(fd);
}

/*
 * 上电函数：确保伺服电机进入可运行状态
 */
void ArmControl::power_on(int fd)
{
    int state = 0;
    get_servo_state(fd, state); // 查询伺服状态
    switch (state)
    {
    case 0: // 停止状态
        set_servo_state(fd, 1);
        set_servo_poweron(fd);
        break;
    case 1: // 就绪状态
        set_servo_poweron(fd);
        break;
    case 2: // 报警状态
        clear_error(fd);
        set_servo_state(fd, 1);
        set_servo_poweron(fd);
        break;
    }
    get_servo_state(fd, state);
    std::cout << "伺服状态: " << state << std::endl;
}

/*
 * 阻塞等待机械臂运行结束（带超时保护）
 */
void ArmControl::wait_for_running_over(int fd, int timeout_ms)
{
    int running_state = 0;
    get_robot_running_state(fd, running_state);

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && running_state == 2) // 2 表示运行中
    {
        usleep(500 * 1000); // 每 500ms 检查一次
        get_robot_running_state(fd, running_state);

        // 超时检查
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed > timeout_ms)
        {
            RCLCPP_WARN(this->get_logger(), "wait_for_running_over timeout after %d ms", timeout_ms);
            break;
        }
    }
}

/*
 * 设置运动目标点
 */
void ArmControl::loadpos(MoveCmd &m, std::vector<double> pos, int coord,
                         double velocity, double acc, double dcc)
{
    m.coord = coord;                                 // 坐标系（0=关节坐标，1=直角坐标）
    m.targetPosType = PosType::data;                 // 使用数据直接指定目标点
    m.acc = acc;                                     // 加速度
    m.dec = dcc;                                     // 减速度
    m.velocity = velocity;                           // 速度
    m.targetPosValue.assign(pos.begin(), pos.end()); // 目标位置
    m.pl = 5;                                        // 路径平滑等级
}

/*
 * 订阅回调：接收 /arm_control 消息，更新控制数据
 */
void ArmControl::ArmControlCallback(const robot_msgs::msg::ArmControl::SharedPtr msg)
{
    arm_data.mode = msg->mode;
    arm_data.work_phase = msg->work_phase;
    arm_data.position_x = msg->position_x;
    arm_data.position_y = msg->position_y;
    arm_data.position_z = msg->position_z;
    arm_data.position_u = msg->position_u;
    arm_data.direction_x = msg->direction_x;
    arm_data.direction_y = msg->direction_y;
    arm_data.direction_z = msg->direction_z;
    arm_data.direction_u = msg->direction_u;
}

/*
 * 点动控制函数
 */
void ArmControl::arm_control_jog(ArmControlData data)
{
    // 获取当前 TCP 坐标
    res = get_current_position(fd, 1, pos_tcp);
    if (res == SUCCESS)
    {
        for (double p : pos_tcp)
            std::cout << p << " ";
        std::cout << std::endl;
    }
    else
    {
        std::cout << "获取位置失败: " << res << std::endl;
    }

    // mode = -1：停止
    if (data.mode == -1)
    {
        robot_stop_jogging(fd, 1);
        robot_stop_jogging(fd, 2);
        robot_stop_jogging(fd, 3);
    }
    // mode = 1：点动
    else if (data.mode == 1)
    {
        // X 方向
        if (data.direction_x != 0 && cnt % 3 == 1)
            robot_start_jogging(fd, 1, data.direction_x > 0);
        else
            robot_stop_jogging(fd, 1);

        // Y 方向
        if (data.direction_y != 0 && cnt % 3 == 2)
            robot_start_jogging(fd, 2, data.direction_y < 0);
        else
            robot_stop_jogging(fd, 2);

        // Z 方向
        if (data.direction_z != 0 && cnt % 3 == 0)
            robot_start_jogging(fd, 3, data.direction_z < 0);
        else
            robot_stop_jogging(fd, 3);

        // 三个方向都为 0，强制停止
        if (data.direction_x == 0 && data.direction_y == 0 && data.direction_z == 0)
        {
            robot_stop_jogging(fd, 1);
            robot_stop_jogging(fd, 2);
            robot_stop_jogging(fd, 3);
        }
    }
}

/*
 * 控制循环（100ms 调用一次）
 */
void ArmControl::arm_control_loop()
{
    // 获取当前位置（TCP + 关节）
    double new_x, new_y, new_z, new_u;
    robot_msgs::msg::ArmInfo msg;

    get_current_position(fd, 1, pos_tcp);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "当前直角坐标：" << pos_tcp[0] << " " << pos_tcp[1] << " " << pos_tcp[2] << " "
              << pos_tcp[3] << " " << pos_tcp[4] << " " << pos_tcp[5] << " " << pos_tcp[6] << std::endl;
    msg.tcp_position_x = pos_tcp[0];
    msg.tcp_position_y = pos_tcp[1];
    msg.tcp_position_z = pos_tcp[2];
    msg.tcp_position_u = pos_tcp[5];

    get_current_position(fd, 0, pos_joint);
    std::cout << "当前关节坐标：" << pos_joint[0] << " " << pos_joint[1] << " " << pos_joint[2] << " "
              << pos_joint[3] << " " << pos_joint[4] << " " << pos_joint[5] << " " << pos_joint[6] << std::endl;
    msg.joint_position_1 = pos_joint[0];
    msg.joint_position_2 = pos_joint[1];
    msg.joint_position_3 = pos_joint[2];
    msg.joint_position_4 = pos_joint[3];

    // 打印接收到的控制数据
    std::cout << "ArmControl msg received: "
              << "mode=" << arm_data.mode
              << " pos=(" << arm_data.position_x << ", " << arm_data.position_y
              << ", " << arm_data.position_z << ", u=" << arm_data.position_u << ")"
              << " dir=(" << arm_data.direction_x << ", " << arm_data.direction_y
              << ", " << arm_data.direction_z << ", u=" << arm_data.direction_u << ")"
              << std::endl;

    // 发布状态消息
    pub_arm_info_->publish(msg);

    // 控制逻辑
    switch (arm_data.mode)
    {
    case -1: // 停止
        break;
    case 0: // 目标位置控制
        new_x = arm_data.position_x;
        new_y = arm_data.position_y;
        new_z = arm_data.position_z;
        new_u = arm_data.position_u;
        break;
    case 1: // 点动（相对运动）
        new_x = pos_tcp[0] + (double)arm_data.direction_x * 2;
        new_y = pos_tcp[1] + (double)arm_data.direction_y * 2;
        new_z = pos_tcp[2] + (double)arm_data.direction_z * 2;
        new_u = pos_tcp[5] + arm_data.direction_u * 0.01;
        break;
    default:
        RCLCPP_WARN(rclcpp::get_logger("controlArm"), "Unknown mode: %d", arm_data.mode);
        break;
    }

    // 越界保护（避免机械臂运动到危险区域）
    double r = std::sqrt(new_x * new_x + new_y * new_y);
    if (new_x >= -50.0 && new_y >= -50.0 && r <= 580.0 &&
        new_z >= -295.0 && new_z <= -100.0)
    {
        pos_tcp[0] = new_x;
        pos_tcp[1] = new_y;
        pos_tcp[2] = new_z;
        pos_tcp[5] = new_u;
    }
    else
    {
        std::cout << "⚠️ 越界: x=" << new_x << " y=" << new_y << " r=" << r << " 忽略更新" << std::endl;
    }

    // 执行运动(运行速度=全局速度*设定速度)
    if (arm_data.mode == 0) // 只要不是停止模式，就执行运动
    {
        loadpos(move_cmd, pos_tcp, 1, 80, 100, 100);
        robot_movel(fd, move_cmd); // 下发运动指令
    }
    else if (arm_data.mode == 1)
    {
        loadpos(move_cmd, pos_tcp, 1, 50, 100, 100);
        robot_movel(fd, move_cmd); // 下发运动指令
    }
}
