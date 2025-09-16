#include "arm_control.hpp"

ArmControl::ArmControl()
    : Node("robotic_arm_control_node"), KeyboardReader(), pos_tcp(7, 0.0), pos_joint(7, 0.0)
{
    RCLCPP_INFO(this->get_logger(), "ArmControl node initialized!");

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
    pub_arm_info_ = this->create_publisher<robot_msgs::msg::ArmInfo>("arm_info", 10);

    fd = connect_robot("192.168.2.14", "6001");
    if (fd <= 0)
    {
        std::cout << "连接失败" << std::endl;
        return;
    }
    std::cout << "连接成功: " << fd << std::endl;

    // 设置示教模式，只有示教模式下的运动需要上电
    set_current_mode(fd, 0);
    // 设置示教模式下的全局速度
    set_speed(fd, 50);
    // 调用封装的上电函数
    power_on(fd);

    set_current_mode(fd, 1);
    // 获取当前位置
    get_current_position(fd, 1, pos_tcp);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "当前直角坐标：" << pos_tcp[0] << " " << pos_tcp[1] << " " << pos_tcp[2] << " " << pos_tcp[3] << " " << pos_tcp[4] << " " << pos_tcp[5] << " " << pos_tcp[6] << std::endl;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ArmControl::arm_control_loop, this));
}

ArmControl::~ArmControl()
{
    robot_msgs::msg::ArmInfo msg{};
    pub_arm_info_->publish(msg);

    set_current_mode(fd, 0);
    set_servo_poweroff(fd);
}

/*
 * 封装上电函数
 */
void ArmControl::power_on(int fd)
{
    int state = 0;
    get_servo_state(fd, state); // 查询当前伺服状态
    switch (state)
    {
    case 0:                     // 当前伺服处于停止
        set_servo_state(fd, 1); // 将伺服设置为就绪
        set_servo_poweron(fd);  // 将伺服上电
        break;
    case 1: // 当前伺服处于就绪
        set_servo_poweron(fd);
        break;
    case 2:              // 当前伺服处于报警
        clear_error(fd); // 清除错误
        set_servo_state(fd, 1);
        set_servo_poweron(fd);
        break;
    }
    get_servo_state(fd, state);
    std::cout << "伺服状态: " << state << std::endl;
}

/*
 * 循环阻塞运动结束函数
 */
void ArmControl::wait_for_running_over(int fd, int timeout_ms)
{
    int running_state = 0;
    get_robot_running_state(fd, running_state);

    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && running_state == 2) // 检查 ROS2 节点是否仍在运行
    {
        usleep(500 * 1000); // 500ms

        get_robot_running_state(fd, running_state);

        // 超时保护
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed > timeout_ms)
        {
            RCLCPP_WARN(this->get_logger(), "wait_for_running_over timeout after %d ms", timeout_ms);
            break;
        }
    }
}

// 设置目标点
void ArmControl::loadpos(MoveCmd &m, std::vector<double> pos, int coord, double velocity, double acc, double dcc)
{
    m.coord = coord;
    m.targetPosType = PosType::data;
    m.acc = acc;
    m.dec = dcc;
    m.velocity = velocity;
    m.targetPosValue.assign(pos.begin(), pos.end());
}

void ArmControl::ArmControlCallback(const robot_msgs::msg::ArmControl::SharedPtr msg)
{
    arm_data.mode = msg->mode;
    arm_data.position_x = msg->position_x;
    arm_data.position_y = msg->position_y;
    arm_data.position_z = msg->position_z;
    arm_data.position_u = msg->position_u;
    arm_data.direction_x = msg->direction_x;
    arm_data.direction_y = msg->direction_y;
    arm_data.direction_z = msg->direction_z;
    arm_data.direction_u = msg->direction_u;
}

void ArmControl::arm_control_jog(ArmControlData data)
{
    res = get_current_position(fd, 1, pos_tcp); // coord=1:直角坐标
    if (res == SUCCESS)
    {
        for (double p : pos_tcp)
        {
            std::cout << p << " ";
        }
        std::cout << std::endl;
    }
    else
    {
        std::cout << "获取位置失败: " << res << std::endl;
    }

    if (data.mode == -1)
    {
        robot_stop_jogging(fd, 1);
        robot_stop_jogging(fd, 2);
        robot_stop_jogging(fd, 3);
    }
    else if (data.mode == 1)
    {
        // X轴
        if (data.direction_x != 0 && cnt % 3 == 1)
        {
            robot_start_jogging(fd, 1, data.direction_x > 0);
        }
        else
        {
            robot_stop_jogging(fd, 1);
        }
        // Y轴
        if (data.direction_y != 0 && cnt % 3 == 2)
        {
            robot_start_jogging(fd, 2, data.direction_y < 0);
        }
        else
        {
            robot_stop_jogging(fd, 2);
        }
        // Z轴
        if (data.direction_z != 0 && cnt % 3 == 0)
        {
            robot_start_jogging(fd, 3, data.direction_z < 0);
        }
        else
        {
            robot_stop_jogging(fd, 3);
        }
        // 如果三个方向都是 0，就停止
        if (data.direction_x == 0 && data.direction_y == 0 && data.direction_z == 0)
        {
            robot_stop_jogging(fd, 1);
            robot_stop_jogging(fd, 2);
            robot_stop_jogging(fd, 3);
        }
    }
}

void ArmControl::arm_control_loop()
{
    // 获取当前位置
    double new_x, new_y, new_z, new_u;
    robot_msgs::msg::ArmInfo msg;

    get_current_position(fd, 1, pos_tcp);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "当前直角坐标：" << pos_tcp[0] << " " << pos_tcp[1] << " " << pos_tcp[2] << " " << pos_tcp[3] << " " << pos_tcp[4] << " " << pos_tcp[5] << " " << pos_tcp[6] << std::endl;
    msg.tcp_position_x = pos_tcp[0];
    msg.tcp_position_y = pos_tcp[1];
    msg.tcp_position_z = pos_tcp[2];
    msg.tcp_position_u = pos_tcp[5];
    get_current_position(fd, 0, pos_joint);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "当前关节坐标：" << pos_joint[0] << " " << pos_joint[1] << " " << pos_joint[2] << " " << pos_joint[3] << " " << pos_joint[4] << " " << pos_joint[5] << " " << pos_joint[6] << std::endl;
    msg.joint_position_1 = pos_joint[0];
    msg.joint_position_2 = pos_joint[1];
    msg.joint_position_3 = pos_joint[2];
    msg.joint_position_4 = pos_joint[3];

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "ArmControl msg received: "
              << "mode=" << arm_data.mode
              << " pos=(" << arm_data.position_x
              << ", " << arm_data.position_y
              << ", " << arm_data.position_z
              << ", u=" << arm_data.position_u << ")"
              << " dir=(" << arm_data.direction_x
              << ", " << arm_data.direction_y
              << ", " << arm_data.direction_z
              << ", u=" << arm_data.direction_u << ")"
              << std::endl;

    pub_arm_info_->publish(msg);

    switch (arm_data.mode)
    {
    case -1:
        break;
    case 0:
        new_x = (double)arm_data.position_x;
        new_y = (double)arm_data.position_y;
        new_z = (double)arm_data.position_z;
        new_u = arm_data.position_u;
        break;
    case 1:
        new_x = pos_tcp[0] + (double)arm_data.direction_x * 2;
        new_y = pos_tcp[1] + (double)arm_data.direction_y * 2;
        new_z = pos_tcp[2] + (double)arm_data.direction_z * 2;
        new_u = pos_tcp[5] + arm_data.direction_u * 0.01;
        break;
    default:
        RCLCPP_WARN(rclcpp::get_logger("controlArm"), "Unknown mode: %d", arm_data.mode);
        break;
    }

    double r = std::sqrt(new_x * new_x + new_y * new_y);
    if (new_x >= -50.0 && new_y >= -50.0 && r <= 575.0 && new_z >= -295.0 && new_z <= -230.0)
    {
        pos_tcp[0] = new_x;
        pos_tcp[1] = new_y;
        pos_tcp[2] = new_z;
        pos_tcp[5] = new_u;
    }
    else
    {
        std::cout << "⚠️ 越界: x=" << new_x
                  << " y=" << new_y
                  << " r=" << r
                  << " 忽略更新" << std::endl;
    }

    if (arm_data.mode != -1)
    {
        loadpos(move_cmd, pos_tcp, 1, 50, 20, 20);
        robot_movel(fd, move_cmd);
    }
}
