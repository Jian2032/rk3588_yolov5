#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>
#include <signal.h>
#include <memory>
#include <iostream>
#include <unistd.h>
#include "cpp_interface/nrc_interface.h"
#include "cpp_interface/nrc_queue_operate.h"
#include "cpp_interface/nrc_job_operate.h"

#include "keyboard_reader.h"
#include "robot_msgs/msg/arm_control.hpp"

using namespace std;
typedef struct _ArmControlData
{
    int mode;
    int position_x;
    int position_y;
    int position_z;
    double position_u;
    int direction_x;
    int direction_y;
    int direction_z;
    double direction_u;
}ArmControlData;

SOCKETFD fd;
KeyboardReader reader;
ArmControlData arm_data;
int cnt;
Result res;

void key_control();
void arm_control_callback(const robot_msgs::msg::ArmControl::SharedPtr msg);
void arm_control(ArmControlData data);
void power_on(int fd);
void wait_for_running_over(int fd);
std::vector<double> pos(7);

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robotic_arm_control_node");
    RCLCPP_INFO(node->get_logger(), "Robotic Arm Control Node has started.");
    
    auto sub = node->create_subscription<robot_msgs::msg::ArmControl>(
        "arm_control", 10, arm_control_callback);

    fd = connect_robot("192.168.2.14", "6001");
    if (fd <= 0)
    {
        std::cout << "连接失败" << std::endl;
        return 0;
    }
    std::cout << "连接成功: "<< fd << std::endl;

    set_current_mode(fd, 0); // 设置示教模式，只有示教模式下的运动需要上电
    set_speed(fd, 10); // 设置示教模式下的全局速度
    power_on(fd); // 调用封装的上电函数


    res = get_current_position(fd, 1, pos); // coord=1:直角坐标
    if (res == SUCCESS) {
        for (double p : pos) {
            std::cout << p << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "获取位置失败: " << res << std::endl;
    }

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        cnt++;
        // 键盘控制机械臂
        key_control();
        // 话题控制机械臂
        arm_control(arm_data);

        rate.sleep();
    }

    set_servo_poweroff(fd);
    disconnect_robot(fd);

    rclcpp::shutdown();
    return 0;
}


void key_control()
{

    int key = reader.readKey();

    if (key == -1)
    {
        robot_stop_jogging(fd,1);
        robot_stop_jogging(fd,2);
        robot_stop_jogging(fd,3);
    }
    switch (key)
    {
        case 'a': robot_start_jogging(fd, 1, true);  break; // X+ 
        case 'd': robot_start_jogging(fd, 1, false); break; // X- 
        case 's': robot_start_jogging(fd, 2, true);  break; // Y+ 
        case 'w': robot_start_jogging(fd, 2, false); break; // Y- 
        case 'q': robot_start_jogging(fd, 3, true);  break; // Z+ 
        case 'e': robot_start_jogging(fd, 3, false); break; // Z- 
        case 'y':
            robot_start_jogging(fd, 1, true);
            robot_start_jogging(fd, 2, true);
            break;
        default:  
            robot_stop_jogging(fd,1);
            robot_stop_jogging(fd,2);
            robot_stop_jogging(fd,3);
            break;
    }

}

void arm_control_callback(const robot_msgs::msg::ArmControl::SharedPtr msg)
{
    arm_data.mode        = msg->mode;
    arm_data.position_x  = msg->position_x;
    arm_data.position_y  = msg->position_y;
    arm_data.position_z  = msg->position_z;
    arm_data.position_u  = msg->position_u;
    arm_data.direction_x = msg->direction_x;
    arm_data.direction_y = msg->direction_y;
    arm_data.direction_z = msg->direction_z;
    arm_data.direction_u = msg->direction_u;
}

void arm_control(ArmControlData data)
{
    res = get_current_position(fd, 1, pos); // coord=1:直角坐标
    if (res == SUCCESS) {
        for (double p : pos) {
            std::cout << p << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "获取位置失败: " << res << std::endl;
    }

    if (data.mode == -1) {
        robot_stop_jogging(fd,1);
        robot_stop_jogging(fd,2);
        robot_stop_jogging(fd,3);
    } 
    else if (data.mode == 1) {
        // X轴
        if (data.direction_x != 0 && cnt % 3 == 1) {
            robot_start_jogging(fd, 1, data.direction_x > 0);
        }
        else{
            robot_stop_jogging(fd,1);
        }
        // Y轴
        if (data.direction_y != 0 && cnt % 3 == 2) {
            robot_start_jogging(fd, 2, data.direction_y < 0);
        }
        else{
            robot_stop_jogging(fd,2);
        }
        // Z轴
        if (data.direction_z != 0 && cnt % 3 == 0) {
            robot_start_jogging(fd, 3, data.direction_z < 0);
        }
        else{
            robot_stop_jogging(fd,3);
        }
        // 如果三个方向都是 0，就停止
        if (data.direction_x == 0 && data.direction_y == 0 && data.direction_z == 0) {
            robot_stop_jogging(fd,1);
            robot_stop_jogging(fd,2);
            robot_stop_jogging(fd,3);
        }

    }
}


/*
 * 封装上电函数
 */
void power_on(int fd)
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
void wait_for_running_over(int fd)
{
    // 等待运动完成
    int running_state = 0;
    get_robot_running_state(fd, running_state); // 查询机器人是否在运动 2-正在运动
    while (running_state == 2)
    {
        usleep(500 * 1000); // 500ms = 500*1000微秒
        get_robot_running_state(fd, running_state);                  // 再次查询
    }
}
