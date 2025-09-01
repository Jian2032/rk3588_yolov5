#include "control.h"

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
