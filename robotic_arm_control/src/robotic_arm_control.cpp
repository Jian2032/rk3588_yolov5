#include "rclcpp/rclcpp.hpp"
#include <vector>

#include "control.h"

using namespace std;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("robotic_arm_control_node");
  RCLCPP_INFO(node->get_logger(), "Robotic Arm Control Node has started.");

  SOCKETFD fd = connect_robot("192.168.2.14", "6001");
  if (fd <= 0)
  {
    std::cout << "连接失败" << std::endl;
    return 0;
  }
  std::cout << "连接成功: "<< fd << std::endl;

  set_current_mode(fd, 0); // 设置示教模式，只有示教模式下的运动需要上电
  set_speed(fd, 50);       // 设置示教模式下的全局速度
  power_on(fd); // 调用封装的上电函数

  std::vector<double> pos(7);

  Result res = get_current_position(fd, 1, pos); // coord=1:直角坐标
  if (res == SUCCESS) {
      for (double p : pos) {
          std::cout << p << " ";
      }
      std::cout << std::endl;
  } else {
      std::cout << "获取位置失败: " << res << std::endl;
  }

  // 构建运动指令，开始运动
  MoveCmd temp_cmd;
  temp_cmd.coord = 1; // 设置关节坐标系
  temp_cmd.targetPosType = PosType::data;
  get_current_position(fd, temp_cmd.coord, temp_cmd.targetPosValue); // 获取当前的关节坐标
  temp_cmd.targetPosValue[1] += 50;                     // 基于当前位置 1轴向正方向移动10°
  temp_cmd.velocity = 50;                               // 指令速度，关节坐标系下速度范围[1,100]
  temp_cmd.acc = 100;                                   // 指令加速度
  temp_cmd.dec = 100;

  set_servo_poweron(fd);
  robot_movel(fd, temp_cmd);
  wait_for_running_over(fd); // 阻塞到运动结束
  set_servo_poweroff(fd);

  res = get_current_position(fd, 1, pos); // coord=1:直角坐标
  if (res == SUCCESS) {
      for (double p : pos) {
          std::cout << p << " ";
      }
      std::cout << std::endl;
  } else {
      std::cout << "获取位置失败: " << res << std::endl;
  }

  // rclcpp::Rate rate(1.0);
  // while (rclcpp::ok())
  // { 
  //   rate.sleep();
  // }
  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}