#include <iostream>
#include "robot_control/robot_control.hpp" 

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 创建并运行 RobotControl 节点
    rclcpp::spin(std::make_shared<RobotControl>());

    rclcpp::shutdown();
    
    return 0;
}
