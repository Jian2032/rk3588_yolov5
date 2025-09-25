#include "keyboard_reader.hpp" // 引入键盘读取的头文件
#include "arm_control.hpp"     // 引入机械臂控制类的头文件

using namespace std;

int main(int argc, char *argv[])
{
    // 初始化 ROS2 通信环境
    rclcpp::init(argc, argv);

    // 创建一个 ArmControl 节点对象
    auto node = std::make_shared<ArmControl>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
