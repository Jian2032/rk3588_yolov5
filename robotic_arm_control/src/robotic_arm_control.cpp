#include "keyboard_reader.hpp"
#include "arm_control.hpp"

using namespace std;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControl>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}






