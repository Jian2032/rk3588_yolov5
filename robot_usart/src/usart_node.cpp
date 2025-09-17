#include "rclcpp/rclcpp.hpp"
#include "robot_usart/usart_config.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<usartConfig>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
