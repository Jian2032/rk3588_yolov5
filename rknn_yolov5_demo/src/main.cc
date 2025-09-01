#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rkYolov5s.hpp"
#include "rknnPool.hpp"
#include <opencv2/opencv.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rknnyolo.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string pkg_path = ament_index_cpp::get_package_share_directory("rknn_yolov5_demo");
    std::string model_path = pkg_path + "/model/RK3588/best-y.rknn";

    RCLCPP_INFO(rclcpp::get_logger("rknn_yolov5_node"), 
        "Using model: %s", model_path.c_str());
    auto node = std::make_shared<RknnYoloNode>(model_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
