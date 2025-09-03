#include "robot_control/robot_control.hpp"


RobotControl::RobotControl() : Node("robot_control_node")
{
    points_.at(0) = {1, 0, 0, 0};
    points_.at(1) = {2, 0, 0, 0};
    points_.at(2) = {3, 0, 0, 0};
    points_.at(3) = {4, 620, 345, 390};

    sub_target_ = this->create_subscription<robot_msgs::msg::TargetArray>(
        "/target_info", 10,
        std ::bind(&RobotControl::targetCallback, this, std::placeholders::_1));
    
    pub_arm_ = this->create_publisher<robot_msgs::msg::ArmControl>("arm_control", 10);
    
    // 创建定时器 (10ms 周期)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RobotControl::controlLoop, this));
}

void RobotControl::targetCallback(const robot_msgs::msg::TargetArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received TargetArray, total_count=%d", msg->total_count);
    target_.total_count = msg->total_count;

    // 遍历 targets 数组
    for (size_t i = 0; i < msg->targets.size(); i++)
    {
        auto &t = msg->targets[i];

        target_.targets[i].index = t.index;
        target_.targets[i].center_u = t.center_u;
        target_.targets[i].center_v = t.center_v;
        target_.targets[i].distance = t.distance;
        // RCLCPP_INFO(this->get_logger(),
        //             "Target %d: center=(%d,%d), distance=%d",
        //             target_.targets[i].index, target_.targets[i].center_u, target_.targets[i].center_v, target_.targets[i].distance);
    }
}

void RobotControl::calculateMovement(const PointInfo &point, const TargetInfo &target)
{
    robot_msgs::msg::ArmControl msg;

    msg.mode = 1;
    RCLCPP_INFO(this->get_logger(),"////////////////////////////////////////////" );
    RCLCPP_INFO(this->get_logger(),
        "PointInfo  -> index: %d, u: %d, v: %d, distance: %d",
        point.index, point.u, point.v, point.distance);
    RCLCPP_INFO(this->get_logger(),
        "TargetInfo -> index: %d, u: %d, v: %d, distance: %d",
        target.index, target.center_u, target.center_v, target.distance);

    if(point.u > target.center_u){
        msg.direction_y = 1;
    }
    else{
        msg.direction_y = -1;
    }

    if(point.v > target.center_v){
        msg.direction_z = -1;
    }
    else{
        msg.direction_z = 1;
    }

    if(point.distance > target.distance){
        msg.direction_x = 1;
    }
    else{
        msg.direction_x = -1;
    }

    if(abs(point.u - target.center_u) <= 5)
        msg.direction_y = 0;
    if(abs(point.v - target.center_v) <= 5)
        msg.direction_z = 0;
    if(abs(point.distance - target.distance) <= 5)
        msg.direction_x = 0;

    if(target.center_u == 0 || target.center_v == 0 || target.distance == 0)
    {
        msg.direction_y = 0;
        msg.direction_z = 0;
        msg.direction_x = 0;
    }

    pub_arm_->publish(msg);
}
void RobotControl::controlLoop()
{
    calculateMovement(points_[3],target_.targets[3]);
}