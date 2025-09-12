#include "robot_control/robot_control.hpp"


RobotControl::RobotControl() : Node("robot_control_node")
{
    points_.at(0) = {1, 625, 375, 350};
    points_.at(1) = {2, 815, 385, 345};
    points_.at(2) = {3, 490, 335, 290};
    points_.at(3) = {4, 910, 290, 280};

    sub_target_ = this->create_subscription<robot_msgs::msg::TargetArray>(
        "/target_info", 10,
        std ::bind(&RobotControl::targetCallback, this, std::placeholders::_1));
    
    pub_arm_ctl_ = this->create_publisher<robot_msgs::msg::ArmControl>("arm_control", 10);
    
    // 创建定时器 (10ms 周期)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RobotControl::controlLoop, this));
}

RobotControl::~RobotControl()
{
    // 发布清零消息
    robot_msgs::msg::ArmControl msg{};

    pub_arm_ctl_->publish(msg);

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

bool RobotControl::calculateMovement(const PointInfo &point, const TargetInfo &target)
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
        msg.direction_y = -1;
    }
    else{
        msg.direction_y = 1;
    }

    if(point.v > target.center_v){
        msg.direction_z = 1;
    }
    else{
        msg.direction_z = -1;
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

    if(target.center_u == 0 || target.center_v == 0 || target.distance == 0 
        || target.center_u > 1280 || target.center_v > 720 || target.distance >= 1000
        || target.center_u < 0 || target.center_v < 0 || target.distance < 0)
    {
        msg.direction_y = 0;
        msg.direction_z = 0;
        msg.direction_x = 0;
    }

    if(abs(point.u - target.center_u) <= 5 && abs(point.v - target.center_v) <= 5 && abs(point.distance - target.distance) <= 5)
    {
        msg.direction_y = 0;
        msg.direction_z = 0;
        msg.direction_x = 0;

        pub_arm_ctl_->publish(msg);

        return true;
    }

    pub_arm_ctl_->publish(msg);

    return false;
}
void RobotControl::controlLoop()
{
    static auto last_time = std::chrono::steady_clock::now();

    if (calculateMovement(points_[index],target_.targets[index])){
        index++;
        last_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Waiting 10 seconds...");
    }

    if (index >= 3) index = 3;

    // 非阻塞等待 10 秒
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_time).count() < 10) {
        return; // 等待中，直接返回
    }
}