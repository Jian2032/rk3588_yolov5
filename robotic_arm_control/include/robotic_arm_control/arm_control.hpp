#ifndef ARM_CONTROL_HPP_
#define ARM_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>
#include <signal.h>
#include <memory>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include "cpp_interface/nrc_interface.h"
#include "cpp_interface/nrc_queue_operate.h"
#include "cpp_interface/nrc_job_operate.h"
#include "robot_msgs/msg/arm_control.hpp"
#include "robot_msgs/msg/arm_info.hpp"
#include "keyboard_reader.hpp"

typedef struct _ArmControlData
{
    int8_t mode;
    int8_t work_phase;
    double position_x;
    double position_y;
    double position_z;
    double position_u;
    int direction_x;
    int direction_y;
    int direction_z;
    double direction_u;
} ArmControlData;

class ArmControl : public rclcpp::Node, public KeyboardReader
{
public:
    ArmControl();
    ~ArmControl();

    void arm_control_loop();

private:
    void power_on(int fd);
    void wait_for_running_over(int fd, int timeout_ms);
    void arm_control_jog(ArmControlData data);
    void loadpos(MoveCmd &m, std::vector<double> pos, int coord, double velocity, double acc, double dcc);

    void ArmControlCallback(const robot_msgs::msg::ArmControl::SharedPtr msg);

    rclcpp::Publisher<robot_msgs::msg::ArmInfo>::SharedPtr pub_arm_info_;
    rclcpp::Subscription<robot_msgs::msg::ArmControl>::SharedPtr sub_control_;

    SOCKETFD fd;
    Result res;
    ArmControlData arm_data;
    int cnt = 0;
    int key;
    std::vector<double> pos_tcp, pos_joint;
    MoveCmd move_cmd;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif