#include "rclcpp/rclcpp.hpp"
#include "robot_usart/usart_config.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include "std_msgs/msg/u_int8_multi_array.hpp"

using namespace std::chrono_literals;

// ----------------- 构造函数 -----------------
// 初始化节点、串口和相关数据
usartConfig::usartConfig()
: Node("usart")
{
  RCLCPP_INFO(this->get_logger(), "usartConfig node starting...");

  // 初始化 forwardSignalData 数组，保证发送数据为 0
  memset(forwardSignalData, 0, sizeof(forwardSignalData));

  // 配置串口、订阅、定时器等
  Usart_Config();
}

// ----------------- 析构函数 -----------------
// 关闭串口和线程，安全退出
usartConfig::~usartConfig()
{
  RCLCPP_INFO(this->get_logger(), "节点正在关闭，准备释放资源...");

  // 停止接收线程
  on_running = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
    RCLCPP_INFO(this->get_logger(), "接收线程已结束");
  }

  // 关闭串口
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->cancel();
    serial_port_->close();
    RCLCPP_INFO(this->get_logger(), "串口已关闭");
  }

  // 停止 io_context
  if (ios_) {
    ios_->stop();
    RCLCPP_INFO(this->get_logger(), "IO 服务已停止");
  }

  // 等待 IO 线程退出
  if (io_thread_.joinable()) {
    io_thread_.join();
    RCLCPP_INFO(this->get_logger(), "IO 线程已结束");
  }

  RCLCPP_INFO(this->get_logger(), "节点关闭完成");
}
// ----------------- 控制命令发送回调 -----------------
// 用定时器周期发送 forward 信号到串口
void usartConfig::controlCmdSendCallback()
{
  std::lock_guard<std::mutex> lk(thread_locker);

  // 串口发送数据格式: [帧头0x03, 帧头0xFC, forwardSignalData[0..2], 0,0,0]
  usartTxBuffer[0] = 0x03;
  usartTxBuffer[1] = 0xFC;
  usartTxBuffer[2] = forwardSignalData[0];
  usartTxBuffer[3] = forwardSignalData[1];
  usartTxBuffer[4] = forwardSignalData[2];
  usartTxBuffer[5] = forwardSignalData[3];
  usartTxBuffer[6] = 0;
  usartTxBuffer[7] = 0;

  // 添加 CRC 校验
  usart_check.Append_CRC16_Check_Sum(usartTxBuffer, TX_LENGTH);

  // 发送到串口
  if (serial_port_ && serial_port_->is_open()) {
    boost::asio::write(*serial_port_, boost::asio::buffer(usartTxBuffer, TX_LENGTH));
  }
}

// ----------------- 显示串口频率 -----------------
// 用于调试，显示串口接收频率
void usartConfig::displayUsartFreq()
{
  freqFlag = 1;

  static uint32_t i = 0, time = 0, num = 0;
  static uint64_t flagSucc = 0;

  i++;
  if (i >= 1000)  // 每 1000 次计数，大约 1 秒
  {
    if (freq >= 1000) freq = 0;  // 频率异常时清零

    i = 0;
    if (freq > 0)
    {
      flagSucc += freq;
      num++;
      float avg = static_cast<float>(flagSucc) / num;
      RCLCPP_INFO(this->get_logger(), "Average freq: %.2f", avg);
    }

    RCLCPP_WARN(this->get_logger(), "Current freq: %d", freq);
    freq = 0;
    time++;
    RCLCPP_INFO(this->get_logger(), "Elapsed seconds: %u", time);
  }
}

// ----------------- 读取串口 -----------------
// 循环读取串口数据，根据帧头和类型判断是否有效
int usartConfig::ReadUsart()
{
  if (!serial_port_ || !serial_port_->is_open()) return -4;  // 串口未打开

  try {
    // ----------------- 接收帧头 -----------------
    if (recv_buff.ON_RECV_HEADER) {
      int recv_len = boost::asio::read(*serial_port_, boost::asio::buffer(recv_buff.buff, 1));
      if (recv_len <= 0) return -3;  // 没有数据

      recv_buff.recv_len = recv_len;
      if (recv_buff.buff[0] == frame_header && !recv_buff.WRONG_TICK) {
        recv_buff.ON_RECV_HEADER = false;  // 帧头正确，进入接收类型阶段
      } else {
        recv_buff.set_wrong_tick();  // 帧头错误，重置状态
        return -3;
      }
    }

    // ----------------- 接收数据类型 -----------------
    if (recv_buff.ON_RECV_TYPE) {
      int recv_len = boost::asio::read(*serial_port_, boost::asio::buffer(recv_buff.buff + 1, 1));
      if (recv_len <= 0) return -3;

      recv_buff.recv_len += recv_len;
      if (recv_buff.buff[1] == 0xFA) {
        recv_buff.ON_RECV_TYPE = false;  // 类型正确，进入接收数据阶段
      } else {
        recv_buff.set_wrong_tick();  // 类型错误
        return -3;
      }
    }

    // ----------------- 接收数据 -----------------
    if (recv_buff.ON_RECV_DATA) {
      int expected_recv_len = RX_LENGTH;
      int need = expected_recv_len - recv_buff.recv_len;

      int recv_len = boost::asio::read(*serial_port_, boost::asio::buffer(recv_buff.buff + recv_buff.recv_len, need));
      if (recv_len <= 0) return -3;

      recv_buff.recv_len += recv_len;

      // 校验 CRC
      if (usart_check.Verify_CRC16_Check_Sum(recv_buff.buff, expected_recv_len)) {
        recData_Decode();          // 数据解析
        if (freqFlag == 1) freq++; // 更新接收频率计数
        recv_buff.ON_RECV_DATA = false;
      } else {
        recv_buff.set_wrong_tick();  // CRC 校验失败
        return -1;
      }
    }

    return recv_buff.buff[1];  // 返回接收到的类型
  } catch (...) {
    return -3;  // 异常情况
  }
}

// ----------------- 串口接收线程 -----------------
void usartConfig::RecvThread()
{
  int error_count = 0;  // 记录连续出错次数

  while (on_running) {
    int ret = ReadUsart();

    if (ret <= 0) {
      RCLCPP_WARN(this->get_logger(), "串口读取失败，错误码=%d", ret);
      recv_buff.reset();  // 出错时也清空缓冲
      std::this_thread::sleep_for(10ms);
      error_count++;

      // 连续出错 100 次自动重启串口
      if (error_count >= 100) {
        RCLCPP_ERROR(this->get_logger(), "串口连续出错，正在尝试重启...");
        UsartRestart();
        error_count = 0;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "收到有效数据，消息类型=%d", ret);
      recv_buff.reset();
      error_count = 0;  // 成功就清零
    }
  }

  RCLCPP_INFO(this->get_logger(), "接收线程退出");
}

// ----------------- 初始化接收数据 -----------------
void usartConfig::recData_Init()
{
  recSensor.chassix_x_linear_velocity = 0.0f;
  recSensor.chassis_y_linear_velocity = 0.0f;
  recSensor.chassis_x_accelerate = 0.0f;
  recSensor.chassis_y_accelerate = 0.0f;
  recSensor.chassis_yaw = 0.0f;
  recSensor.time_stamp_10us = 0;
}

// ----------------- 数据解码 -----------------
// 根据串口接收到的缓冲区解析传感器和危险信号
void usartConfig::recData_Decode()
{
    // 帧头校验
    if (recv_buff.buff[0] != 0x05 || recv_buff.buff[1] != 0xFA) {
        RCLCPP_WARN(this->get_logger(), "接收到无效帧头: 0x%02X 0x%02X", recv_buff.buff[0], recv_buff.buff[1]);
        return;
    }

    // 提取三个距离值（单位：mm）
    uint8_t dist1 = recv_buff.buff[2];
    uint8_t dist2 = recv_buff.buff[3];
    uint8_t dist3 = recv_buff.buff[4];

    RCLCPP_INFO(this->get_logger(), "距离值: dist1=%d mm, dist2=%d mm, dist3=%d mm",
                dist1, dist2, dist3);

    // -------------------- danger 发布 --------------------
    auto msg = std_msgs::msg::Int32();

    if (dist1 < 150) {
        if (dist1 < 50) {
            msg.data = 4;  // 严重危险
        } else {
            msg.data = 1;  // 一般危险
        }
    } 
    else if (dist2 < 150) {
        if (dist2 < 50) {
            msg.data = 5;
        } else {
            msg.data = 2;
        }
    } 
    else if (dist3 < 150) {
        if (dist3 < 50) {
            msg.data = 6;
        } else {
            msg.data = 3;
        }
    } 
    else {
        msg.data = 0;  // 安全
    }

    dangerPub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "危险信号已发布: %d", msg.data);

    // -------------------- 红外状态发布 --------------------
    std_msgs::msg::Int32 status_msg;
    int hongwai = recv_buff.buff[5] * 10 + recv_buff.buff[6];
    status_msg.data = hongwai;
    statusPub->publish(status_msg);
    RCLCPP_INFO(this->get_logger(), "红外状态已发布: %d", hongwai);
}

// ----------------- 串口关闭 -----------------
void usartConfig::UsartClose()
{
  std::lock_guard<std::mutex> lk(thread_locker);
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->cancel();
    serial_port_->close();
  }
  on_running = false;
}

// ----------------- 串口重启 -----------------
void usartConfig::UsartRestart()
{
  UsartClose();
  std::this_thread::sleep_for(50ms); // 等待 50ms
  Usart_Config();
}

// ----------------- 串口配置 -----------------
void usartConfig::Usart_Config()
{
  // try {
  //   std::string pkg_dir = ament_index_cpp::get_package_share_directory("robot_usart");
  //   std::string json_path = pkg_dir + "/param.json";
  //   std::ifstream ifs(json_path);
  //   if (ifs) ifs >> param;  // 读取 JSON 配置
  // } catch (...) {
  //   RCLCPP_WARN(this->get_logger(), "Could not read param.json");
  // }

  this->declare_parameter("usart_node.use_global", false);
  use_global = this->get_parameter("usart_node.use_global").as_bool();

  ios_ = std::make_unique<io_context>();
  try {
    serial_port_ = std::make_unique<serial_port>(*ios_, usb_device_);
    serial_port_->set_option(serial_port::baud_rate(921600));
    serial_port_->set_option(serial_port::flow_control(serial_port::flow_control::none));
    serial_port_->set_option(serial_port::parity(serial_port::parity::none));
    serial_port_->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    serial_port_->set_option(serial_port::character_size(8));
  } catch (std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
    return;
  }

  // ----------------- 订阅 forward 信号 -----------------
  sub_2 = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "relay_control", 10,
    std::bind(&usartConfig::forwardSignalCallback, this, std::placeholders::_1));

  // ----------------- 发布话题 -----------------
  dangerPub = this->create_publisher<std_msgs::msg::Int32>("danger_signal", 10);
  statusPub = this->create_publisher<std_msgs::msg::Int32>("hongwai", 10);

  // ----------------- 定时器 -----------------
  cmdTimer = this->create_wall_timer(20ms, std::bind(&usartConfig::controlCmdSendCallback, this));
  sensorTimer = this->create_wall_timer(5ms, std::bind(&usartConfig::PubSensor_DataSendCallback, this));

  // ----------------- 启动线程 -----------------
  io_running = true;
  io_thread_ = std::thread([this]() { ios_->run(); });
  on_running = true;
  recv_thread_ = std::thread(&usartConfig::RecvThread, this);
}

// ----------------- 发布传感器数据 -----------------
void usartConfig::PubSensor_DataSendCallback() {}

// ----------------- forward 信号回调 -----------------
void usartConfig::forwardSignalCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  for (size_t i = 0; i < 10 && i < msg->data.size(); ++i) {
    forwardSignalData[i] = static_cast<uint8_t>(msg->data[i]);
  }
}
