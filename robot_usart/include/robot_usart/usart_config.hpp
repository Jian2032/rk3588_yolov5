#ifndef ROBOT_USART__USART_CONFIG_HPP_
#define ROBOT_USART__USART_CONFIG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
//#include <nlohmann/json.hpp>

#include "crc.h"
#include "fontColor.h"

//using json = nlohmann::json;

#define RAD_TO_ANGLE 57.2957795f
#define RX_LENGTH   10
#define TX_LENGTH   10
#define UART_XMIT_SIZE 4096

typedef struct SensorData {
  float chassix_x_linear_velocity;
  float chassis_y_linear_velocity;
  float chassis_x_accelerate;
  float chassis_y_accelerate;
  float chassis_yaw;
  uint32_t time_stamp_10us;
} SensorData_Define;

class usartConfig : public rclcpp::Node
{
public:
  usartConfig();
  ~usartConfig();

  void recData_Init(void);
  void Usart_Config(void);
  void controlCmdSendCallback();              
  void controlCMdSend(void);
  int ReadUsart();
  void RecvThread();
  void UsartClose();
  void UsartRestart();
  void run();
  void displayUsartFreq(void);

private:
  void recData_Decode(void);
  void PubSensor_DataSendCallback();          
  void forwardSignalCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  SensorData_Define recSensor;

  using serial_port = boost::asio::serial_port;
  using io_context = boost::asio::io_context;

  std::unique_ptr<io_context> ios_;
  std::unique_ptr<serial_port> serial_port_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dangerPub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr statusPub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr aclOdomDataPub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr SensorDataPub;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_2;

  rclcpp::TimerBase::SharedPtr cmdTimer;
  rclcpp::TimerBase::SharedPtr sensorTimer;

  std::thread recv_thread_;
  std::thread io_thread_;
  std::mutex thread_locker;
  std::atomic<bool> on_running{false};
  std::atomic<bool> io_running{false};

  uint8_t usartTxBuffer[TX_LENGTH];
  CRC usart_check;

  bool use_global = false;
  uint32_t freq = 0;
  uint8_t freqFlag = 0;

  struct RecvStatus {
    int recv_len = 0;
    bool ON_RECV_HEADER = true;
    bool ON_RECV_TYPE = true;
    bool ON_RECV_DATA = true;
    bool WRONG_TICK = false;
    unsigned char buff[UART_XMIT_SIZE];
    void reset() {
      recv_len = 0;
      ON_RECV_HEADER = true;
      ON_RECV_TYPE = true;
      ON_RECV_DATA = true;
      WRONG_TICK = false;
    }
    void set_wrong_tick() {
      WRONG_TICK = true;
      ON_RECV_HEADER = true;
      ON_RECV_TYPE = true;
      ON_RECV_DATA = true;
      recv_len = 0;
    }
  } recv_buff;

  static const int frame_header = 0x05;
  uint8_t forwardSignalData[10] = {0};
  //json param;
  std::string usb_device_ = "/dev/ttyS1";
};

#endif  // ROBOT_USART__USART_CONFIG_HPP_
