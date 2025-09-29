# 挤奶机器人

## 挤奶机器人主要包括视觉识别、机械臂控制、继电器控制等多个部分

平台：rk3588 16+256
环境：ubuntu22 ros2-humble

一键启动：

```bash
colcon build --symlink-install # 不添加后面的编译参数无法获取yolov5模型
ros2 launch robot_control robot_control.launch.py 
```

## realsense-ros

### realsense-ros:D435i功能包

### 环境配置

参考:[Installation on Ubuntu](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu)

安装 librealsense2
```bash
sudo apt install ros-<ROS_DISTRO>-librealsense2*

sudo apt install ros-humble-librealsense2*
```

获取功能包 realsense-ros
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/

git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ~/ros2_ws
```

安装依赖
```bash 
sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
```

编译
```bash 
colcon build
```

加载到环境
```bash 
ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: kilted, jazzy, iron, humble, foxy
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
. install/local_setup.bash
```

### 运行相机

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```

### D435i固件版本问题

rk3588 ubuntu24 ros2-humble 要使用5.15.0.2版本(测试可用，并不是每一个版本都可以用5.16.0.1就不能使用)

固件下载地址 [RealSens SDK](https://github.com/intelrealsense/librealsense/releases?utm_source=chatgpt.com)

```bash
rs-fw-update -f Signed_Image_UVC_5_15_0_2.bin -s 239122071767

-f → 固件文件路径
-s → 相机序列号（你的 D435i 是 239122071767）
```

出现权限问题，直接复制搜索，可以解决

## rknn_yolov5_demo

### rknn_yolov5_demo:基于ros2调用rk3588 npu进行yolov5识别，根据开源[rknn-cpp-Multithreading](https://github.com/leafqycc/rknn-cpp-Multithreading)

### 运行yolov5

```bash
ros2 run rknn_yolov5_demo rknn_yolov5_node
```

## robot_control

### 项目决策控制，整合所有话题消息，发布控制指令

### 运行

```bash
ros2 run robot_control robot_control_node
```

## robot_msgs

### robot_msgs:自定义消息包，保存所有自定义消息类型

### 消息类型

ArmControl.msg 机械臂控制消息
ArmInfo.msg 机械臂笛卡尔、关节坐标信息
TargetArray.msg TargetInfo.msg 识别目标信息

## robotic_arm_control

### robotic_arm_control:接受机械臂控制指令，控制机械臂，添加机械臂方向运动控制指令

### 运行

```bash
ros2 run robotic_arm_control robotic_arm_control_node
```

## robot_usart

### robot_usart:与STM32通信功能包，发送继电器控制指令，接受避障传感器信息

### 运行

```bash
ros2 run robot_usart usart_node
```