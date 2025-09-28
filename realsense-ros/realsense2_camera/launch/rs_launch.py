# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# 许可证信息，使用该文件需要遵循 Apache License 2.0

"""Launch realsense2_camera node."""
# 本文件用于启动 realsense2_camera 节点，支持多参数配置

import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration

# 定义可配置参数列表，每个参数包含名称、默认值和描述
configurable_parameters = [
    {'name': 'camera_name', 'default': 'camera', 'description': '相机唯一名称'},
    {'name': 'camera_namespace', 'default': 'camera', 'description': '相机命名空间'},
    {'name': 'serial_no', 'default': "''", 'description': '根据序列号选择设备'},
    {'name': 'usb_port_id', 'default': "''", 'description': '根据USB端口选择设备'},
    {'name': 'device_type', 'default': "''", 'description': '选择设备类型'},
    {'name': 'config_file', 'default': "''", 'description': 'yaml配置文件路径'},
    {'name': 'json_file_path', 'default': "''", 'description': '高级配置文件路径'},
    {'name': 'initial_reset', 'default': 'false', 'description': '是否初始重置相机'},
    {'name': 'accelerate_gpu_with_glsl', 'default': "false", 'description': '启用GLSL GPU加速'},
    {'name': 'rosbag_filename', 'default': "''", 'description': '使用rosbag文件作为相机输入'},
    {'name': 'rosbag_loop', 'default': 'false', 'description': 'rosbag循环播放'},
    {'name': 'log_level', 'default': 'info', 'description': '日志等级 [DEBUG|INFO|WARN|ERROR|FATAL]'},
    {'name': 'output', 'default': 'screen', 'description': '节点输出方式 [screen|log]'},
    # 以下为各个流和传感器参数
    {'name': 'enable_color', 'default': 'true', 'description': '启用彩色图像流'},
    {'name': 'rgb_camera.color_profile', 'default': '1280,720,30', 'description': '彩色流分辨率配置'},
    {'name': 'rgb_camera.color_format', 'default': 'RGB8', 'description': '彩色图像格式'},
    {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': '彩色自动曝光'},
    {'name': 'enable_depth', 'default': 'true', 'description': '启用深度图像流'},
    {'name': 'enable_infra', 'default': 'false', 'description': '启用红外0流'},
    {'name': 'enable_infra1', 'default': 'false', 'description': '启用红外1流'},
    {'name': 'enable_infra2', 'default': 'false', 'description': '启用红外2流'},
    {'name': 'depth_module.depth_profile', 'default': '0,0,0', 'description': '深度流分辨率配置'},
    {'name': 'depth_module.depth_format', 'default': 'Z16', 'description': '深度图像格式'},
    # IMU 和同步参数
    {'name': 'enable_gyro', 'default': 'false', 'description': '启用陀螺仪流'},
    {'name': 'enable_accel', 'default': 'false', 'description': '启用加速度计流'},
    {'name': 'enable_motion', 'default': 'false', 'description': '启用IMU运动流'},
    # 其他滤波器、对齐和点云参数
    {'name': 'align_depth.enable', 'default': 'true', 'description': '启用深度对齐'},
    {'name': 'colorizer.enable', 'default': 'false', 'description': '启用深度可视化颜色化'},
    {'name': 'pointcloud.enable', 'default': 'false', 'description': '启用点云发布'},
    {'name': 'publish_tf', 'default': 'false', 'description': '是否发布TF'},
]

# 根据参数列表声明launch文件中的可配置参数
def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

# 将LaunchConfiguration对象映射为字典
def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

# YAML文件转字典
def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

# 节点启动函数
def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    # 从全局配置文件获取生命周期节点设置
    lifecycle_param_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'global_settings.yaml'
    )
    lifecycle_params = yaml_to_dict(lifecycle_param_file)
    use_lifecycle_node = lifecycle_params.get("use_lifecycle_node", False)

    _output = LaunchConfiguration('output' + param_name_suffix)
    
    # 根据配置选择普通节点或生命周期节点
    node_action = launch_ros.actions.LifecycleNode if use_lifecycle_node else launch_ros.actions.Node
    log_message = "Launching as LifecycleNode" if use_lifecycle_node else "Launching as Normal ROS Node"

    # Foxy不支持LaunchConfiguration对象作为输出参数，需要获取字符串
    if(os.getenv('ROS_DISTRO') == 'foxy'):
        _output = context.perform_substitution(_output)

    return [
        LogInfo(msg=f"🚀 {log_message}"),  # 打印日志信息
        node_action(
            package='realsense2_camera',  # 节点所在包
            namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),  # 节点命名空间
            name=LaunchConfiguration('camera_name' + param_name_suffix),  # 节点名称
            executable='realsense2_camera_node',  # 可执行文件
            parameters=[params, params_from_file],  # 参数列表
            output=_output,  # 输出方式
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
            emulate_tty=True,
        )
    ]

# 生成launch描述
def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters) + [
            OpaqueFunction(function=launch_setup, kwargs = {'params' : set_configurable_parameters(configurable_parameters)})
        ]
    )
