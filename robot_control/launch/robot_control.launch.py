from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # RealSense 相机 launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={}.items()
    )

    # YOLOv5 节点
    yolov5_node = Node(
        package='rknn_yolov5_demo',
        executable='rknn_yolov5_node',
        name='rknn_yolov5',
        output='screen',
        prefix=['xterm', '-e']   # 单独终端
    )

    # 机械臂节点
    arm_node = Node(
        package='robotic_arm_control',
        executable='robotic_arm_control_node',
        name='robotic_arm_control',
        output='screen',
        prefix=['xterm', '-e']
    )

    # 串口节点
    usart_node = Node(
        package='robot_usart',
        executable='usart_node',
        name='usart',
        output='screen',
        prefix=['xterm', '-e']
    )

    # 延时 5 秒启动 robot_control 节点
    robot_control_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_control',
                executable='robot_control_node',
                name='robot_control',
                output='screen',
                prefix=['xterm', '-e']
            )
        ]
    )

    return LaunchDescription([
        realsense_launch,
        yolov5_node,
        arm_node,
        usart_node,
        robot_control_node
    ])
