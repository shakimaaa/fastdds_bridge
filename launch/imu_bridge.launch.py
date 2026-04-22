from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fastdds_ros2_bridge',  # 包名（可执行文件所在的ROS2包）
            executable='fastdds_ros2_bridge_imu_node',  # 可执行文件名
            name='imu_forwarder_node',  # 节点名称
            output='screen',  # 将日志输出到屏幕
            parameters=[
                {'debug_output': False},
                {'enable_forwarding': False},
                {'forward_hz': 100.0},
            ]
        )
    ])