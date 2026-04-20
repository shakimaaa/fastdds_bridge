from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fastdds_ros2_bridge',  # 包名（可执行文件所在的ROS2包）
            executable='fastdds_ros2_bridge_image_node',  # 可执行文件名
            name='image_bridge_node',  # 节点名称
            output='screen',  # 将日志输出到屏幕
            parameters=[
                {'debug_output': False}  # 设置参数 debug_output
            ]
        )
    ])