from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([
            Node(
                package='fastdds_ros2_bridge',  # 包名（可执行文件所在的ROS2包）
                executable='fastdds_ros2_bridge_joint_node',  # 可执行文件名
                name='joint_bridge_node',  # 节点名称
                output='screen',  # 将日志输出到屏幕
                parameters=[
                    {'debug_output': False,
                     'robot_controllers' : None
                     }
                ]
            )
        ])