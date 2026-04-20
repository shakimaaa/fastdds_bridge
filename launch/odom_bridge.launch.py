from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='lr_pro', description='机器人类型，DDS 话题名将变为 <robot>_odom_sim'),
        Node(
            package='fastdds_ros2_bridge',
            executable='fastdds_ros2_bridge_odom_node',
            name='odom_forwarder_node',
            output='screen',
            parameters=[{'debug_output': False}, {'ros_topic': '/Odometry'}],
            arguments=['robot', LaunchConfiguration('robot')],
        ),
    ])
