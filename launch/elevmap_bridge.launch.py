from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    debug_output = LaunchConfiguration('debug_output', default='false')
    robot = LaunchConfiguration('robot', default='lr_pro')
    return LaunchDescription([
        DeclareLaunchArgument('debug_output', default_value='false',
                              description='Set true to log each ElevMap forward to DDS.'),
        DeclareLaunchArgument('robot', default_value='lr_pro', description='Robot type (DDS topic: <robot>_elevmap_sim).'),
        Node(
            package='fastdds_ros2_bridge',
            executable='fastdds_ros2_bridge_elevation_node',
            name='elevmap_forwarder_node',
            output='screen',
            parameters=[{'debug_output': debug_output}],
            arguments=['robot', robot],
        ),
    ])
