from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    debug_output = LaunchConfiguration('debug_output', default='false')
    robot = LaunchConfiguration('robot', default='lr_pro')
    ros_topic = LaunchConfiguration('ros_topic', default='/elevation_mapping_node/body_elevation_cloud')
    layer_name = LaunchConfiguration('layer_name', default='inpaint')
    return LaunchDescription([
        DeclareLaunchArgument('debug_output', default_value='false',
                              description='Log each forward to DDS.'),
        DeclareLaunchArgument('robot', default_value='lr_pro',
                              description='DDS topic: <robot>_elevmap_body_cloud_sim'),
        DeclareLaunchArgument('ros_topic', default_value='/elevation_mapping_node/body_elevation_cloud',
                              description='ROS2 PointCloud2 source.'),
        DeclareLaunchArgument('layer_name', default_value='inpaint',
                              description='DDS ElevMapBodyPointCloud.layer_name.'),
        Node(
            package='fastdds_ros2_bridge',
            executable='fastdds_ros2_bridge_elevmap_body_cloud_node',
            name='elevmap_body_cloud_forwarder_node',
            output='screen',
            arguments=['robot', robot],
            parameters=[{
                'debug_output': debug_output,
                'ros_topic': ros_topic,
                'layer_name': layer_name,
            }],
        ),
    ])
