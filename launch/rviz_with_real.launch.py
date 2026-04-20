from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 替换为你的机器人描述文件路径
    # 通过xacro获取URDF机器人描述
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),  # 查找xacro可执行文件
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('lr_pro_description'),  # 查找包路径
                 'xacro', 'robot.xacro']  # xacro文件路径
            ),
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('lr_pro_description'), 'rviz', 'robot.rviz']
    )

    return LaunchDescription([
        # 启动 robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        # 启动 Joint State Publisher 节点
        Node(
            package="fastdds_ros2_bridge",
            executable="tf_robot_date_real",
            output="screen",
        ),
        # 启动 rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
