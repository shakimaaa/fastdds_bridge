#file: combined_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():

    robot_name = LaunchConfiguration('robot', default='lr_pro')
    robot_package_name = [robot_name, '_description']
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(robot_package_name),
            'config',
            'gz_sim_ctrl.yaml',
        ]
    )

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
        Node(
            package='fastdds_ros2_bridge',  # 包名
            executable='fastdds_ros2_bridge_imu_node',  # IMU转换节点可执行文件名
            name='imu_forwarder_node',  # IMU节点名称
            output='screen',  # 控制台输出日志
            parameters=[
                {'debug_output': False}  # 传递给IMU节点的参数
            ],
            arguments=['robot', robot_name]  # 传递机器人类型参数
        ),
        # Node(
        #     package='fastdds_ros2_bridge',  # 包名
        #     executable='fastdds_ros2_bridge_joint_node',  # 关节转换节点可执行文件名
        #     name='joint_bridge_node',  # 关节节点名称
        #     output='screen',  # 控制台输出日志
        #     parameters=[
        #         {'debug_output': False,
        #          "robot_controllers": robot_controllers
        #          }  # 传递给关节节点的参数
        #     ],
        #     arguments=['robot', robot_name]  # 传递机器人类型参数
        # ),
        Node(
            package='fastdds_ros2_bridge',  # 包名
            executable='fastdds_ros2_bridge_image_node',  # 关节转换节点可执行文件名
            name='image_bridge_node',  # 关节节点名称
            output='screen',  # 控制台输出日志
            parameters=[
                {'debug_output': False}  # 传递给关节节点的参数
            ]
        ),
        Node(
            package='fastdds_ros2_bridge',
            executable='fastdds_ros2_bridge_odom_node',
            name='odom_forwarder_node',
            output='screen',
            parameters=[{'debug_output': False}, {'ros_topic': '/Odometry'}],
            arguments=['robot', robot_name],
        ),
        # Node(
        #     package='fastdds_ros2_bridge',
        #     executable='fastdds_ros2_bridge_elevation_node',
        #     name='elevmap_forwarder_node',
        #     output='screen',
        #     parameters=[{'debug_output': False}],
        #     arguments=['robot', robot_name],
        # ),
        # Node(
        #     package='fastdds_ros2_bridge',
        #     executable='tf_robot_date_real',
        #     name='robot_date_real_node',
        #     output='screen',
        #     parameters=[{'debug_output': False}],
        #     arguments=['robot', robot_name],
        # ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        Node(
            package="fastdds_ros2_bridge",
            executable="tf_robot_date_real",
            output="screen",
            parameters=[{'debug_output': False}],
            arguments=['robot', robot_name],
        ),
        Node(
            package='fastdds_ros2_bridge',
            executable='fastdds_ros2_bridge_elevmap_body_cloud_node',
            name='elevmap_body_cloud_forwarder_node',
            output='screen',
            parameters=[{'debug_output': False}],
            arguments=['robot', robot_name],
        )
    ])