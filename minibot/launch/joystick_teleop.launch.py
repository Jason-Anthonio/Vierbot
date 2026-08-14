import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('minibot')
    joy_params = os.path.join(pkg_dir, 'config', 'joystick_params.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])
