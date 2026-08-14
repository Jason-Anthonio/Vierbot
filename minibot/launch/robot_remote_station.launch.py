import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'minibot'
    pkg_dir = get_package_share_directory(pkg_name)
    rviz_config_file = os.path.join(pkg_dir, 'config', 'minibot_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        rviz_node
    ])
