import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'minibot'
    pkg_dir = get_package_share_directory(pkg_name)

    # 1. Robot description for simulation
    xacro_file = os.path.join(pkg_dir, 'description', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file, ' use_ros2_control:=false sim_mode:=true'])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 2. Gazebo launch
    gazebo_world = os.path.join(pkg_dir, 'worlds', 'playground.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': gazebo_world}.items()
    )

    # 3. Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'vierbot', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_entity
    ])
