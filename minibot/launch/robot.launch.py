import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'minibot'
    pkg_dir = get_package_share_directory(pkg_name)

    # 1. Robot State Publisher
    xacro_file = os.path.join(pkg_dir, 'description', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file, ' use_ros2_control:=true sim_mode:=false'])
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # 2. Controller Manager & ros2_control
    controller_params_file = os.path.join(pkg_dir, 'config', 'controller.yaml')
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params_file],
        output='screen'
    )

    delayed_controller_manager = TimerAction(period=2.0, actions=[controller_manager])

    # 3. Spawners for Broadcaster and Diff Drive Controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    delayed_diff_drive_spawner = TimerAction(period=3.0, actions=[diff_drive_spawner])

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    delayed_joint_broad_spawner = TimerAction(period=3.0, actions=[joint_broad_spawner])

    # 4. RPLidar C1 / SLLidar
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_pkg, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/rplidar', 'frame_id': 'laser_frame'}.items()
    )

    # 5. Twist Mux
    twist_mux_params = os.path.join(pkg_dir, 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        rsp_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        lidar_launch,
        twist_mux_node
    ])
