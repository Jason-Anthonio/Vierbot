import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import sys
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import TimerAction

def generate_launch_description():

    package_name= 'minibot'
    package_dir= get_package_share_directory(package_name) 

    use_sim_time = LaunchConfiguration('use_sim_time')
    map = LaunchConfiguration('map')
    use_slam_option = LaunchConfiguration('use_slam_option')
    use_keyboard = LaunchConfiguration('use_keyboard')
    
    declare_use_sim_time= DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='If true, use simulated clock'
    )  

    declare_map= DeclareLaunchArgument(
        'map',
        default_value='./src/minibot/maps/sample_map.yaml',
        description='If true, use simulated clock'
    )

    declare_use_slam_option = DeclareLaunchArgument(
        'use_slam_option',
        default_value='online_async_slam',
        description='Choose SLAM option: amcl, mapper_params_localization, or online_async_slam'
    )  

    declare_use_keyboard = DeclareLaunchArgument(
        'use_keyboard',
        default_value='true',
        description='If true, start keyboard teleop (teleop_wasd). If false, start joystick teleop.'
    )

    # Declare the path to files
    joy_params_file = os.path.join(
        get_package_share_directory(package_name), 
        'config', 
        'joystick_params.yaml' 
    )

    mapper_params_online_async_file = os.path.join(
        package_dir, 
        'config', 
        'mapper_params_online_async.yaml'
    )

    mapper_params_localization_file = os.path.join(
        package_dir, 
        'config', 
        'mapper_params_localization.yaml'
    )

    nav2_params_file = os.path.join(
        package_dir, 
        'config', 
        'nav2_params.yaml'
    )

    # Keyboard teleop (default): publishes TwistStamped to joy_vel for twist_mux
    teleop_wasd = Node(
        package='minibot',
        executable='teleop_wasd.py',
        name='teleop_wasd',
        parameters=[{
            'speed': 0.2,
            'turn': 0.5,
            'repeat_rate': 10.0,
            'topic': 'cmd_vel'
        }],
        condition=IfCondition(PythonExpression(["'", use_keyboard, "' == 'true'"]))
    )

    # Joystick teleop (optional): enabled when use_keyboard is false
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params_file],
        condition=IfCondition(PythonExpression(["'", use_keyboard, "' == 'false'"]))
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params_file],
        remappings=[('/cmd_vel','joy_vel')],
        condition=IfCondition(PythonExpression(["'", use_keyboard, "' == 'false'"]))
    )
        
    # online_async_slam launch 
    online_async_slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                )]), 
                launch_arguments={
                    'slam_params_file': mapper_params_online_async_file,
                    'use_sim_time': use_sim_time
                }.items(),
                condition=IfCondition(PythonExpression(["'", use_slam_option, "' == 'online_async_slam'"]))         
    )

    # nav2_nmapper_params_localization launch 
    mapper_params_localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'localization_launch.py'
                )]), 
                launch_arguments={
                    'slam_params_file': mapper_params_localization_file,
                    'use_sim_time': use_sim_time
                }.items(),
                condition=IfCondition(PythonExpression(["'", use_slam_option, "' == 'mapper_params_localization'"]))         
    )

    # amcl launch 
    amcl = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'localization_launch.py'
                )]), 
                launch_arguments={
                    'map': map,
                    'use_sim_time': use_sim_time
                }.items(),
                condition=IfCondition(PythonExpression(["'", use_slam_option, "' == 'amcl'"]))           
    )

    # nav2_navigation launch 
    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                )]), 
                launch_arguments={
                    'params_file': nav2_params_file,
                    'use_sim_time': use_sim_time
                }.items()            
    )

    

     # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_use_slam_option)
    ld.add_action(declare_use_keyboard)

    # Add the nodes to the launch description
    # ld.add_action(teleop_wasd)
    # ld.add_action(joy_node)
    # ld.add_action(teleop_node)

    # Add SLAM options
    # Delay starting the SLAM node a little so TF is populated
    ld.add_action(TimerAction(period=2.0, actions=[online_async_slam]))
    # Run the helper script directly via the Python interpreter so it doesn't
    # need to be installed as an entrypoint. The script exits with code 0 when
    # the TF becomes available, or non-zero on timeout.
    wait_script = os.path.join(package_dir, 'scripts', 'wait_for_map_tf.py')
    wait_proc = ExecuteProcess(
        cmd=[sys.executable, wait_script, '--ros-args', '-p', 'timeout:=30.0'],
        output='screen'
    )

    # Start navigation when the wait process exits
    ld.add_action(wait_proc)
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(target_action=wait_proc, on_exit=[navigation])
    ))
    ld.add_action(mapper_params_localization)
    ld.add_action(amcl)

    # Add navigation
    ld.add_action(navigation)

    # Generate the launch description and 
    return ld

    