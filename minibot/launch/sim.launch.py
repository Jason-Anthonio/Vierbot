import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
##
robot_description_xacro = os.path.join(
    get_package_share_directory('minibot'),
    'description',
    'robot.urdf.xacro'
)

# fallback static transform in case TF from robot_state_publisher / controllers is not available yet
static_lidar_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_lidar_to_base',
    output='screen',
    arguments=['0','0','0','0','0','0','base_link','lidar_frame']
)
## static odom->base_link to provide an odom frame when controllers aren't up yet
static_odom_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_odom_to_base',
    output='screen',
    arguments=['0','0','0','0','0','0','odom','base_link']
)
##
# Image Transport Republishers
# terminal command example: ros2 run image_transport republish raw compressed --ros-args -r in:=/camera/image_raw -r out/compressed:=/camera/image_raw/compressed
def image_transport_republisher(transport, camera_topics):
    base_topic = camera_topics.split('/')[-1]
    
    return Node(
        package='image_transport',
        executable='republish',
        name=f'image_transport_republish_{transport}_{base_topic}',
        arguments=['raw', transport],
        remappings=[
            ('in', f'/camera/{camera_topics}'),
            (f'out/{transport}', f'/camera/{camera_topics}/{transport}'),
        ],
    )

def generate_launch_description():

    package_name= 'minibot'
    package_dir= get_package_share_directory(package_name) 

    # Add launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ros2_control_gz_sim = LaunchConfiguration('use_ros2_control_gz_sim')

    # Declare launch arguments
    declare_use_sim_time= DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='If true, use simulated clock'
    )
    
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='If true, use ros2_control (set false to use Gazebo DiffDrive plugin)'
    )

    declare_use_ros2_control_gz_sim = DeclareLaunchArgument(
        'use_ros2_control_gz_sim',
        default_value='true',
        description='If true, use ros2_control in gz_sim'
    )

    # Declare the path to files
    robot_description_xacro_file = os.path.join(
        package_dir,
        'description',
        'robot.urdf.xacro'
    )

    world_file_path = os.path.join(
        package_dir, 
        'worlds', 
        'playground.sdf'
    )

    rviz_config_file = os.path.join(
        package_dir, 
        'config', 
        'sim_config.rviz'
    )

    robot_controllers_file = os.path.join(
        package_dir, 
        'config', 
        'controller_gz_sim.yaml'
    )

    gazebo_params_file = os.path.join(
        package_dir, 
        'config', 
        'gz_params.yaml'
    )
    
    twist_mux_params_file = os.path.join(
        package_dir, 
        'config', 
        'twist_mux.yaml'
    )

    # robot_state_publisher setup    
    robot_description_config = Command ([
        'xacro ', 
        robot_description_xacro_file, 
        ' use_ros2_control:=', 
        use_ros2_control,
        ' use_ros2_control_gz_sim:=',
        use_ros2_control_gz_sim,        
        ])
    
    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str), 
        'use_sim_time': use_sim_time,        
    }

    # robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # gz_bridge node
    # format: '<topic_name>@/[/]<ros2_topic_type_name>@/[/]<gazebo_topic_type_name>',
    # ros2 topic type <topic_name> to check topi type_name in ros2
    # gz topic -t <topic_name> -i to check topic type_name in gazebo
    node_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            
            # General
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Gazebo_Control
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Do NOT bridge /tf when using robot_state_publisher to avoid duplicate TFs

            # Lidar 
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # Camera
            # '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # RGBD Camera
            '/camera/depth/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_raw/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image_raw/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )
    
    # Publish odom->base_link TF from /odom messages when using Gazebo plugin
    # (Gazebo's internal TF isn't reliably bridged by parameter_bridge)
    node_odom_tf_publisher = Node(
        package='minibot',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_ros2_control)
    )

    # Image Transport Republishers Node
    camera = 'image_raw'
    depth_camera = 'depth/image_raw'
    image_transports = ['compressed','compressedDepth', 'theora', 'zstd' ]  
    node_image_republishers = [image_transport_republisher(transport, depth_camera) 
                          for transport in image_transports]

    # gz launch world
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )]), 
                launch_arguments=
                {'gz_args': f'-r -v 4 {world_file_path}',
                 'extra_gazebo_args': '--ros-args --params-file' + gazebo_params_file}.items()
    )

    # gz spawn robot entity 
    node_gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', 
                '-name', 'minibot',
                '-allow_renaming', 'true',
                '-z', '0.1'],
    )

    # rviz2 node
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='both'
    )

    # controller spawn 
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=IfCondition(use_ros2_control)
    )
    
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", 
                   "--param-file", 
                   robot_controllers_file
        ],
        condition=IfCondition(use_ros2_control)
    )

    # twist_mux -> diff_drive_controller/cmd_vel when using ros2_control
    node_twist_mux_ros2_control = Node(
        package="twist_mux",
        executable="twist_mux",
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_params_file,
            {'use_stamped': True},
        ],
        remappings=[
            ('cmd_vel_out', 
             'diff_drive_controller/cmd_vel'),
        ],
        condition=IfCondition(use_ros2_control)
    )

    # twist_mux -> /cmd_vel when using Gazebo DiffDrive system plugin
    node_twist_mux_gz = Node(
        package="twist_mux",
        executable="twist_mux",
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_params_file,
            {'use_stamped': True},
        ],
        remappings=[
            ('cmd_vel_out', 
             '/cmd_vel'),
        ],
        condition=UnlessCondition(use_ros2_control)
    )

    node_twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', 'cmd_vel_smoothed'),
            ('cmd_vel_out', 'nav_vel'),
        ],
    )

    register_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action= node_gz_spawn_entity,
            on_start= [joint_state_broadcaster_spawner],
        )
    )

    register_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action= joint_state_broadcaster_spawner,
            on_start= [diff_drive_controller_spawner],
        )
    )

    group_spawn_gz= GroupAction(
        [gazebo, 
         node_gz_spawn_entity,
        ]
    )


    # Create the launch description and populate
    ld = LaunchDescription()
    # Static TFs for early TF availability - DISABLED: robot_state_publisher provides correct transforms from URDF
    # ld.add_action(static_lidar_tf) ## Disabled: URDF defines base_link->chasis->lidar_frame correctly
    # Temporary static odom TF (remove after verifying dynamic TF works)
    # ld.add_action(static_odom_tf) ## Disabled: now using odom_to_tf.py for dynamic TF
    # Add the nodes to the launch description
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_ros2_control)
    ld.add_action(declare_use_ros2_control_gz_sim)

    ld.add_action(register_joint_state_broadcaster_spawner)
    ld.add_action(register_diff_drive_controller_spawner)

    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_twist_mux_ros2_control)
    ld.add_action(node_twist_mux_gz)
    ld.add_action(node_twist_stamper)
    # Delay the Gazebo bridge and robot spawn slightly so robot_state_publisher
    # and TFs have time to initialize before sensor topics start publishing.
    ld.add_action(TimerAction(period=2.0, actions=[node_gz_bridge, group_spawn_gz]))
    ld.add_action(node_odom_tf_publisher)  # Publish dynamic odom->base_link TF from /odom
    for node_republisher in node_image_republishers:
        ld.add_action(node_republisher)
    # group_spawn_gz is now started by the TimerAction above
    ld.add_action(node_rviz2)

    # Generate the launch description  
    return ld

    