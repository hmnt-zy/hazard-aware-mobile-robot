from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FileContent, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ----------------------------------------------------
    # 1. Launch Configurations
    # ----------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    urdf_file = LaunchConfiguration('urdf_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_delay = LaunchConfiguration('slam_delay')
    nav2_delay = LaunchConfiguration('nav2_delay')

    description_pkg = FindPackageShare('har_description')
    slam_pkg = FindPackageShare('har_slam')

    default_slam_params = PathJoinSubstitution([
        slam_pkg,
        'config',
        'mapper_params_online_async.yaml'
    ])

    urdf_path = PathJoinSubstitution([
        description_pkg,
        'urdf',
        urdf_file
    ])

    # ----------------------------------------------------
    # 2. Launch Arguments
    # ----------------------------------------------------
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true, hardware clock if false'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='test-world.sdf',
        description='SDF world file name to load into Gazebo (simulation only)'
    )

    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value='BB2.urdf',
        description='URDF file name for robot description'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to SLAM parameters YAML file'
    )

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value='nav2_params.yaml',
        description='Navigation parameter file name in har_navigation/config'
    )

    declare_slam_delay_cmd = DeclareLaunchArgument(
        'slam_delay',
        default_value='15.0',
        description='Seconds to wait before launching SLAM (allows simulator/hardware to initialize)'
    )

    declare_nav2_delay_cmd = DeclareLaunchArgument(
        'nav2_delay',
        default_value='20.0',
        description='Seconds to wait before launching Nav2 (allows SLAM map & TF to initialize)'
    )

    # ----------------------------------------------------
    # 3. Simulation Mode (use_sim_time = true)
    #    Launches Gazebo world, robot spawn, bridge, and robot_description
    # ----------------------------------------------------
    sim_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('har_gazebo'),
                'launch',
                'sim.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_file,
            'urdf_file': urdf_file,
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    # ----------------------------------------------------
    # 4. Hardware Mode (use_sim_time = false)
    #    Stops Gazebo and runs ONLY robot_state_publisher + RPLIDAR driver
    # ----------------------------------------------------
    urdf_content = ParameterValue(FileContent(urdf_path), value_type=str)

    hardware_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': urdf_content
        }],
        output='screen',
        condition=UnlessCondition(use_sim_time)
    )

    # rplidar_launch_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('rplidar_ros'),
    #             'launch',
    #             'rplidar.launch.py'
    #         ])
    #     ]),
    #     condition=UnlessCondition(use_sim_time)
    # )

    rplidar_launch_cmd = Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/lidar',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        )




    # ----------------------------------------------------
    # 5. SLAM Launch (har_slam)
    #    Delayed to ensure clock/hardware/scan data is alive
    # ----------------------------------------------------
    slam_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('har_slam'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    slam_delayed_action = TimerAction(
        period=slam_delay,
        actions=[slam_launch_cmd]
    )

    # ----------------------------------------------------
    # 6. Nav2 Launch (har_navigation)
    #    Delayed further to ensure SLAM map & TF transforms are available
    # ----------------------------------------------------
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('har_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items()
    )

    nav2_delayed_action = TimerAction(
        period=nav2_delay,
        actions=[nav2_launch_cmd]
    )


    camera_pub = Node(package='har_hardware', 
    executable='camera_pub.py',
    # parameters=[{'frame_id': 'lidar','angle_compensate':True }], 
    output='screen')

    cmd_bridge = Node(package='har_hardware', 
    executable='cmd_vel_bridge.py',
    # parameters=[{'frame_id': 'lidar','angle_compensate':True }], 
    output='screen')

    odom_pub = Node(package='har_hardware', 
    executable='odom_pub.py',
    # parameters=[{'frame_id': 'lidar','angle_compensate':True }], 
    output='screen')

    serial_bridge = Node(package='har_hardware', 
    executable='serial_bridge.py',
    # parameters=[{'frame_id': 'lidar','angle_compensate':True }], 
    output='screen')

    hazard_node = Node(package='har_perception', 
    executable='hazard_scan_node.py',
    # parameters=[{'frame_id': 'lidar','angle_compensate':True }], 
    output='screen')

    serial_delayed_action = TimerAction(
        period=5.0,
        actions=[serial_bridge]
    )

    cam_delayed_action = TimerAction(
        period=10.0,
        actions=[camera_pub]
    )




    # ----------------------------------------------------
    # 7. Return LaunchDescription
    # ----------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_file_cmd,
        declare_urdf_file_cmd,
        declare_slam_params_file_cmd,
        declare_nav2_params_file_cmd,
        declare_slam_delay_cmd,
        declare_nav2_delay_cmd,
        # sim_launch_cmd,
        hardware_rsp_node,
        # cam_delayed_action,
        odom_pub,
        cmd_bridge,
        serial_delayed_action,
        hazard_node,
        rplidar_launch_cmd,
        slam_delayed_action,
        nav2_delayed_action,
    ])