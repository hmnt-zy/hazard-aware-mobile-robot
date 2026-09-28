from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Define Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # 2. Path to your custom params YAML in har_slam
    default_slam_params_file = PathJoinSubstitution([
        FindPackageShare('har_slam'),
        'config',
        'mapper_params_online_async.yaml'  #mapper_params_online_async.yaml
    ])

    # 3. Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock if true'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # 4. Include SLAM Toolbox online_async launch file with your custom params
    slam_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # 5. Return LaunchDescription
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        slam_launch_cmd
    ])