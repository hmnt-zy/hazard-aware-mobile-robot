from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Define Launch Configurations

    # 2. Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_param_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='nav2_params.yaml', # empty_bullet.sdf
        description='Navigation parameter file'
    )



    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')



    # 4. Construct path to the world file
    params_file_path = PathJoinSubstitution([
        FindPackageShare('har_navigation'),
        'config',
        params_file
    ])

    navigation_launch_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ])
    ]),
    launch_arguments={
        'params_file': params_file_path,
        'use_sim_time': use_sim_time,
    }.items()
)

    
    # 6. Return the LaunchDescription
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_param_file_cmd,
        navigation_launch_cmd,
    ])