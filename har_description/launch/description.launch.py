from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,FileContent,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition,IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
	# use_sim_time = true for using gazebo time when simulating .
	use_sim_time=LaunchConfiguration('use_sim_time',default='false')

	use_jsp=LaunchConfiguration('use_jsp',default='true')
	
	description_pkg = FindPackageShare('har_description')

	urdf_file = LaunchConfiguration('urdf_file', default='BB2.urdf')
	
	urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([description_pkg,'urdf', urdf_file]))
	
	urdf_content=ParameterValue(FileContent(urdf_path), value_type=str)
	# dynamically load rviz configuration file - for just showing robot model and TF
	# rviz_config_location=LaunchConfiguration('config_file', default=PathJoinSubstitution([description_pkg,'config','rviz.rviz']))
	
	# RViz Node
	start_rviz=Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		# arguments=['-d',rviz_config_location], # -d = Load Custom RViz Configuration File
		parameters=[{'use_sim_time':use_sim_time}],
		output='screen' # output prints in terminal else goes to log
	)
	# Robot State Publisher Node
	start_rsp=Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		parameters=[{'use_sim_time':use_sim_time,'robot_description':urdf_content}],
		output='screen'
	)
	# Joint State Publisher Node
	start_jsp=Node(
        package='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
		condition=IfCondition(use_jsp), # Run only if use_jsp is true
    )
	
	
	return LaunchDescription([
			start_rsp,
			start_jsp,
			# start_rviz,
			])