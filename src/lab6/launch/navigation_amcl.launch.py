from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declare_map_name = DeclareLaunchArgument(
		'map_name',
		default_value='map.yaml',
		description='Full path to map yaml file'
	)
    
    lifecycle_nodes = ['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator']
    use_sim_time = True
    autostart = True

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),  # Или True для симуляции
        
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': '/home/vlad/dev/humble_pouk_ws/src/lab6/maps/map.yaml',
                'use_sim_time': use_sim_time,
                'topic_name': 'map',
                'frame_id': 'map',
                'always_send_full_map': 'true',
                'lathc': 'true'
            }]
        ),
        
		# AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('lab6'),
                'config',
                'amcl_params.yaml'
            ])]
        ),

        # Nav2
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
            launch_arguments={
                'params_file': PathJoinSubstitution([FindPackageShare('lab6'), 'config', 'nav2_params.yaml'])
            }.items()
        ),
        
		Node(
			package='nav2_lifecycle_manager',
			executable='lifecycle_manager',
			name='lifecycle_manager',
			output='screen',
			emulate_tty=True,
			parameters=[{'use_sim_time': use_sim_time},
						{'autostart': autostart},
						{'node_names': lifecycle_nodes}]
        )
        
    ])