from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description(): 
    # Параметры для stage
    selector_control_params = os.path.join(
        get_package_share_directory('lab4'),
        'config',
        'selector_params.yaml'
	)

    use_sim_time = LaunchConfiguration('use_sim_time',  default='true')
    launch_dir = os.path.join(
        get_package_share_directory('stage_ros2'), 
        'launch')

    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_cmd = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment')
    

    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_cmd = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static')
   
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    stage = LaunchConfiguration('stage')
    declare_stage_cmd = DeclareLaunchArgument(
        'stage',
        default_value='True',
        description='Whether run a stage')

    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world', default_value='cave',
        description='world to load in stage and rviz config [cave, example]')

    return LaunchDescription([
        # Нода управления линией
        Node(
            package='lab4',
            executable='control_selector_node',
            name='control',
            output='screen',
            remappings=[('scan', 'base_scan')],
            parameters=[selector_control_params]
        ),
        
        declare_namespace_cmd,
        declare_stage_cmd,
        enforce_prefixes_cmd,
        one_tf_tree_cmd,
        declare_world,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'stage.launch.py')),
            condition=IfCondition(stage),
            launch_arguments={'one_tf_tree':one_tf_tree,
                              'enforce_prefixes':enforce_prefixes,
                              'world': world,
                              'use_sim_time': use_sim_time}.items()
        )
    ])