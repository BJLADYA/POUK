from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_amcl', default_value='true', description='Use AMCL instead of SLAM'),
        DeclareLaunchArgument('map_name', default_value='map.yaml', description='Map name for AMCL'),
        
        # Запуск симулятора
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('stage_ros2'), 'launch', 'stage.launch.py'])
        ),
        
        # Параметр use_sim_time
        SetParameter(name='use_sim_time', value=True),
        
        # Запуск SLAM (если use_amcl=false)
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('lab6'), 'launch', 'navigation.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('use_amcl'))
        ),
        
        # Запуск AMCL (если use_amcl=true)
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('lab6'), 'launch', 'navigation_amcl.launch.py']),
            condition=IfCondition(LaunchConfiguration('use_amcl')),
            launch_arguments={'map_name': LaunchConfiguration('map_name')}.items()
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('lab6'), 'config', 'navi.rviz'])]
        )
    ])