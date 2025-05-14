from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lab6'),
        'config',
        'slam_config.yaml'
    )

    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config]  # Явная передача конфига
        ),
        
        # Nav2
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
            launch_arguments={
                'params_file': PathJoinSubstitution([FindPackageShare('lab6'), 'config', 'nav2_params.yaml'])
            }.items()
        )
    ])