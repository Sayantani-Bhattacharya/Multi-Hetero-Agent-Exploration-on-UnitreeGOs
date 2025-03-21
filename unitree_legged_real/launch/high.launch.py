
"""
Launches high level go controls for the go1
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true','false'],
            description='Open RVIZ for Go1 visualization'
        ),

        DeclareLaunchArgument(
            name='fixed_frame',
            default_value='base_footprint',
            description='Fixed frame for RVIZ'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go1_description'),
                    'launch',
                    'load_go1.launch.py'
                ])
            ),
            launch_arguments=[
                ('use_jsp', 'none'),
                ('fixed_frame', LaunchConfiguration('fixed_frame')),
                ('use_nav2_links', 'true'),
                ('use_rviz', LaunchConfiguration('use_rviz')),
            ],
        ),

        Node(
            package='unitree_legged_real',
            executable='udp_high',
            output='screen'
        ),

        Node(
            package='unitree_legged_real',
            executable='jsp_high',
            output='screen'
        )
    ])