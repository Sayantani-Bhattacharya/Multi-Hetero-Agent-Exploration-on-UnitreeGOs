
"""
Launches low level go controls for the go1
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals


def generate_launch_description():
     return LaunchDescription([

          DeclareLaunchArgument(name='display', default_value='all',
                                choices=['all', 'state_only', 'cmd_only', 'none'],
                                description='Choose which robot models you want to show in rviz.'),

          SetLaunchConfiguration(name='rviz_state_only',
                                 value=PathJoinSubstitution(
                                   [FindPackageShare('unitree_legged_real'),
                                                     'config',
                                                     'state_only.rviz'])),

          SetLaunchConfiguration(name='rviz_cmd_only',
                                 value=PathJoinSubstitution(
                                   [FindPackageShare('unitree_legged_real'),
                                                     'config',
                                                     'cmd_only.rviz'])),

          SetLaunchConfiguration(name='rviz_all',
                                 value=PathJoinSubstitution(
                                   [FindPackageShare('unitree_legged_real'),
                                                     'config',
                                                     'state_and_cmd.rviz'])),

          # Always run the udp!
          Node(package='unitree_legged_real',
               executable='udp_low',
               output='screen'),

          # Run two JSPs corresponding to real and commanded joint positions
          Node(package='unitree_legged_real',
               executable='jsp_low',
               parameters=[{"js_source": "cmd"}],
               namespace="cmd",
               output='screen'),

          Node(package='unitree_legged_real',
               executable='jsp_low',
               parameters=[{"js_source": "state"}],
               namespace="state",
               output='screen'),

          # Load robot models
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('go1_description'),
                                                           'launch',
                                                           'load_go1.launch.py'])),
                    launch_arguments=[
                         ('use_jsp', 'none'),
                         ('namespace', 'cmd'),
                         ('use_rviz', 'false')
                    ],
          ),

          IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('go1_description'),
                                                           'launch',
                                                           'load_go1.launch.py'])),
                    launch_arguments=[
                         ('use_jsp', 'none'),
                         ('namespace', 'state'),
                         ('use_rviz', 'false')
                    ],
          ),

          # Connect tfs to a world frame
          Node(package="tf2_ros",
               executable="static_transform_publisher",
               arguments=['--frame-id', 'world', '--child-frame-id', 'state/base', '--z', '0.5']),

          Node(package="tf2_ros",
               executable="static_transform_publisher",
               arguments=['--frame-id', 'world', '--child-frame-id', 'cmd/base', '--z', '0.5']),

          # Launch rviz
          Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=[
                    '-d', LaunchConfiguration('rviz_state_only'),
               ],
               condition=LaunchConfigurationEquals('display', 'state_only'),
          ),

          Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=[
                    '-d', LaunchConfiguration('rviz_cmd_only'),
               ],
               condition=LaunchConfigurationEquals('display', 'cmd_only'),
          ),

          Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=[
                    '-d', LaunchConfiguration('rviz_all'),
               ],
               condition=LaunchConfigurationEquals('display', 'all'),
          ),

    ])