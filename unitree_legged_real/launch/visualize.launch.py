"""
Launches the RViz environment with required paramters. Serves as a way to visualize nodes running on the Jetson, without having to run RViz on it.
TODO: include into unitree_zed.launch.py
"""


import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):

    camera_name_val = ''
    camera_model_val = ''

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    if (camera_model_val == ''):
        camera_model_val = 'zed2i'

    # Rviz2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('unitree_legged_real'),
        'config',
        'zed_unitree.rviz'
    )

    # Rviz2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name_val,
        executable='rviz2',
        name=camera_model_val +'_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )


    return [
        rviz2_node
    ]

def generate_launch_description():
    return LaunchDescription([

        OpaqueFunction(function=launch_setup)
    ])