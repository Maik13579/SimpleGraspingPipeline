#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_grasping')
    params_file = os.path.join(pkg_share, 'params', 'params.yaml')

    # Declare launch argument to remap the input cloud topic
    input_cloud_arg = DeclareLaunchArgument(
        'input_cloud_topic',
        default_value='~/input_cloud',
        description='Topic to subscribe for input cloud'
    )

    simple_grasping_node = Node(
        package='simple_grasping',
        executable='simple_grasping_node',
        name='simple_grasping_node',
        output='screen',
        parameters=[params_file],
        remappings=[('~/input_cloud', LaunchConfiguration('input_cloud_topic'))]
    )

    return LaunchDescription([
        input_cloud_arg,
        simple_grasping_node
    ])
