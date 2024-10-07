#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the AMCL configuration file
    amcl_config = os.path.join(
        os.getcwd(),  # Replace with the correct path to your config
        'src',
        'dmap_localization',
        'config',
        'amcl_config.yaml'
    )

    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config],
        remappings=[
            ('/map', 'map'),
        ]
    )

    # Lifecycle manager to automatically activate the amcl node
    amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['amcl']
        }]
    )

    return LaunchDescription([
        amcl_node,
        amcl_lifecycle_manager
    ])