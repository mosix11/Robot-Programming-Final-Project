#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # AMCL Node for Localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Use simulation time
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'scan_topic': 'scan',
        }],
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