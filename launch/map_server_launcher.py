from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True, 'yaml_filename': './src/files/diag_map.yaml'}],
            # parameters=[{'use_sim_time': True, 'yaml_filename': './src/files/map2.yaml'}],
            namespace='',
        ),
        # Lifecycle manager to automatically activate the map_server node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,  # This will automatically transition to active
                'node_names': ['map_server']
            }]
        )
    ])