from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the map server launch file
    # map_server_launch_file = os.path.join(
    #     get_package_share_directory('your_map_server_package'),
    #     'launch',
    #     'your_map_server_launch_file.launch.py'
    # )
    map_server_launch_file = './launch/map_server_launcher.py'

    # Include the map server launch file
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_server_launch_file)
    )

    # Path to the turtlebot3_fake_node launch file
    turtlebot_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_fake_node'),
        'launch',
        'turtlebot3_fake_node.launch.py'
    )

    # Include the turtlebot3_fake_node launch file
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot_launch_file)
    )

    # Static transform publisher node
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Delay turtlebot launch by 4 seconds (2 seconds after static transform publisher)
    delayed_mapserver_launch = TimerAction(
        period=3.0,
        actions=[map_server_launch]
    )
    
    # Delay static transform publisher by 2 seconds
    delayed_static_transform = TimerAction(
        period=4.5,
        actions=[static_transform_publisher_node]
    )



    # Combine all launch descriptions and nodes
    return LaunchDescription([
        turtlebot_launch,
        delayed_mapserver_launch,
        delayed_static_transform,
    ])