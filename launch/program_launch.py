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
    amcl_launch_file = './launch/amcl_launch.py'

    # Include the map server launch file
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_server_launch_file)
    )
    
    # Include the map server launch file
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch_file)
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

    # Static transform publisher node for adding odom to map
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Static transform publisher node for adding laser scanner's base_scan to base_footprint
    # static_transform_publisher_node2 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_scan']
    # )
    
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
    
    delayed_amcl_launch = TimerAction(
        period=6.0,
        actions=[amcl_launch]
    )
    
    # delayed_static_transform2 = TimerAction(
    #     period=6.0,
    #     actions=[static_transform_publisher_node2]
    # )



    # Combine all launch descriptions and nodes
    return LaunchDescription([
        turtlebot_launch,
        # delayed_static_transform2,
        delayed_mapserver_launch,
        delayed_static_transform,
        # delayed_amcl_launch
    ])