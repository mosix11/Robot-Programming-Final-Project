from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the map server launch file
    map_server_launch_file = os.path.join(
        os.getcwd(),  # Replace with the correct path to your config
        'launch',
        'map_server_launcher.py'
    )

    # Include the map server launch file
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_server_launch_file)
    )

    # Path to the turtlebot3_fake_node launch file
    turtlebot_launch_file = os.path.join(
        os.getcwd(),
        'src',
        'turtlebot3_simulations',
        'turtlebot3_fake_node',
        'launch',
        'turtlebot3_fake_node.launch.py'
    )

    # Include the turtlebot3_fake_node launch file
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot_launch_file)
    )

    # Delay turtlebot launch by 2 seconds 
    delayed_mapserver_launch = TimerAction(
        period=2.0,
        actions=[map_server_launch]
    )
    

    # Combine all launch descriptions and nodes
    return LaunchDescription([
        turtlebot_launch,
        delayed_mapserver_launch,
    ])