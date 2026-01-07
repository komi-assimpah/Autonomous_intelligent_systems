"""
Launch file for SLAM + Autonomous exploration (NO object detection).
Use this to generate a map of an unknown environment.

Usage: ros2 launch object_search_navigation slam_explore.launch.py
Then: make map_save (to save the map)
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')
    
    # Cartographer SLAM
    cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_dir, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Autonomous navigation (FSM) - no YOLO, no camera processing
    navigation = Node(
        package='object_search_navigation',
        executable='navigation_node',
        name='navigation',
        output='screen'
    )
    
    return LaunchDescription([
        set_model,
        slam_launch,
        navigation,
    ])
