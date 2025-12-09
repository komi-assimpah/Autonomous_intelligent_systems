"""
Launch file for SLAM + FSM exploration + Object Detection.
Use this in an unknown environment to:
1. Build a map with Cartographer
2. Explore autonomously with FSM
3. Detect the target object

Usage: ros2 launch object_search_navigation slam_search.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')
    
    cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_dir, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    camera_processor = Node(
        package='object_search_navigation',
        executable='camera_processor_node',
        name='camera_processor',
        output='screen'
    )
    
    # Navigation (FSM exploration)
    navigation = Node(
        package='object_search_navigation',
        executable='navigation_node',
        name='navigation',
        output='screen'
    )
    
    detector = Node(
        package='object_detector',
        executable='simple_detector_node',
        name='object_detector',
        output='screen'
    )
    
    return LaunchDescription([
        set_model,
        slam_launch,
        camera_processor,
        navigation,
        detector,
    ])
