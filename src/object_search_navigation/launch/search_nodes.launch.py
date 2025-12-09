"""
Launch file for object search nodes ONLY (without Gazebo).
Use this when Gazebo is already running.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_processor = Node(
        package='object_search_navigation',
        executable='camera_processor_node',
        name='camera_processor',
        output='screen'
    )
    
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
        camera_processor,
        navigation,
        detector,
    ])
