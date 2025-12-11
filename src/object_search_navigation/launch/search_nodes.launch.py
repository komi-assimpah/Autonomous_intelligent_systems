"""
Launch file for object search nodes ONLY (without Gazebo).
Use this when Gazebo is already running.

Launches:
- camera_processor_node: Processes camera images
- navigation_node: FSM exploration and obstacle avoidance
- inference: YOLO object detection (ia_package)

Usage:
    ros2 launch object_search_navigation search_nodes.launch.py target_class:=dog
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_target_class = DeclareLaunchArgument(
        'target_class',
        default_value='dog',
        description='Target object class to search for (e.g., dog, cat, person)'
    )
    
    target_class = LaunchConfiguration('target_class')
    
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
    
    # YOLO inference node (replaces simple_detector)
    inference = Node(
        package='ia_package',
        executable='inference',
        name='inference',
        output='screen',
        parameters=[{
            'target_class': target_class,
            'model_path': 'yolo11n-seg.pt',
            'conf_threshold': 0.5
        }]
    )
    
    return LaunchDescription([
        declare_target_class,
        camera_processor,
        navigation,
        inference,
    ])
