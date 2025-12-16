"""
Launch file for SLAM + Autonomous exploration + Object Detection.
Use this in an unknown environment to:
1. Build a map with Cartographer
2. Explore autonomously
3. Detect the target object with YOLO

Usage: ros2 launch object_search_navigation slam_n_search.launch.py target_class:=dog
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')
    
    # Target class argument
    declare_target_class = DeclareLaunchArgument(
        'target_class',
        default_value='dog',
        description='Target object class to search for'
    )
    target_class = LaunchConfiguration('target_class')
    
    # Cartographer SLAM
    cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_dir, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Camera processor
    camera_processor = Node(
        package='object_search_navigation',
        executable='camera_processor_node',
        name='camera_processor',
        output='screen'
    )
    
    # Autonomous navigation
    navigation = Node(
        package='object_search_navigation',
        executable='navigation_node',
        name='navigation',
        output='screen'
    )
    
    # YOLO inference (replaces old object_detector)
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
        set_model,
        declare_target_class,
        slam_launch,
        camera_processor,
        navigation,
        inference,
    ])
