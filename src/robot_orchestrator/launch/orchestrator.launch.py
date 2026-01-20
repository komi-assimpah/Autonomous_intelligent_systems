from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='dog',
        description='Target object class to detect (e.g. dog, cat, bottle)'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Launch simulation environment (Gazebo + RViz)'
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('object_search_navigation'), 'launch', 'sim_d435i.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    return LaunchDescription([
        target_class_arg,
        sim_arg,
        sim_launch,
        Node(
            package='robot_orchestrator',
            executable='fsm_node',
            name='robot_orchestrator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='object_search_navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='object_search_navigation',
            executable='camera_processor_node',
            name='camera_processor_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='ia_package',
            executable='inference',
            name='inference',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'target_class': LaunchConfiguration('target_class')
            }]
        ),
        Node(
            package='ia_package',
            executable='pointcloud_visualizer',
            name='pointcloud_visualizer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
