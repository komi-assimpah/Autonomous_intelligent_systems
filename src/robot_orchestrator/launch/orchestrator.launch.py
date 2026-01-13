from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='ia_package',
            executable='pointcloud_visualizer',
            name='pointcloud_visualizer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
