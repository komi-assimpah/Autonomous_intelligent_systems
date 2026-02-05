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
        description='Target object class to detect (e.g. dog, cat, bottle, person)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    turtlebot3_cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
    robot_orchestrator_dir = get_package_share_directory('robot_orchestrator')
    
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_cartographer_dir, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_rviz': 'false'
        }.items()
    )
    
    return LaunchDescription([
        target_class_arg,
        use_rviz_arg,
        cartographer_launch,
        
        # RGB-D Decompressor: receives packed RGB+Depth from robot
        Node(
            package='object_search_navigation',
            executable='image_decompressor_node',
            name='image_decompressor',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'image_topic': '/camera/rgbd/compressed',
                'output_rgb_topic': '/camera/image_decompressed',
                'output_depth_topic': '/camera/depth/image_raw',
                'show_window': False
            }]
        ),
        
        Node(
            package='ia_package',
            executable='inference',
            name='inference',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'target_class': LaunchConfiguration('target_class')
            }]
        ),
        
        Node(
            package='robot_orchestrator',
            executable='fsm_node',
            name='robot_orchestrator',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        Node(
            package='object_search_navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        # DISABLED (inference.py now does direct 3D calc for the demon on the real robot)
        # Node(
        #     package='ia_package',
        #     executable='pointcloud_visualizer',
        #     name='pointcloud_visualizer',
        #     output='screen',
        #     parameters=[{'use_sim_time': False}]
        # ),
        
        
        # RViz with custom config for real robot exploration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                robot_orchestrator_dir,
                'rviz', 'real_robot_exploration.rviz'
            )]
        ),
    ])

