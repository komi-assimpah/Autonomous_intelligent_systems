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
            'use_sim_time': 'false'
        }.items()
    )
    
    return LaunchDescription([
        target_class_arg,
        use_rviz_arg,
        cartographer_launch,
        
        # Image decompressor: receives compressed JPEG from robot
        Node(
            package='object_search_navigation',
            executable='image_decompressor_node',
            name='image_decompressor',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'image_topic': '/camera/rgb/image_compressed',
                'show_window': True,
                'window_name': 'Robot Camera Feed'
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
        
        
        # RViz with custom config for real robot exploration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                robot_orchestrator_dir,
                'rviz', 'real_robot_exploration.rviz'
            )],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])

