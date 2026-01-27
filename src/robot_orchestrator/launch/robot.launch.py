from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_dir, 'launch', 'robot.launch.py')
        )
    )
    
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        namespace='camera',  # Topic will be /camera/color/image_raw
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,  # Disable depth to reduce USB load
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,   # Disable IMU - permission issues
            'enable_accel': False,  # Disable IMU - permission issues
            'color_width': 640,
            'color_height': 480,
            'color_fps': 15,
        }]
    )
    
    return LaunchDescription([
        robot_bringup,
        
        realsense_node,
        
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
        
        Node(
            package='object_search_navigation',
            executable='camera_processor_node',
            name='camera_processor_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])

