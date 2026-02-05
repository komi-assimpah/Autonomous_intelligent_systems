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
    
    robot_orchestrator_dir = get_package_share_directory('robot_orchestrator')
    realsense_config = os.path.join(robot_orchestrator_dir, 'config', 'realsense_color_only.yaml')
    
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        namespace='camera',
        output='screen',
        parameters=[realsense_config]
    )
    
    return LaunchDescription([
        robot_bringup,
        
        # RGB-D Packed Publisher (6 FPS for USB stability)
        Node(
            package='mqtt_rgb_bridge',
            executable='rgb_publisher',
            name='realsense_rgbd_publisher',
            output='screen',
            parameters=[{
                'width': 424,
                'height': 240,
                'fps': 6,
                'jpeg_quality': 50,
                'depth_quality': 1,
                'topic': '/camera/rgbd/compressed'
            }]
        ),
        
        # Node(
        #     package='robot_orchestrator',
        #     executable='fsm_node',
        #     name='robot_orchestrator',
        #     output='screen',
        #     parameters=[{'use_sim_time': False}]
        # ),
        
        # Camera TF: base_link -> camera_link 
        # Position: 5cm forward, 12cm up (matches D435i URDF)
        # Rotation: Roll=-90°, Pitch=0°, Yaw=-90° (RealSense optical frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_broadcaster',
            arguments=['0.05', '0', '0.12', '-1.5708', '0', '-1.5708', 'base_link', 'camera_link']
        ),
        
        # Node(
        #     package='object_search_navigation',
        #     executable='navigation_node',
        #     name='navigation_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': False}]
        # ),
    ])