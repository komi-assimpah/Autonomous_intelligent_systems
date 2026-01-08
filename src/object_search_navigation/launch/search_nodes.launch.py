import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.actions import SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    object_search_dir = get_package_share_directory('object_search_navigation')
    nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    map_yaml_file = os.path.join(nav2_dir, 'map', 'map.yaml')
    
    rviz_config = os.path.join(object_search_dir, 'rviz', 'search_with_map.rviz')
    
    declare_target_class = DeclareLaunchArgument(
        'target_class',
        default_value='dog',
        description='Target object class to search for (e.g., dog, cat, person)'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Initial pose for AMCL (where the robot starts on the map)
    # Must match spawn position in sim_d435i.launch.py: x=-0.5, y=0.5
    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='-0.5',
        description='Initial X position on the map'
    )
    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.5',
        description='Initial Y position on the map'
    )
    declare_initial_pose_yaw = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial yaw orientation on the map'
    )
    
    target_class = LaunchConfiguration('target_class')
    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    
    # ============================================
    # NAV2 LOCALIZATION (map_server + AMCL)
    # ============================================

    # Map Server - loads the pre-recorded map
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        namespace = '',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )
    
    # AMCL - Adaptive Monte Carlo Localization
    # set_initial_pose: true means AMCL will use initial_pose_* params at startup
    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        namespace= '',
        parameters=[{
            'use_sim_time': use_sim_time,
            'set_initial_pose': True,
            'initial_pose.x': initial_pose_x,
            'initial_pose.y': initial_pose_y,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': initial_pose_yaw,
            # Frame configuration
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            # Particle filter settings
            'max_particles': 2000,
            'min_particles': 500,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'laser_model_type': 'likelihood_field',
            'laser_max_range': 3.5,
            'laser_min_range': 0.12,
            'scan_topic': 'scan',
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
        }]
    )
    
    # Lifecycle Manager - activates map_server and amcl
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    # ============================================
    # RVIZ2 - Custom config with map + pointclouds + markers
    # Fixed Frame: map (now available thanks to AMCL)
    # ============================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ============================================
    # SEARCH NODES
    # ============================================
    camera_processor = Node(
        package='object_search_navigation',
        executable='camera_processor_node',
        name='camera_processor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    navigation = Node(
        package='object_search_navigation',
        executable='navigation_node', 
        name='navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # YOLO inference node
    inference = Node(
        package='ia_package',
        executable='inference',
        name='inference',
        output='screen',
        parameters=[{
            'target_class': target_class,
            'model_path': 'yolo11n-seg.pt',
            'conf_threshold': 0.5,
            'use_sim_time': use_sim_time
        }]
    )

    # PointCloud visualizer for RViz2 confirmation
    pointcloud_visualizer = Node(
        package='ia_package',
        executable='pointcloud_visualizer',
        name='pointcloud_visualizer',
        output='screen',
        parameters=[{
            'target_class': target_class,
            'filter_radius': 0.5,
            'marker_size': 0.3,
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        declare_target_class,
        declare_use_sim_time,
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_yaw,
        # Nav2 Localization (map_server + AMCL)
        map_server,
        amcl,
        lifecycle_manager,
        rviz_node,
        camera_processor,
        navigation,
        inference,
        pointcloud_visualizer,
    ])
