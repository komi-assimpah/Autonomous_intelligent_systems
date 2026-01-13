import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, SetEnvironmentVariable, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    set_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', 
        value='burger'
    )
    
    # Path to TurtleBot3 Gazebo launch file
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_descriptions_dir = get_package_share_directory('turtlebot3_descriptions')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    launch_file_dir = os.path.join(turtlebot3_gazebo_dir, 'launch')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-0.5')
    y_pose = LaunchConfiguration('y_pose', default='0.5')
    headless = LaunchConfiguration('headless', default='false')
    
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI (faster)'
    )
    
    world = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')
    
    urdf_file = os.path.join(
        turtlebot3_descriptions_dir, 'urdf', 'turtlebot3_burger_d435i.urdf'
    )
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Set Gazebo resource path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo_dir, 'models')
    )
    
    # Gazebo server (always runs)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world]}.items()
    )
    
    # Gazebo client (GUI) - only runs if NOT headless
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items(),
        condition=UnlessCondition(headless)
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    
    # RViz2 with custom D435i config (always runs)
    object_search_nav_dir = get_package_share_directory('object_search_navigation')
    rviz_config_file = os.path.join(object_search_nav_dir, 'rviz', 'search_with_map.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )
    
    
    # === NAV2 MAP SERVER & AMCL ===
    map_file = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'map',
        'map.yaml'
    )

    nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_file}]
    )

    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'set_initial_pose': True},
                    {'initial_pose.x': x_pose},
                    {'initial_pose.y': y_pose},
                    {'initial_pose.yaw': 0.0}]
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )
    
    return LaunchDescription([
        declare_headless,
        set_model,
        set_env_vars_resources,
        gzserver_cmd,
        gzclient_cmd,  # Only runs if headless=false
        robot_state_publisher,
        spawn_turtlebot_cmd,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
        rviz_cmd,
    ])
