from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )
    
    # Path to TurtleBot3 Gazebo launch file
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_house.launch.py')
        )
    )
    
    camera_processor = Node(
        package='object_search_navigation', # nom du package
        executable='camera_processor_node', # nom de l'exécutable
        name='camera_processor', # nom du noeud
        output='screen' # afficher la sortie log dans le terminal
    )
    
    navigation = Node(
        package='object_search_navigation',
        executable='navigation_node',
        name='navigation',
        output='screen'
    )
    
    detector = Node(
        package='object_detector',
        executable='simple_detector_node',
        name='object_detector',
        output='screen'
    )
    
    return LaunchDescription([
        set_turtlebot3_model,
        gazebo_launch,
        camera_processor,
        navigation,
        detector,
    ])
