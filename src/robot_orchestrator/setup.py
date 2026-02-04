from setuptools import setup, find_packages

package_name = 'robot_orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/orchestrator.launch.py',
          'launch/robot.launch.py',
          'launch/pc.launch.py']),
        ('share/' + package_name + '/rviz',
         ['rviz/real_robot_exploration.rviz']),
        ('share/' + package_name + '/config',
         ['config/realsense_color_only.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='un-defined22',
    maintainer_email='komi-jean-paul.assimpah@etu.univ-cotedazur.fr',
    description='FSM robot orchestrator with navigation, obstacle avoidance, detection and point cloud projection',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fsm_node = robot_orchestrator.fsm_node:main',
        ],
    },
)
