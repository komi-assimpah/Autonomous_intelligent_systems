from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_search_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='un-defined22',
    maintainer_email='komi-jean-paul.assimpah@etu.univ-cotedazur.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation_node = object_search_navigation.navigation_node:main',
            'camera_processor_node = object_search_navigation.camera_processor_node:main',
            'mapping_node = object_search_navigation.mapping_node:main',
        ], 
    },
)
