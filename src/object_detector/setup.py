from setuptools import find_packages, setup

package_name = 'object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='un-defined22',
    maintainer_email='komi-jean-paul.assimpah@etu.univ-cotedazur.fr',
    description='Object detection node for autonomous search mission',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_detector_node = object_detector.simple_detector_node:main',
        ],
    },
)
