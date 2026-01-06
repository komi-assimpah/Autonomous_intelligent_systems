from setuptools import find_packages, setup

package_name = 'ia_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics', 'matplotlib', 'scipy', 'numpy', 'sensor_msgs_py', 'opencv-python'],
    zip_safe=True,
    maintainer='yanis',
    maintainer_email='yanis@todo.todo',
    description='Package for AI inference and 3D visualization',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'inference = ia_package.inference:main',
            'point_cloud = ia_package.point_cloud:main',
        ],
    },
)
