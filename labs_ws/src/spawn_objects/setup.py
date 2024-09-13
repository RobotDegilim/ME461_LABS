from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spawn_objects'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.json'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotdegilim',
    maintainer_email='robotdegilim@robot.com',
    description='Spawn Objects in Gazebo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_objects = spawn_objects.spawn_objects:main',
        ],
    },
)
