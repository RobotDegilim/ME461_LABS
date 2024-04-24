from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spawn_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'params'), glob('params/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmedmuhammad',
    maintainer_email='thisisdeahmed@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_service = spawn_service.spawn_service:main',
            'spawn_targets_service = spawn_service.spawn_targets_service:main',
            'user_spawn_service = spawn_service.user_spawn_service:main',
            'spawn_model = spawn_service.spawn:main'
        ],
    },
)
