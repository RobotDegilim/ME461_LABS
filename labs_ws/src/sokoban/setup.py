from setuptools import find_packages, setup
import os

package_name = 'sokoban'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        relative_path = os.path.relpath(path, directory)
        for filename in filenames:
            paths.append((os.path.join('share', package_name, directory, relative_path), [os.path.join(path, filename)]))
    return paths

extra_files = package_files('urdf') + package_files('launch') + package_files('worlds') + package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + extra_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotdegilim',
    maintainer_email='robotdegilim@recaptcha.com',
    description='Sokonban game implementation with ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)