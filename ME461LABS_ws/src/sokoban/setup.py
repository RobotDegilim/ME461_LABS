from setuptools import find_packages, setup
import os

package_name = 'sokoban'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

urdf_files = package_files('urdf')
launch_files = package_files('launch')
world_files = package_files('worlds')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', urdf_files),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/worlds', world_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usame_aw',
    maintainer_email='osamasaedawad@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
