import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'explorer_sim'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jesvanth',
    maintainer_email='jesvanth@example.com',
    description='Gazebo Harmonic simulation world and robot integration for Symbolic Explorer',
    license='Apache-2.0',
    tests_require=['pytest'],
)
