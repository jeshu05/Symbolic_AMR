import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'explorer_bringup'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jesvanth',
    maintainer_email='jesvanth@example.com',
    description='Complete bringup launch files, diagnostics, and configuration for Symbolic Explorer',
    license='Apache-2.0',
    tests_require=['pytest'],
)
