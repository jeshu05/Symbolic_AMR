from setuptools import find_packages, setup

package_name = 'explorer_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Jesvanth',
    maintainer_email='jesvanth@example.com',
    description='Symbolic mission planner and target evaluator for Symbolic Explorer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = explorer_mission.mission_node:main'
        ],
    },
)