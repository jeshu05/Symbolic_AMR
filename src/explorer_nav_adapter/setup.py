from setuptools import find_packages, setup

package_name = 'explorer_nav_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jesvanth',
    maintainer_email='jesvanth@example.com',
    description='Nav2 integration and goal execution adapter for Symbolic Explorer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_adapter_node = explorer_nav_adapter.nav_adapter_node:main'
        ],
    },
)