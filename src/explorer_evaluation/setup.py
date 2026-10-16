from setuptools import find_packages, setup

package_name = 'explorer_evaluation'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'tabulate'],
    zip_safe=True,
    maintainer='Jesvanth',
    maintainer_email='jesvanth@example.com',
    description='Empirical Gazebo simulation benchmarks, baseline strategies, and metrics collection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_benchmarks = explorer_evaluation.benchmark_runner:main',
            'run_gazebo_benchmark = explorer_evaluation.gazebo_benchmark:main',
        ],
    },
)
