"""Master demonstration launch file bringing up simulation, SLAM, Nav2, and Symbolic Explorer 2.0."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_dir = get_package_share_directory('explorer_sim')
    bringup_dir = get_package_share_directory('explorer_bringup')

    default_world = os.path.join(sim_dir, 'worlds', 'research_facility_world.sdf')
    default_nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    default_explorer_params = os.path.join(bringup_dir, 'config', 'explorer_params.yaml')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    nav2_params = LaunchConfiguration('nav2_params')
    explorer_params = LaunchConfiguration('explorer_params')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='True', description='Run Gazebo in headless mode'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='False', description='Start RViz visualization'
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world, description='Path to SDF simulation world'
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params', default_value=default_nav2_params, description='Path to Nav2 params file'
    )
    declare_explorer_params = DeclareLaunchArgument(
        'explorer_params', default_value=default_explorer_params, description='Path to Explorer params file'
    )

    # 1. Direct Gazebo + Online SLAM + Nav2 Bringup
    sim_and_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'simulation_and_nav.launch.py')
        ),
        launch_arguments={
            'headless': headless,
            'use_rviz': use_rviz,
            'world': world,
            'params_file': nav2_params,
            'x_pose': '0.0',
            'y_pose': '0.0',
            'yaw_pose': '0.0',
        }.items(),
    )

    # 2. Symbolic Reasoning, Frontier Detection, and Nav Adapter Stack
    exploration_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'exploration_stack.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': explorer_params,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_headless)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_world)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_explorer_params)
    ld.add_action(sim_and_nav)
    ld.add_action(exploration_stack)

    return ld
