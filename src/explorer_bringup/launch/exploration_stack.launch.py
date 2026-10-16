"""Launch file starting the autonomous reasoning and exploration stack."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('explorer_bringup')
    default_params = os.path.join(bringup_dir, 'config', 'explorer_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params, description='Path to parameters file'
    )

    frontier_node = Node(
        package='explorer_mapping',
        executable='frontier_node',
        name='frontier_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    knowledge_node = Node(
        package='explorer_knowledge',
        executable='knowledge_node',
        name='knowledge_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    mission_node = Node(
        package='explorer_mission',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    nav_adapter_node = Node(
        package='explorer_nav_adapter',
        executable='nav_adapter_node',
        name='nav_adapter_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(frontier_node)
    ld.add_action(knowledge_node)
    ld.add_action(mission_node)
    ld.add_action(nav_adapter_node)

    return ld