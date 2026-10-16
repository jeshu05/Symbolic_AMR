"""Direct simulation, online SLAM, and Nav2 bringup with tuned parameters and robust containerization."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('explorer_bringup')
    sim_dir = get_package_share_directory('explorer_sim')
    tb3_sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_world = os.path.join(sim_dir, 'worlds', 'facility_world.sdf')
    default_nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')
    params_file = LaunchConfiguration('params_file')

    declare_headless = DeclareLaunchArgument('headless', default_value='True')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='False')
    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw_pose', default_value='0.0')
    declare_params = DeclareLaunchArgument('params_file', default_value=default_nav2_params)

    # 1. Robot State Publisher
    urdf_path = os.path.join(tb3_sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )

    # 2. Gazebo Server
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world],
        output='screen',
    )

    # 3. Gazebo GUI Client
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(['not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g']}.items(),
    )

    # 4. Spawn Robot in Gazebo at exact coordinates
    robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_sim_dir, 'launch', 'spawn_tb3.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_name': 'turtlebot3_waffle',
            'robot_sdf': robot_sdf,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': '0.01',
            'yaw': yaw_pose,
        }.items(),
    )

    # 5. Nav2 Bringup with online SLAM, isolated container, and lifecycle management
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'True',
        }.items(),
    )

    # 6. RViz (if requested)
    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={'use_sim_time': 'true', 'rviz_config': rviz_config}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_headless)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_world)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_yaw)
    ld.add_action(declare_params)

    ld.add_action(robot_state_pub)
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    ld.add_action(spawn_robot)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_launch)

    return ld