"""Simulation launch file starting Gazebo Harmonic, spawning the robot, and configuring ROS bridges."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('explorer_sim')
    tb3_sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='True', description='Run Gazebo in headless mode'
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'facility_world.sdf'),
        description='Full path to SDF world file',
    )
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='Initial robot X coordinate'
    )
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0', description='Initial robot Y coordinate'
    )

    # Robot State Publisher
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

    # Gazebo Server
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world],
        output='screen',
    )

    # Gazebo GUI Client (if not headless)
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(['not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g']}.items(),
    )

    # Spawn Robot in Gazebo
    robot_sdf = os.path.join(tb3_sim_dir, 'urdf', 'gz_waffle.sdf.xacro')
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
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(robot_state_pub)
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    ld.add_action(spawn_robot)

    return ld