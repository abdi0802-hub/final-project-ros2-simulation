"""Spawn the reference dummy arena and rover using only this workspace."""

from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    description_share = Path(get_package_share_directory('rover_steering_v1'))
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    livox_prefix = Path(get_package_prefix('ros2_livox_simulation'))
    model = description_share / 'urdf/rover_steering_v1.urdf.xacro'
    robot_description = ParameterValue(
        xacro.process_file(str(model)).toxml(), value_type=str,
    )

    arguments = [
        DeclareLaunchArgument(
            'world', default_value=str(bringup_share / 'worlds/dummy_arena.sdf'),
            description='Absolute path to the Gazebo world file.',
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # gazebo_ros builds its child process environment from these paths.
    # Include Classic's media so a fresh shell can locate its shader library.
    environment = [
        AppendEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11',
        ),
        AppendEnvironmentVariable(
            'GAZEBO_MODEL_PATH', str(description_share.parent),
        ),
        AppendEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH', str(livox_prefix / 'lib'),
        ),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch/gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('pause'),
            'verbose': LaunchConfiguration('verbose'),
            'server_required': 'true',
            'gui_required': 'true',
        }.items(),
    )
    publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=[
            '-topic', 'robot_description', '-entity', 'rover',
            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw'),
        ],
    )
    return LaunchDescription(arguments + environment + [gazebo, publisher, spawn])
