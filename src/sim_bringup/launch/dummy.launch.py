"""Launch the bundled dummy arena and rover with Gazebo Harmonic."""
from pathlib import Path
import os
import re
import shlex
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _spawn_finished(event, context):
    if event.returncode != 0:
        return [Shutdown(reason='Rover spawn failed; see ros_gz_sim create output.')]
    return []


def _launch_simulation(context):
    try:
        versions = subprocess.run(
            ['gz', 'sim', '--versions'], check=False, capture_output=True,
            text=True, timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        raise RuntimeError(
            'Gazebo Harmonic is unavailable. Run bash scripts/install_dependencies.sh --check.'
        ) from error
    if versions.returncode != 0 or not re.search(r'(?m)^\s*8\.\d+', versions.stdout):
        raise RuntimeError(
            'This workspace requires Gazebo Harmonic (gz-sim 8) and its matching ros_gz bridge. '
            'Run bash scripts/install_dependencies.sh --check; install dependencies from '
            'the previous module or use --install. Detected: '
            + (versions.stdout.strip() or versions.stderr.strip() or 'no gz-sim version')
        )

    get = lambda name: LaunchConfiguration(name).perform(context)
    enabled = lambda name: get(name).lower() in {'true', '1', 'yes'}
    world = Path(get('world')).expanduser().resolve()
    if not world.is_file():
        raise RuntimeError(f'World file does not exist: {world}')
    description_share = Path(get_package_share_directory('rover_steering_v1'))
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    gazebo_share = Path(get_package_share_directory('ros_gz_sim'))
    model = description_share / 'urdf/rover_steering_v1.urdf.xacro'
    description = ParameterValue(xacro.process_file(str(model)).toxml(), value_type=str)
    arguments = ['-v', '2']
    if enabled('gui'):
        arguments.extend(['--render-engine-gui', get('render_engine_gui')])
    if not enabled('pause'):
        arguments.append('-r')
    if not enabled('gui'):
        arguments.append('-s')
        if enabled('headless_rendering'):
            arguments.append('--headless-rendering')
    arguments.append(str(world))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch/gz_sim.launch.py')),
        launch_arguments={
            'gz_args': shlex.join(arguments), 'gz_version': '8',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': description, 'use_sim_time': True}],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': str(bringup_share / 'config/bridge.yaml'),
                     'use_sim_time': True}],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', get('world_name'), '-name', 'rover',
                   '-allow_renaming', 'false', '-topic', 'robot_description',
                   '-x', get('x'), '-y', get('y'), '-z', get('z'), '-Y', get('yaw')],
    )
    return [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(description_share.parent)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(bringup_share / 'worlds')),
        gazebo, publisher, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=_spawn_finished)),
        spawn,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('sim_bringup'))
    defaults = {
        'world': (str(share / 'worlds/dummy_arena.sdf'), 'Path to an SDF world.'),
        'world_name': ('dummy_arena', 'World name inside the SDF.'),
        'gui': ('true', 'Open the Gazebo GUI.'),
        'render_engine_gui': (
            'ogre' if os.environ.get('ROS_DISTRO') == 'humble' else 'ogre2',
            'GUI renderer; Humble defaults to ogre for older Mesa compatibility. Sensors use ogre2.',
        ),
        'pause': ('false', 'Start with physics paused.'),
        'headless_rendering': ('false', 'Use EGL for sensors when gui:=false.'),
        'x': ('-11.0', 'Initial rover x coordinate in meters; west of the central ramp.'),
        'y': ('0.0', 'Initial rover y coordinate in meters.'),
        'z': ('0.1', 'Initial rover height in meters.'),
        'yaw': ('0.0', 'Initial rover yaw in radians.'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in defaults.items()],
        OpaqueFunction(function=_launch_simulation),
    ])
