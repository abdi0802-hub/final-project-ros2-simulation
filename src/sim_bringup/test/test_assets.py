"""Check installed simulation assets without sourcing the reference workspace."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
import xacro


def test_robot_meshes_and_scan_pattern_resolve_in_this_workspace():
    description = Path(get_package_share_directory('rover_steering_v1'))
    model = description / 'urdf/rover_steering_v1.urdf.xacro'
    expanded = xacro.process_file(str(model)).toxml()
    robot = ET.fromstring(expanded)
    for mesh in robot.findall('.//mesh'):
        uri = mesh.attrib['filename']
        assert uri.startswith('package://')
        package, relative = uri.removeprefix('package://').split('/', 1)
        assert (Path(get_package_share_directory(package)) / relative).is_file()
    patterns = robot.findall('.//csv_file_name')
    assert len(patterns) == 1
    pattern = Path(patterns[0].text)
    plugin_share = Path(get_package_share_directory('ros2_livox_simulation'))
    assert pattern == plugin_share / 'scan_mode/mid360.csv'
    assert pattern.is_file()
    assert 'simulasi_rover' not in expanded


def test_all_simulation_packages_are_built_in_one_workspace():
    packages = ('sim_bringup', 'rover_steering_v1',
                'ros2_livox_simulation', 'livox_ros_driver2')
    install_roots = {Path(get_package_prefix(name)).parent for name in packages}
    assert len(install_roots) == 1


def test_dummy_arena_contains_reference_obstacles_without_external_models():
    share = Path(get_package_share_directory('sim_bringup'))
    world = ET.parse(share / 'worlds/dummy_arena.sdf').getroot().find('world')
    models = {model.attrib['name'] for model in world.findall('model')}
    assert {'ground_plane', 'wall_north', 'wall_south', 'wall_east',
            'wall_west', 'obstacle_wall_90deg', 'traversable_ramp'} <= models
    assert not world.findall('.//include')


def test_livox_plugin_is_installed():
    prefix = Path(get_package_prefix('ros2_livox_simulation'))
    assert (prefix / 'lib/libros2_livox.so').is_file()
