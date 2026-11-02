"""Validate the distributable arena, robot, and ROS/Gazebo contract."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import pytest
import yaml

WORKSPACE = Path(__file__).resolve().parents[3]
BRINGUP = WORKSPACE / 'src/sim_bringup'
DESCRIPTION = WORKSPACE / 'src/rover_steering_v1'


def robot():
    return ET.parse(DESCRIPTION / 'urdf/rover_steering_v1.urdf.xacro').getroot()


def test_robot_has_standard_lidar_imu_and_harmonic_drive():
    model = robot()
    lidar = model.find(".//sensor[@type='gpu_lidar']")
    assert lidar is not None, 'Harmonic needs a gpu_lidar sensor'
    assert lidar.findtext('always_on') == 'true'
    assert lidar.findtext('topic') == '/scan'
    assert lidar.findtext('gz_frame_id') == 'lidar_link'
    assert model.find(".//sensor[@type='imu']").findtext('topic') == '/imu'
    drive = model.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    assert drive is not None
    assert {joint.text for joint in drive.findall('left_joint')} == {'wheel1_joint', 'wheel2_joint'}
    assert {joint.text for joint in drive.findall('right_joint')} == {'wheel3_joint', 'wheel4_joint'}
    assert drive.findtext('child_frame_id') == 'base_footprint'
    assert not model.findall('.//csv_file_name')
    assert all('libgazebo_' not in p.attrib['filename'] and 'livox' not in p.attrib['filename']
               for p in model.findall('.//plugin'))


def test_all_robot_meshes_are_bundled():
    meshes = robot().findall('.//mesh')
    assert len(meshes) == 9
    for mesh in meshes:
        uri = mesh.attrib['filename']
        assert uri.startswith('package://rover_steering_v1/')
        relative = uri.removeprefix('package://rover_steering_v1/')
        assert (DESCRIPTION / relative).is_file()


def test_link_inertias_satisfy_physical_principal_moment_constraints():
    def determinant(matrix):
        a, b, c = matrix[0]
        d, e, f = matrix[1]
        g, h, i = matrix[2]
        return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

    for link in robot().findall('link'):
        element = link.find('inertial/inertia')
        if element is None:
            continue
        values = {key: float(value) for key, value in element.attrib.items()}
        matrix = [[values['ixx'], values['ixy'], values['ixz']],
                  [values['ixy'], values['iyy'], values['iyz']],
                  [values['ixz'], values['iyz'], values['izz']]]
        assert matrix[0][0] > 0 and determinant(matrix) > 0, link.attrib['name']
        assert matrix[0][0] * matrix[1][1] - matrix[0][1] ** 2 > 0
        # A physical inertia tensor must have principal moments satisfying
        # the triangle inequalities. Its associated covariance is PSD.
        half_trace = sum(matrix[i][i] for i in range(3)) / 2
        covariance = [[(half_trace if i == j else 0) - matrix[i][j]
                       for j in range(3)] for i in range(3)]
        assert all(covariance[i][i] >= 0 for i in range(3)), link.attrib['name']
        for i, j in ((0, 1), (0, 2), (1, 2)):
            assert covariance[i][i] * covariance[j][j] - covariance[i][j] ** 2 >= -1e-12
        assert determinant(covariance) >= -1e-12, link.attrib['name']


def test_dummy_world_is_offline_and_loads_required_systems():
    world = ET.parse(BRINGUP / 'worlds/dummy_arena.sdf').getroot().find('world')
    models = {m.attrib['name'] for m in world.findall('model')}
    assert {'ground_plane', 'wall_north', 'wall_south', 'wall_east',
            'wall_west', 'central_platform', 'ramp_west', 'ramp_east',
            'ramp_north', 'ramp_south', 'aruco_0', 'aruco_1', 'aruco_2', 'aruco_3'} <= models
    systems = {p.attrib['name'] for p in world.findall('plugin')}
    assert {'gz::sim::systems::Physics', 'gz::sim::systems::UserCommands',
            'gz::sim::systems::SceneBroadcaster', 'gz::sim::systems::Sensors',
            'gz::sim::systems::Imu'} <= systems
    assert not world.findall('.//include'), 'Arena must not download Fuel models'
    assert not world.findall('.//material/script'), 'Classic materials are unsupported'


def test_bridge_exposes_the_public_ros_interface():
    mappings = yaml.safe_load((BRINGUP / 'config/bridge.yaml').read_text())
    by_ros = {m['ros_topic_name']: m for m in mappings}
    assert set(by_ros) == {'/clock', '/cmd_vel', '/odom', '/tf', '/joint_states',
                           '/imu', '/scan', '/points', '/camera/image_raw',
                           '/camera/camera_info'}
    assert by_ros['/cmd_vel']['direction'] == 'ROS_TO_GZ'
    assert all(m['direction'] == 'GZ_TO_ROS' for topic, m in by_ros.items() if topic != '/cmd_vel')
    assert by_ros['/points']['gz_topic_name'] == '/scan/points'
    assert by_ros['/points']['ros_type_name'] == 'sensor_msgs/msg/PointCloud2'
    assert by_ros['/scan']['ros_type_name'] == 'sensor_msgs/msg/LaserScan'
    assert all(m['gz_type_name'].startswith('gz.msgs.') for m in mappings)
    assert all('livox' not in m['ros_type_name'] for m in mappings)
    assert by_ros['/camera/image_raw']['ros_type_name'] == 'sensor_msgs/msg/Image'
    assert by_ros['/camera/camera_info']['ros_type_name'] == 'sensor_msgs/msg/CameraInfo'


def test_four_ramps_join_ground_to_platform_without_steps():
    world = ET.parse(BRINGUP / 'worlds/dummy_arena.sdf').getroot().find('world')
    platform = world.find("model[@name='central_platform']")
    assert platform is not None, 'Missing central navigation platform'
    assert [float(v) for v in platform.findtext('pose').split()] == [0, 0, 0.6, 0, 0, 0]
    assert platform.findtext('link/collision/geometry/box/size') == '8 8 1.2'
    expected = {'west': ((-10, 0), (-4, 0)), 'east': ((10, 0), (4, 0)),
                'north': ((0, 10), (0, 4)), 'south': ((0, -10), (0, -4))}
    for side, (toe, top) in expected.items():
        ramp = world.find(f"model[@name='ramp_{side}']")
        assert ramp is not None
        x, y, z, roll, pitch, yaw = map(float, ramp.findtext('pose').split())
        assert (x, y, z, roll, pitch) == (*toe, 0, 0, 0)
        meshes = ramp.findall('.//geometry/mesh')
        assert len(meshes) == 2
        for mesh in meshes:
            assert list(map(float, mesh.findtext('scale').split())) == [6, 3, 1.2]
            asset = BRINGUP / 'worlds' / mesh.findtext('uri')
            assert asset.is_file()
            ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
            tree = ET.parse(asset)
            raw = list(map(float, tree.find('.//c:float_array', ns).text.split()))
            points = [raw[i:i + 3] for i in range(0, len(raw), 3)]
            # Unit wedge: the toe is at ground, the complete upper edge at z=1.
            assert {tuple(p) for p in points if p[0] == 0} == {(0, -0.5, 0), (0, 0.5, 0)}
            assert {(1, -0.5, 1), (1, 0.5, 1)} <= {tuple(p) for p in points}
        assert (x + 6 * math.cos(yaw), y + 6 * math.sin(yaw)) == pytest.approx(top)


def test_world_link_children_have_unique_names():
    world = ET.parse(BRINGUP / 'worlds/dummy_arena.sdf').getroot().find('world')
    for link in world.findall('.//link'):
        names = [child.attrib['name'] for child in link if 'name' in child.attrib]
        assert len(names) == len(set(names)), 'SDF requires unique sibling names'


def test_corner_marker_boards_face_into_arena_with_white_margin():
    world = ET.parse(BRINGUP / 'worlds/dummy_arena.sdf').getroot().find('world')
    coordinates = [(13, 13), (-13, 13), (-13, -13), (13, -13)]
    for marker_id, coordinate in enumerate(coordinates):
        marker = world.find(f"model[@name='aruco_{marker_id}']")
        assert marker is not None, f'Missing ArUco ID {marker_id}'
        x, y, z, roll, pitch, yaw = map(float, marker.findtext('pose').split())
        assert (x, y) == coordinate
        assert (z, roll, pitch) == (1.1, 0, 0)
        # Board normal +X must face the center.
        assert math.cos(yaw) == pytest.approx(-x / math.hypot(x, y))
        assert math.sin(yaw) == pytest.approx(-y / math.hypot(x, y))
        board = marker.find("link/visual[@name='white_board']")
        assert list(map(float, board.findtext('geometry/box/size').split())) == [0.05, 1.2, 1.2]
        cells = marker.findall("link/visual")
        cells = [cell for cell in cells if cell.attrib['name'].startswith('cell_')]
        assert cells
        for cell in cells:
            cx, cy, cz, *_ = map(float, cell.findtext('pose').split())
            assert cx > 0.025, 'Black cells must be in front of board'
            assert abs(cy) + 0.075 <= 0.451
            assert abs(cz) + 0.075 <= 0.451


@pytest.mark.parametrize('marker_id', range(4))
def test_corner_marker_decodes_from_visible_front_face(marker_id):
    # Optional decoder check: the simulation itself does not depend on OpenCV.
    cv2 = pytest.importorskip('cv2')
    if not hasattr(cv2, 'aruco'):
        pytest.skip('OpenCV ArUco module is unavailable')
    np = pytest.importorskip('numpy')
    world = ET.parse(BRINGUP / 'worlds/dummy_arena.sdf').getroot().find('world')
    marker = world.find(f"model[@name='aruco_{marker_id}']")
    image = np.full((720, 720), 255, dtype=np.uint8)
    for cell in marker.findall('link/visual'):
        if not cell.attrib['name'].startswith('cell_'):
            continue
        _, y, z, *_ = map(float, cell.findtext('pose').split())
        # Viewed upright from +X looking toward -X, screen-right is +Y.
        u, v = round(360 + y * 600), round(360 - z * 600)
        image[v - 45:v + 45, u - 45:u + 45] = 0
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    assert ids is not None, 'Front-facing marker must decode without mirroring'
    assert ids.flatten().tolist() == [marker_id]


def test_front_camera_has_calibration_and_optical_frame():
    model = robot()
    sensor = model.find(".//sensor[@type='camera']")
    assert sensor is not None, 'ArUco tasks need a camera'
    assert sensor.findtext('topic') == '/camera/image_raw'
    assert sensor.findtext('gz_frame_id') == 'camera_optical_frame'
    assert sensor.findtext('camera/camera_info_topic') == '/camera/camera_info'
    assert sensor.findtext('camera/image/width') == '1280'
    assert sensor.findtext('camera/image/height') == '720'
    optical = model.find("joint[@name='camera_optical_joint']")
    assert optical.attrib['type'] == 'fixed'
    assert optical.find('parent').attrib['link'] == 'camera_link'
    assert optical.find('child').attrib['link'] == 'camera_optical_frame'
    assert list(map(float, optical.find('origin').attrib['rpy'].split())) == pytest.approx(
        [-math.pi / 2, 0, -math.pi / 2])


def test_workspace_is_independent_of_classic_and_hardware_drivers():
    manifests = list((WORKSPACE / 'src').glob('*/package.xml'))
    assert {ET.parse(p).getroot().findtext('name') for p in manifests} == {'sim_bringup', 'rover_steering_v1'}
    for path in manifests:
        dependencies = {e.text for e in ET.parse(path).getroot() if e.tag.endswith('depend')}
        assert not dependencies & {'gazebo_ros', 'gazebo_plugins', 'gazebo_dev',
                                   'livox_ros_driver2', 'ros2_livox_simulation'}


def test_description_has_no_machine_specific_paths():
    for path in (BRINGUP / 'launch', BRINGUP / 'worlds', BRINGUP / 'config', DESCRIPTION):
        for extension in ('*.py', '*.xml', '*.xacro', '*.sdf', '*.yaml'):
            for asset in path.rglob(extension):
                content = asset.read_text()
                assert '/home/rdtzaa' not in content, str(asset)
                assert '/usr/share/gazebo-11' not in content, str(asset)
