#!/usr/bin/env python3
"""Check real Harmonic sensors, transforms, forward motion, and turning."""
import argparse
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import time
import uuid


def yaw(orientation):
    return math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                      1 - 2 * (orientation.y ** 2 + orientation.z ** 2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--gui', action='store_true', help='Also launch the Gazebo GUI.')
    parser.add_argument('--arena-course', action='store_true',
                        help='Also drive up all four ramps and decode all corner markers (requires OpenCV ArUco).')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    logs = workspace / 'log/smoke'
    logs.mkdir(parents=True, exist_ok=True)
    partition = 'ros2_fp_test_' + uuid.uuid4().hex[:12]
    os.environ['ROS_DOMAIN_ID'] = str(random.randrange(130, 200))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['GZ_PARTITION'] = partition
    os.environ['IGN_PARTITION'] = partition
    os.environ['ROS_LOG_DIR'] = str(logs / 'ros')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import PoseArray, Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = rclpy.create_node('ros2_fp_smoke_test')
    received = {}
    counts = {}
    subscriptions = []
    message_types = {'/clock': Clock, '/odom': Odometry, '/imu': Imu,
                     '/joint_states': JointState, '/scan': LaserScan, '/points': PointCloud2,
                     '/_test/dynamic_poses': PoseArray,
                     '/camera/image_raw': Image, '/camera/camera_info': CameraInfo}

    def remember(topic, message):
        received[topic] = message
        counts[topic] = counts.get(topic, 0) + 1

    for topic, message_type in message_types.items():
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda message, topic=topic: remember(topic, message),
            qos_profile_sensor_data,
        ))
    transforms = Buffer()
    listener = TransformListener(transforms, node)
    commands = node.create_publisher(Twist, '/cmd_vel', 10)
    process = None
    ground_truth_bridge = None
    log_path = logs / 'launch.log'
    try:
        with log_path.open('w') as log, (logs / 'ground_truth.log').open('w') as truth_log:
            pose_topic = '/world/dummy_arena/dynamic_pose/info'
            ground_truth_bridge = subprocess.Popen(
                ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 pose_topic + '@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
                 '--ros-args', '-r', pose_topic + ':=/_test/dynamic_poses',
                 '-r', '__node:=ros2_fp_ground_truth'],
                cwd=workspace, stdout=truth_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            process = subprocess.Popen(
                ['ros2', 'launch', 'sim_bringup', 'dummy.launch.py',
                 'gui:=true' if args.gui else 'gui:=false',
                 'headless_rendering:=false' if args.gui else 'headless_rendering:=true'],
                cwd=workspace, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = time.monotonic() + args.timeout
            next_progress = time.monotonic() + 10
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if process.poll() is not None:
                    raise RuntimeError('Simulation exited before readiness')
                if set(message_types) <= received.keys() and counts.get('/clock', 0) > 1:
                    stamp = received['/clock'].clock
                    if stamp.sec * 1_000_000_000 + stamp.nanosec > 0:
                        try:
                            for frame in ('base_footprint', 'base_link', 'lidar_link', 'imu_link',
                                          'camera_link', 'camera_optical_frame'):
                                transforms.lookup_transform('odom', frame, Time())
                        except TransformException:
                            pass
                        else:
                            break
                if time.monotonic() > next_progress:
                    print('Waiting for simulation:', sorted(set(message_types) - received.keys()), flush=True)
                    next_progress = time.monotonic() + 10
            else:
                raise RuntimeError('Readiness timeout; missing topics: '
                                   + str(sorted(set(message_types) - received.keys()))
                                   + '; also check odom-to-sensor TF connectivity')
            scan = received['/scan']
            cloud = received['/points']
            assert len(scan.ranges) == 360, f'Unexpected scan size: {len(scan.ranges)}'
            assert any(math.isfinite(r) and scan.range_min < r < scan.range_max for r in scan.ranges), 'No arena returns in lidar scan'
            assert scan.header.frame_id == 'lidar_link', scan.header.frame_id
            assert cloud.header.frame_id == 'lidar_link', cloud.header.frame_id
            assert cloud.width * cloud.height == 360 and cloud.data, 'Empty or incorrect point cloud'
            assert received['/imu'].header.frame_id == 'imu_link'
            assert math.isfinite(received['/imu'].linear_acceleration.z)
            assert {'wheel1_joint', 'wheel2_joint', 'wheel3_joint', 'wheel4_joint'} <= set(received['/joint_states'].name)
            image = received['/camera/image_raw']
            info = received['/camera/camera_info']
            assert (image.width, image.height) == (1280, 720)
            assert image.encoding == 'rgb8' and len(image.data) == image.step * image.height
            assert (info.width, info.height) == (image.width, image.height)
            assert image.header.frame_id == info.header.frame_id == 'camera_optical_frame'
            assert info.k[0] > 0 and info.k[4] > 0 and info.k[8] == 1
            print('PASS: clock, odom, lidar scan/cloud, IMU, camera/calibration, joints and all TF frames.', flush=True)

            settle_until = time.monotonic() + (15 if args.gui else 2)
            while time.monotonic() < settle_until:
                rclpy.spin_once(node, timeout_sec=0.1)
            initial = received['/odom'].pose.pose
            initial_x, initial_y = initial.position.x, initial.position.y
            # SceneBroadcaster emits dynamic models before links. This bundled
            # world has exactly one dynamic model (rover), so its world pose is
            # the first entry. Read actual body motion, not wheel odometry.
            actual = received['/_test/dynamic_poses'].poses[0]
            actual_x, actual_y = actual.position.x, actual.position.y
            heading = yaw(actual.orientation)

            def drive_until(command, measure, threshold):
                deadline = time.monotonic() + 30
                next_command = 0
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError('Simulation exited during drive check')
                    if time.monotonic() >= next_command:
                        commands.publish(command)
                        next_command = time.monotonic() + 0.05
                    rclpy.spin_once(node, timeout_sec=0.05)
                    value = measure(received['/_test/dynamic_poses'].poses[0])
                    if value > threshold:
                        commands.publish(Twist())
                        return value
                commands.publish(Twist())
                raise RuntimeError(f'No correct drive response: {value:.4f}, expected > {threshold}')

            forward = Twist()
            forward.linear.x = 0.25
            distance = drive_until(
                forward,
                lambda pose: ((pose.position.x - actual_x) * math.cos(heading)
                              + (pose.position.y - actual_y) * math.sin(heading)),
                0.10,
            )
            odom_pose = received['/odom'].pose.pose
            odom_distance = math.hypot(odom_pose.position.x - initial_x,
                                       odom_pose.position.y - initial_y)
            assert odom_distance > 0.05, 'Odometry did not follow forward driving'
            start_yaw = yaw(received['/_test/dynamic_poses'].poses[0].orientation)
            turn = Twist()
            turn.angular.z = 0.3
            angle = drive_until(
                turn, lambda pose: math.atan2(math.sin(yaw(pose.orientation) - start_yaw),
                                             math.cos(yaw(pose.orientation) - start_yaw)),
                0.15,
            )
            course = {}
            if args.arena_course:
                import cv2
                import numpy as np
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

                def teleport(x, y, heading):
                    commands.publish(Twist())
                    request = (f'name: "rover", position: {{x: {x}, y: {y}, z: 0.1}}, '
                               f'orientation: {{z: {math.sin(heading / 2)}, '
                               f'w: {math.cos(heading / 2)}}}')
                    response = subprocess.run(
                        ['gz', 'service', '-s', '/world/dummy_arena/set_pose',
                         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                         '--timeout', '10000', '--req', request],
                        capture_output=True, text=True, timeout=15,
                    )
                    assert response.returncode == 0 and 'data: true' in response.stdout, response
                    deadline = time.monotonic() + 20
                    old_images = counts['/camera/image_raw']
                    while time.monotonic() < deadline:
                        commands.publish(Twist())
                        rclpy.spin_once(node, timeout_sec=0.05)
                        pose = received['/_test/dynamic_poses'].poses[0]
                        error = math.atan2(math.sin(yaw(pose.orientation) - heading),
                                           math.cos(yaw(pose.orientation) - heading))
                        if (math.hypot(pose.position.x - x, pose.position.y - y) < 0.15
                                and abs(error) < 0.05 and abs(pose.position.z) < 0.08
                                and counts['/camera/image_raw'] > old_images + 5):
                            return
                    raise RuntimeError(f'Robot did not settle at {(x, y, heading)}')

                ramp_results = {}
                for side, x, y, heading in (
                    ('west', -11, 0, 0), ('east', 11, 0, math.pi),
                    ('north', 0, 11, -math.pi / 2), ('south', 0, -11, math.pi / 2),
                ):
                    teleport(x, y, heading)
                    climb = Twist()
                    climb.linear.x = 0.8
                    deadline = time.monotonic() + args.timeout
                    next_progress = time.monotonic() + 10
                    while time.monotonic() < deadline:
                        commands.publish(climb)
                        rclpy.spin_once(node, timeout_sec=0.05)
                        pose = received['/_test/dynamic_poses'].poses[0]
                        distance_up = ((pose.position.x - x) * math.cos(heading)
                                       + (pose.position.y - y) * math.sin(heading))
                        if distance_up >= 8.0:
                            commands.publish(Twist())
                            assert abs(pose.position.z - 1.2) < 0.12, (side, pose)
                            ramp_results[side] = {'travel_m': distance_up, 'height_m': pose.position.z}
                            print(f'PASS: ramp {side}, height {pose.position.z:.3f} m.', flush=True)
                            break
                        if time.monotonic() >= next_progress:
                            print(f'Climbing {side}: {distance_up:.2f} m, z={pose.position.z:.2f}', flush=True)
                            next_progress = time.monotonic() + 10
                    else:
                        commands.publish(Twist())
                        raise RuntimeError(f'Could not climb ramp {side}; progress {distance_up:.3f} m')

                marker_results = {}
                for marker_id, (x, y) in enumerate(((10, 10), (-10, 10), (-10, -10), (10, -10))):
                    teleport(x, y, math.atan2(y, x))
                    deadline = time.monotonic() + 20
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                        image = received['/camera/image_raw']
                        # Respect ROS row stride, even when rows include padding.
                        pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
                        rgb = pixels[:, :image.width * 3].reshape(image.height, image.width, 3)
                        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
                        if hasattr(cv2.aruco, 'ArucoDetector'):
                            corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
                        else:
                            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
                        if ids is not None and marker_id in ids.flatten():
                            cv2.imwrite(str(logs / f'aruco_{marker_id}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                            marker_results[str(marker_id)] = {'approach_xy': [x, y], 'detected_ids': ids.flatten().tolist()}
                            print(f'PASS: rendered corner ArUco ID {marker_id}.', flush=True)
                            break
                    else:
                        cv2.imwrite(str(logs / f'aruco_{marker_id}_failed.png'), gray)
                        raise RuntimeError(f'ArUco {marker_id} was not readable from its approach')
                course = {'ramps': ramp_results, 'markers': marker_results}
            content = log_path.read_text(errors='replace')
            for error in ('Failed to load system plugin', 'Unable to find file',
                          'Failed to create', 'Service call timed out'):
                assert error not in content, error
            result = {'status': 'passed', 'ros_distro': os.environ.get('ROS_DISTRO'),
                      'forward_m': distance, 'turn_rad': angle,
                      'wheel_odometry_m': odom_distance,
                      'topic_counts': counts, 'scan_frame': scan.header.frame_id,
                      'point_frame': cloud.header.frame_id, 'arena_course': course}
            (logs / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            print(f'PASS: forward {distance:.3f} m and positive turn {angle:.3f} rad.', flush=True)
            print(f'Log: {log_path}', flush=True)
    except Exception:
        if log_path.exists():
            print(log_path.read_text(errors='replace')[-14000:], flush=True)
        raise
    finally:
        commands.publish(Twist())
        for child in (process, ground_truth_bridge):
            if child is None:
                continue
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
