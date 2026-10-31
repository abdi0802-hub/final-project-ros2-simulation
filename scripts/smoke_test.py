#!/usr/bin/env python3
"""Check spawn, sensors, and drive response in an isolated headless simulation."""

import argparse
import math
import os
from pathlib import Path
import random
import signal
import socket
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    log_dir = workspace / 'log/smoke'
    log_dir.mkdir(parents=True, exist_ok=True)
    with socket.socket() as port_probe:
        port_probe.bind(('127.0.0.1', 0))
        gazebo_port = port_probe.getsockname()[1]
    os.environ['ROS_DOMAIN_ID'] = str(random.randrange(130, 200))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['GAZEBO_MASTER_URI'] = f'http://127.0.0.1:{gazebo_port}'
    os.environ['GAZEBO_IP'] = '127.0.0.1'
    os.environ['ROS_LOG_DIR'] = str(log_dir / 'ros')
    os.environ['GAZEBO_LOG_PATH'] = str(log_dir / 'gazebo')

    # Set isolation before importing/initializing ROS middleware.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState, PointCloud2

    rclpy.init()
    node = rclpy.create_node('ros2_fp_smoke_test')
    received = {}
    subscriptions = []
    types = {
        '/clock': Clock, '/odom': Odometry, '/imu': Imu,
        '/joint_states': JointState, '/livox/lidar': CustomMsg,
        '/livox/lidar_PointCloud2': PointCloud2,
    }
    for topic, message_type in types.items():
        subscriptions.append(node.create_subscription(
            message_type, topic,
            lambda message, topic=topic: received.__setitem__(topic, message),
            qos_profile_sensor_data,
        ))
    commands = node.create_publisher(Twist, '/cmd_vel', 10)
    process = None
    log_path = log_dir / 'launch.log'
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'sim_bringup', 'dummy.launch.py',
                 'gui:=false', 'verbose:=true'],
                cwd=workspace, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if process.poll() is not None:
                    raise RuntimeError('Launch exited before sensor checks completed')
                if set(types) <= received.keys():
                    stamp = received['/clock'].clock
                    if stamp.sec * 1_000_000_000 + stamp.nanosec > 0:
                        break
            missing = set(types) - received.keys()
            if missing:
                raise RuntimeError(f'Timed out waiting for topics: {sorted(missing)}')
            print('Received clock, odom, joints, IMU, and both Livox cloud types.',
                  flush=True)
            stamp = received['/clock'].clock
            assert stamp.sec * 1_000_000_000 + stamp.nanosec > 0, (
                'Simulation clock is not advancing'
            )
            assert received['/livox/lidar'].point_num > 0, 'Livox cloud is empty'
            cloud = received['/livox/lidar_PointCloud2']
            assert cloud.width * cloud.height > 0 and cloud.data, 'PointCloud2 is empty'
            assert received['/joint_states'].name, 'Joint state names are empty'
            assert math.isfinite(received['/imu'].linear_acceleration.z)

            # Allow the robot to settle before measuring its drive response.
            settle_until = time.monotonic() + 2.0
            while time.monotonic() < settle_until:
                rclpy.spin_once(node, timeout_sec=0.1)
            start_position = received['/odom'].pose.pose.position
            start_x, start_y = start_position.x, start_position.y
            command = Twist()
            command.linear.x = 0.2
            moved = 0.0
            move_deadline = time.monotonic() + 30.0
            while time.monotonic() < move_deadline:
                commands.publish(command)
                rclpy.spin_once(node, timeout_sec=0.1)
                position = received['/odom'].pose.pose.position
                moved = math.hypot(position.x - start_x, position.y - start_y)
                if moved > 0.05:
                    break
            commands.publish(Twist())
            assert moved > 0.05, f'Rover did not respond to /cmd_vel: {moved:.3f} m'
            content = log_path.read_text(errors='replace')
            assert 'Successfully spawned entity [rover]' in content, 'Spawn did not succeed'
            for error in ('Unable to find shader lib', 'Failed to load plugin',
                          'cannot get csv file', 'Failed to initialize scene'):
                assert error not in content, error
            print('PASS: rover spawned in dummy arena; clock, odom, joints, IMU,')
            print(f'Livox CustomMsg/PointCloud2 received; drive response {moved:.3f} m.')
            print(f'Launch log: {log_path}')
    except Exception:
        print(f'Simulation verification failed. Launch log: {log_path}')
        if log_path.exists():
            print(log_path.read_text(errors='replace')[-10000:])
        raise
    finally:
        # Signal only the process group created by this test, including Gazebo.
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
