#!/usr/bin/env python3
import math, os, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

class ImuWatch:
    def __init__(self, node):
        self.pitch = 0.0
        self.max_abs = 0.0
        node.create_subscription(Imu, '/imu', self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        q = msg.orientation
        if q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z > 0.5:
            self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.max_abs = max(self.max_abs, abs(self.pitch))

    def on_top(self):
        # Mendeteksi pernah nanjak (> 0.12 rad) lalu kembali rata (< 0.04 rad)
        return self.max_abs > 0.12 and abs(self.pitch) < 0.04

def main():
    rclpy.init()
    nav = BasicNavigator()
    imu = ImuWatch(nav)

    # Set pose awal robot di garis start (-11.0, 0.0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = -11.0 
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    nav.setInitialPose(initial_pose)
    
    print("Menunggu Nav2 aktif...")
    nav.waitUntilNav2Active()

    print("\n[SISTEM] Menjalankan Nav2 secara normal...")
    print("[SISTEM] AMCL aktif.")
    print("[SISTEM] Controller aktif.")
    print("[SISTEM] Costmap aktif.")
    print("[SISTEM] Robot mulai menanjak...")

    # Bersihkan sisa costmap sebelum mulai
    nav.clearAllCostmaps()

    # Kirim goal menuju ke tengah platform (0.0, 0.0)
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x = 0.0
    goal.pose.position.y = 0.0
    goal.pose.orientation.w = 1.0
    
    print("\n[MISI 1] Robot mulai menanjak...")
    nav.goToPose(goal)

    top_detected = False
    last_log = time.monotonic()

    while not nav.isTaskComplete():
        if imu.on_top() and not top_detected:
            top_detected = True
            print("\n[INFO] Robot sudah melewati ramp!")
            print("[INFO] Melanjutkan perjalanan menuju pusat platform (0,0)...")

            # Pulihkan AMCL jika sebelumnya ditunda
            os.system(
                'ros2 param set /amcl update_min_d 0.1 > /dev/null 2>&1'
            )
            os.system(
                'ros2 param set /amcl update_min_a 0.1 > /dev/null 2>&1'
            )

        feedback = nav.getFeedback()
        if feedback is not None:
            now = time.monotonic()
            if now - last_log >= 1.0:
                print(
                    f"[NAV2] Jarak menuju pusat: "
                    f"{feedback.distance_remaining:.2f} meter"
                )
                last_log = now

        time.sleep(0.05)

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("\n[SUKSES] Robot berhasil mencapai pusat platform!")
    elif result == TaskResult.CANCELED:
        print("\n[INFO] Navigasi dibatalkan.")
    else:
        print("\n[GAGAL] Robot tidak berhasil mencapai pusat platform.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()