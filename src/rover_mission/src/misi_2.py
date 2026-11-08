#!/usr/bin/env python3
import math, time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Ramp timur membentang dari tepi platform x=4 hingga dasar ramp x=10.
TURUN_RAMP = ("Turun melalui ramp timur", 10.0, 0.0, 0.0)

# Titik berhenti digeser 1 m ke dalam dari waypoint sebelumnya.
ARUCO_POINTS = [
    ("ArUco ID 0",   9.0,   9.0,  0.7854),
    ("ArUco ID 1",  -9.0,   9.0,  2.3562),
    ("ArUco ID 2",  -9.0,  -9.0, -2.3562),
    ("ArUco ID 3",   9.0,  -9.0, -0.7854),
]
SCAN_SECONDS = 3.0

def make_goal(nav, x, y, yaw):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Beritahu Nav2 posisi awal ada di tengah platform
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0 
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()

    print("\n[MISI 2] Menahan posisi di tengah platform selama 2 detik...")
    time.sleep(2.0)
    nav.clearAllCostmaps()

    # === LANGKAH 1: TURUN LURUS DULU KE DASAR RAMP ===
    nama, x, y, yaw = TURUN_RAMP
    print(f"\n[FASE 1] Robot berjalan {nama} (X:{x}, Y:{y})...")

    nav.goToPose(make_goal(nav, x, y, yaw))
    
    while not nav.isTaskComplete():
        time.sleep(0.5)

    if nav.getResult() != TaskResult.SUCCEEDED:
        print("[GAGAL] Goal dasar ramp tidak tercapai; misi ArUco dihentikan.")
        nav.clearAllCostmaps()
        rclpy.shutdown()
        return

    print("[SUKSES] Robot mencapai dasar ramp timur.")

    time.sleep(2.0)
    nav.clearAllCostmaps()

    # === LANGKAH 2: KELILING ARUCO DI POJOK-POJOK ===
    for nama, x, y, yaw in ARUCO_POINTS:
        print(f"\n[FASE 2] Menuju {nama} ({x:.1f}, {y:.1f})...")

        nav.goToPose(make_goal(nav, x, y, yaw))
        
        while not nav.isTaskComplete():
            time.sleep(0.5)

        if nav.getResult() == TaskResult.SUCCEEDED:
            print(f"[SUKSES] Sampai di {nama}. Berhenti selama {SCAN_SECONDS:.0f} detik...")
            time.sleep(SCAN_SECONDS)
        else:
            print(f"[GAGAL] Gagal mencapai {nama}, lanjut ke titik berikutnya.")
            nav.clearAllCostmaps()

    print("\n--- SEMUA MISI SELESAI DENGAN SUKSES! ---")
    rclpy.shutdown()

if __name__ == '__main__':
    main()