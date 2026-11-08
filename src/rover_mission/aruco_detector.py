#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import os
import json
from datetime import datetime

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # Subscribe ke kamera robot
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Direktori Penyimpanan
        self.save_dir = os.path.expanduser('~/aruco_captures')
        os.makedirs(self.save_dir, exist_ok=True)
        self.metadata_path = os.path.join(self.save_dir, 'metadata.json')

        # Load ID tersimpan sebelumnya
        self.metadata_records = self.load_metadata()
        self.saved_ids = {item['aruco_id'] for item in self.metadata_records}

        # Setup Kamus ArUco DICT_4X4_50
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        try:
            self.aruco_params = aruco.DetectorParameters_create()
            self.detector = None
        except AttributeError:
            self.aruco_params = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.current_frame = None
        self.detected_ids_in_frame = []

        self.get_logger().info("==========================================")
        self.get_logger().info("   NODE DETEKSI & CAPTURE ARUCO AKTIF    ")
        self.get_logger().info("==========================================")
        self.print_summary()
        self.get_logger().info("Petunjuk: Kendalikan robot manual via teleop.")
        self.get_logger().info("Tekan [SPASI] di jendela gambar untuk CAPTURE manual, atau [Q] untuk keluar.")

    def load_metadata(self):
        if os.path.exists(self.metadata_path):
            try:
                with open(self.metadata_path, 'r') as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().error(f"Gagal membaca metadata: {e}")
                return []
        return []

    def save_capture(self, frame, marker_id):
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        img_filename = f"aruco_ID_{marker_id}_{timestamp_str}.png"
        img_full_path = os.path.join(self.save_dir, img_filename)

        # 1. Simpan Gambar ke Disk
        success_img = cv2.imwrite(img_full_path, frame)

        if not success_img:
            self.get_logger().error(f"[GAGAL] Gagal menyimpan file gambar ke {img_full_path}")
            return False

        # 2. Tambah Metadata jika gambar berhasil terbuat
        record = {
            "aruco_id": int(marker_id),
            "filename": img_filename,
            "filepath": img_full_path,
            "timestamp": datetime.now().isoformat(),
            "status_valid": True
        }
        
        self.metadata_records.append(record)
        
        try:
            with open(self.metadata_path, 'w') as f:
                json.dump(self.metadata_records, f, indent=4)
            success_meta = True
        except Exception as e:
            self.get_logger().error(f"[GAGAL] Penulisan metadata gagal: {e}")
            success_meta = False

        # 3. Tampilkan Konfirmasi HANYA jika penulisan berhasil
        if success_img and success_meta:
            self.saved_ids.add(int(marker_id))
            print("\n--------------------------------------------------")
            print(f" SUCCESS: ArUco ID {marker_id} berhasil disimpan!")
            print(f" File Gambar : {img_full_path}")
            print("--------------------------------------------------")
            self.print_summary()
            return True
        return False

    def print_summary(self):
        sorted_ids = sorted(list(self.saved_ids))
        print(f"\n [RINGKASAN ID TERSIMPAN]: {sorted_ids} (Total: {len(sorted_ids)} marker)")
        print(f" Total Record Valid : {len(self.metadata_records)}\n")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        display_frame = cv_image.copy()

        # Proses Deteksi Marker
        if self.detector:
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, rejected = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

        self.detected_ids_in_frame = []

        if ids is not None:
            aruco.drawDetectedMarkers(display_frame, corners, ids)
            for marker_id in ids.flatten():
                self.detected_ids_in_frame.append(marker_id)
                
                # Auto-capture jika ID baru terdeteksi
                if marker_id not in self.saved_ids:
                    self.save_capture(cv_image, marker_id)

        # Tampilkan Jendela UI Real-Time
        cv2.imshow("Kamera Rover - Deteksi ArUco", display_frame)
        key = cv2.waitKey(1) & 0xFF

        # Tombol SPASI untuk capture manual
        if key == 32:  # Spacebar
            if ids is not None:
                for m_id in ids.flatten():
                    self.save_capture(cv_image, m_id)
            else:
                self.get_logger().warn("Tidak ada ArUco terdeteksi di kamera untuk dicapture!")
        elif key == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()