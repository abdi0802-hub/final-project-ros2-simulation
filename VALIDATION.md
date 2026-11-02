# Validasi migrasi Harmonic

## Preview Gazebo Classic — 2 November 2026

`python3 scripts/preview_classic.py` menghasilkan world sementara SDF 1.7
untuk melihat geometri arena pada Gazebo Classic 11. Model arena berasal dari
world utama yang sama; plugin Harmonic dikeluarkan dari salinan preview.

- `python3 scripts/preview_classic.py --check`: validator Classic lulus.
- Ke-14 model arena dipertahankan, dan URI mesh preview menunjuk aset lokal yang ada.
- 15 tes aset tetap lulus setelah penambahan helper.
- Tes startup `gzserver --iters 5` timeout pada preview maupun world kosong
  minimal di sesi ini; GUI preview belum diverifikasi.

## Perubahan arena platform dan ArUco — 1 November 2026

Arena kini berisi platform 8 × 8 × 1,2 m, empat ramp 3 × 6 m, dan empat
papan DICT_4X4_50 ID 0–3 di sudut arena. Rover memperoleh kamera RGB depan,
CameraInfo, frame optik, dan spawn default pada (-11, 0).

Verifikasi perubahan ini pada host pengembangan:

- Build kedua paket dengan `colcon build --symlink-install --executor sequential`: lulus.
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q`: 15 tes lulus,
  termasuk decode OpenCV dari proyeksi geometri muka depan keempat marker.
- `ctest --test-dir build/sim_bringup --output-on-failure`: lulus.
- `ign sdf -k` pada world dan hasil konversi URDF rover: valid (libsdformat 12).
- Mesh ramp tertutup dan aset mesh tersedia pada package share hasil install.
- Review terpisah menemukan refleksi pola marker; arah kolom diperbaiki dan
  empat tes decode yang sebelumnya gagal sekarang lulus.

Host saat ini mempunyai Gazebo Classic 11 / Fortress 6, tidak mempunyai
Gazebo Harmonic 8. Smoke test perubahan ini berhenti pada guard versi launch.
**Rendering kamera, pendakian fisik, dan decode gambar langsung di Harmonic
belum diverifikasi untuk arena baru.** Opsi `scripts/smoke_test.py --arena-course`
disediakan untuk menjalankan pemeriksaan tersebut di environment Harmonic;
memerlukan OpenCV Python dengan `cv2.aruco`.

Variabel `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` menghindari konflik plugin anyio
yang terpasang pada host dengan versi pytest sistem; tidak mengubah paket proyek.

## Hasil migrasi sebelum perubahan arena

Pengujian dilakukan pada 1 November 2026 dalam filesystem Ubuntu terisolasi,
dengan paket ROS/Gazebo dari apt dan renderer software Mesa/llvmpipe. Instalasi
ROS/Gazebo pada sistem laptop tidak diganti untuk pengujian ini.

| Ubuntu / ROS | Gazebo Harmonic | Build + 7 tes aset + runtime | Maju fisik | Belok fisik |
| --- | --- | --- | --- | --- |
| 22.04 / Humble | 8.15.0 | Lulus | 0.101 m | 0.151 rad |
| 24.04 / Jazzy | 8.15.0 | Lulus | 0.101 m | 0.154 rad |

Runtime memeriksa clock berjalan, odometri, joint roda, LaserScan dan PointCloud2,
IMU, frame sensor, serta rantai TF odom ke badan/sensor. Gerak diukur dari posisi
model sebenarnya pada simulator, bukan hanya putaran roda atau wheel odometry.
GUI juga dibuka di layar virtual dan screenshot tersedia dalam bukti pengujian.
GUI Humble memakai Ogre 1 untuk kompatibilitas renderer software Mesa Jammy;
GUI Jazzy memakai Ogre 2. Sensor memakai Ogre 2 pada kedua kombinasi.

Log, JSON hasil, dan screenshot disimpan di `log/validation-harmonic/` pada
workspace pengembangan. Folder log diabaikan Git; workflow CI menghasilkan log
dan JSON baru dari pengujian headless pada checkout baru. Pengujian tidak mencakup algoritma Nav2,
SLAM, atau perangkat/GPU fisik tertentu.

Perbaikan saat migrasi: nilai inersia sasis asli (0.1, 0.1, 0.5) melanggar syarat
fisik principal moments. Nilai diganti dengan box inertia untuk massa 5.3544 kg
dan ukuran 0.6 x 0.4 x 0.2 m: (0.08924, 0.17848, 0.232024). Geometri dan massa
rover tetap mengikuti referensi; tensor ini juga diperiksa oleh tes regresi.
