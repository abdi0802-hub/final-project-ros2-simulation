# ros2_fp — arena rover untuk tugas internship

Arena dummy dan rover empat roda untuk **Gazebo Harmonic (gz-sim 8)**.
Environment mengikuti modul sebelumnya:

| Ubuntu | ROS 2 | Integrasi Gazebo |
| --- | --- | --- |
| 22.04 | Humble | `gz-harmonic` + `ros-humble-ros-gzharmonic` dari repository OSRF |
| 24.04 | Jazzy | `ros-jazzy-ros-gz` |

Workspace memakai sensor standar. Tidak memerlukan Livox SDK, driver hardware,
FAST-LIO, atau Gazebo Classic. Mesh rover dan geometri arena mengikuti referensi
`simulasi_rover`; world dan material disertakan secara lokal tanpa download Fuel.

## Lihat bentuk arena dengan Gazebo Classic 11

Untuk melihat platform, empat ramp, dan papan ArUco di laptop yang masih
menggunakan Gazebo Classic, jalankan:

```bash
cd ~/Documents/ros2_fp
python3 scripts/preview_classic.py
```

Tidak perlu build atau source ROS untuk preview ini. Kamera tampilan dimulai
dari atas secara miring agar seluruh arena terlihat. Gunakan mouse untuk
menggeser/memutar tampilan dan scroll untuk zoom. Tutup Gazebo untuk keluar.

Script membaca world arena terbaru dan membuat salinan sementara berformat
SDF 1.7 yang sesuai untuk Classic. Preview menampilkan geometri arena;
rover, sensor ROS, dan tugas navigasi memakai launch Harmonic di bawah.
Untuk memeriksa konversi tanpa membuka GUI: `python3 scripts/preview_classic.py --check`.

## Mulai dari environment modul sebelumnya

ROS 2 sesuai versi Ubuntu harus sudah terpasang. Buka terminal Bash baru:

```bash
cd ~/Documents/ros2_fp
bash scripts/install_dependencies.sh --check
```

Perintah tersebut hanya memeriksa dependency. Bila ada yang belum tersedia:

```bash
bash scripts/install_dependencies.sh --install
```

Mode instalasi menampilkan simulasi perubahan paket, lalu apt meminta konfirmasi.
Pada Humble, gunakan bridge **Humble–Harmonic** yang sama dengan modul sebelumnya;
`ros-humble-ros-gz` biasa menargetkan Fortress dan bukan penggantinya.

## Build dan jalankan

```bash
cd ~/Documents/ros2_fp
source scripts/setup_env.bash
colcon build --symlink-install --executor sequential
source install/local_setup.bash
ros2 launch sim_bringup dummy.launch.py
```

Pada terminal berikutnya cukup jalankan `source scripts/setup_env.bash` dari root
workspace. Jangan source setup Gazebo Classic atau workspace `simulasi_rover`.
Launch memeriksa keberadaan gz-sim 8 dan menghentikan proses bila versinya salah.

Robot muncul di `(-11, 0)` pada lantai arena, menghadap ramp sisi barat (+X),
dengan sensor aktif dan waktu simulasi berjalan. Arena berukuran **30 × 30 m**
dengan tembok keliling. Bagian tengah berisi platform **8 × 8 m**, permukaan
setinggi **1,2 m**, dan empat ramp hijau dari masing-masing sisi.
Setiap ramp selebar **3 m**, panjang mendatar **6 m**, kemiringan sekitar
**11,3°**, dengan ujung bawah pada x/y = ±10 m dan ujung atas pada x/y = ±4 m.
Ramp menggunakan mesh baji tertutup dengan collision yang menyambung ke lantai
dan platform. Area atas cukup untuk rover berhenti dan berputar; tepinya terbuka.

## Target ArUco di empat sudut arena

Empat papan tegak menghadap pusat arena. Gunakan dictionary **`DICT_4X4_50`**.
Sisi persegi marker hitam (termasuk border hitam, tanpa margin putih) adalah
**0,9 m**; papan putih **1,2 × 1,2 m**, pusat marker **z = 1,1 m** dari lantai.
Nilai panjang untuk estimasi pose ArUco adalah **`markerLength = 0.9`** meter.

| ID | Posisi pusat papan (x, y, z), meter | Posisi pendekatan contoh (x, y) | Yaw robot menuju papan |
| --- | --- | --- | --- |
| 0 | (13, 13, 1.1) | (10, 10) | 0.7854 |
| 1 | (-13, 13, 1.1) | (-10, 10) | 2.3562 |
| 2 | (-13, -13, 1.1) | (-10, -10) | -2.3562 |
| 3 | (13, -13, 1.1) | (10, -10) | -0.7854 |

Pola marker dibuat dari geometri sel hitam di depan papan putih dan dibundel
dalam world. Simulasi tidak membutuhkan unduhan tekstur atau OpenCV.
Peserta dapat memakai kamera depan untuk scan; algoritma navigasi autonomous,
SLAM, dan node detector ArUco dikerjakan oleh peserta.

Kamera menghadap +X robot, beresolusi **1280 × 720**, target **15 Hz**, horizontal
FOV **80°**. Gambar dan kalibrasi tersedia pada `/camera/image_raw` dan
`/camera/camera_info`. Frame `camera_optical_frame` mengikuti konvensi ROS:
+Z ke depan kamera, +X ke kanan gambar, +Y ke bawah.

## Kontrol dengan keyboard

Di terminal kedua:

```bash
cd ~/Documents/ros2_fp
source scripts/setup_env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Ikuti petunjuk tombol pada terminal teleop. Input `/cmd_vel` menggunakan
`geometry_msgs/msg/Twist`, dengan `linear.x` untuk maju/mundur dan `angular.z`
untuk belok. Batas kecepatan model adalah 1 m/s dan 1 rad/s.

## Antarmuka ROS untuk tugas

| Topik | Tipe | Keterangan |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Perintah gerak dari ROS ke simulator |
| `/odom` | `nav_msgs/msg/Odometry` | Odometri rover, frame `odom` → `base_footprint` |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR 2D 360°, 360 sampel, 10 Hz, jarak 0.1–30 m |
| `/points` | `sensor_msgs/msg/PointCloud2` | Titik dari pemindai 2D yang sama, bukan pemindaian 3D |
| `/imu` | `sensor_msgs/msg/Imu` | IMU, target 100 Hz, frame `imu_link` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Kamera RGB depan, 1280 × 720, target 15 Hz |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Kalibrasi intrinsik, frame `camera_optical_frame` |
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint empat roda |
| `/clock` | `rosgraph_msgs/msg/Clock` | Waktu simulasi |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Transformasi robot dan sensor |
| `/robot_description` | `std_msgs/msg/String` | Deskripsi robot |

LiDAR memakai frame `lidar_link`. Node peserta yang menggunakan waktu atau TF
perlu `use_sim_time:=true`. Bridge ada di
`src/sim_bringup/config/bridge.yaml`; hanya `/cmd_vel` yang mengalir ROS → Gazebo.

## Opsi launch

```bash
# Posisi dan orientasi awal di lantai (meter dan radian)
ros2 launch sim_bringup dummy.launch.py x:=-11.0 y:=-5.0 z:=0.1 yaw:=1.57

# Langsung di atas platform (permukaan z=1.2, beri clearance 0.1 m)
ros2 launch sim_bringup dummy.launch.py x:=0.0 y:=0.0 z:=1.3 yaw:=0.0

# Mulai dijeda; tekan Play di Gazebo
ros2 launch sim_bringup dummy.launch.py pause:=true

# Tanpa GUI, sensor dirender melalui EGL
ros2 launch sim_bringup dummy.launch.py gui:=false headless_rendering:=true

# Renderer software untuk sesi tanpa akselerasi GPU
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch sim_bringup dummy.launch.py gui:=false headless_rendering:=true
```

World lain dapat dipilih dengan `world:=/path/world.sdf world_name:=nama_world`.
Hentikan simulasi dengan `Ctrl+C` pada terminal launch.

GUI memakai `ogre` pada Humble untuk kompatibilitas Mesa Ubuntu 22.04, dan
`ogre2` pada Jazzy. Pilihan ini hanya memengaruhi tampilan; sensor tetap memakai
Ogre 2. Renderer GUI dapat dipilih melalui `render_engine_gui:=ogre2` atau `ogre`.

## Struktur source

```text
src/
├── rover_steering_v1/
│   ├── meshes/       # Mesh rover lokal
│   └── urdf/         # Model, penggerak, LiDAR, IMU, dan kamera
└── sim_bringup/
    ├── config/       # Bridge ROS ↔ Gazebo
    ├── launch/       # dummy.launch.py
    ├── test/         # Pemeriksaan aset dan antarmuka
    └── worlds/       # dummy_arena.sdf dan meshes/ramp.dae
scripts/
├── install_dependencies.sh
├── setup_env.bash
└── smoke_test.py
```

## Pemeriksaan otomatis

```bash
source scripts/setup_env.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 scripts/smoke_test.py

# Opsional: uji menaiki empat ramp dan decode gambar keempat marker.
# Memerlukan OpenCV Python dengan modul cv2.aruco, hanya untuk pengujian ini.
python3 scripts/smoke_test.py --arena-course --timeout 180
```

Smoke test membuat ROS domain dan Gazebo partition terpisah, menjalankan arena
secara headless, memeriksa data sensor, gambar/kalibrasi kamera dan TF, kemudian menguji gerak maju serta
putaran positif dari posisi fisik robot di simulator. Bridge posisi untuk tes
berjalan hanya selama pengujian. Proses milik pengujian dihentikan setelah selesai. Log dan hasil
JSON tersedia di `log/smoke/`.
Opsi `--arena-course` memindahkan rover ke awal setiap ramp untuk menguji pendakian
lurus, lalu ke posisi pendekatan setiap papan untuk menguji deteksi marker dari
gambar simulator. Hasil ramp dan ID disimpan pada JSON yang sama, serta gambar
`aruco_0.png` sampai `aruco_3.png` disimpan bila berhasil. Ini pemeriksaan arena;
bukan solusi navigasi autonomous untuk peserta.

Workflow `.github/workflows/harmonic.yml` menyediakan pengujian yang sama untuk
Ubuntu 22.04/Humble dan Ubuntu 24.04/Jazzy pada push/PR GitHub. Renderer software
dipakai di CI; pengujian ini tidak memerlukan GPU khusus.

Ringkasan pengujian migrasi tersedia di [VALIDATION.md](VALIDATION.md).

Referensi: [migrasi ke Gazebo Sim](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/)
dan [pasangan ROS/Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/).
