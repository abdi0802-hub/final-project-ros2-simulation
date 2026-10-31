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

Robot muncul di tengah arena, dengan sensor aktif dan waktu simulasi berjalan.
Arena berukuran 30 × 30 m, memiliki tembok keliling, balok penghalang merah,
dan ramp hijau dengan pembatas kuning.

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
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint empat roda |
| `/clock` | `rosgraph_msgs/msg/Clock` | Waktu simulasi |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Transformasi robot dan sensor |
| `/robot_description` | `std_msgs/msg/String` | Deskripsi robot |

LiDAR memakai frame `lidar_link`. Node peserta yang menggunakan waktu atau TF
perlu `use_sim_time:=true`. Bridge ada di
`src/sim_bringup/config/bridge.yaml`; hanya `/cmd_vel` yang mengalir ROS → Gazebo.

## Opsi launch

```bash
# Posisi dan orientasi awal (meter dan radian)
ros2 launch sim_bringup dummy.launch.py x:=1.0 y:=-1.0 z:=0.1 yaw:=1.57

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
│   └── urdf/         # Model, penggerak, LiDAR, dan IMU
└── sim_bringup/
    ├── config/       # Bridge ROS ↔ Gazebo
    ├── launch/       # dummy.launch.py
    ├── test/         # Pemeriksaan aset dan antarmuka
    └── worlds/       # dummy_arena.sdf
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
```

Smoke test membuat ROS domain dan Gazebo partition terpisah, menjalankan arena
secara headless, memeriksa data sensor dan TF, kemudian menguji gerak maju serta
putaran positif dari posisi fisik robot di simulator. Bridge posisi untuk tes
berjalan hanya selama pengujian. Proses milik pengujian dihentikan setelah selesai. Log dan hasil
JSON tersedia di `log/smoke/`.

Workflow `.github/workflows/harmonic.yml` menyediakan pengujian yang sama untuk
Ubuntu 22.04/Humble dan Ubuntu 24.04/Jazzy pada push/PR GitHub. Renderer software
dipakai di CI; pengujian ini tidak memerlukan GPU khusus.

Ringkasan pengujian migrasi tersedia di [VALIDATION.md](VALIDATION.md).

Referensi: [migrasi ke Gazebo Sim](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/)
dan [pasangan ROS/Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/).
