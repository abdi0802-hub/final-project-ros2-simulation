# ros2_fp — simulasi rover dan arena dummy

Workspace ROS 2 Humble + Gazebo Classic 11, diadaptasi dari
`~/Documents/simulasi_rover`. Arena 30 × 30 m, rintangan, ramp, mesh rover,
penggerak empat roda, Livox MID-360, dan IMU mengikuti referensi.

## Paket

| Direktori di `src` | Nama paket ROS | Isi |
| --- | --- | --- |
| `sim_bringup` | `sim_bringup` | Launch dan arena dummy |
| `rover_steering_v1` | `rover_steering_v1` | Mesh dan deskripsi robot |
| `livox_laser_simulation_ros2` | `ros2_livox_simulation` | Plugin sensor dan pola scan Livox |
| `livox_ros_driver2` | `livox_ros_driver2` | Tipe pesan Livox dan source driver asli |

Nav2, FAST-LIO, dan aplikasi kontrol dari workspace referensi tidak disertakan.
Spawn tidak menjalankan node driver LiDAR fisik.

## Build

Gunakan terminal baru, cukup source ROS Humble. Workspace referensi tidak perlu
di-source. Dependency ROS/Gazebo dan Livox SDK2 sudah tersedia pada komputer ini.
Driver asli membutuhkan header dan library Livox SDK2 di `/usr/local` saat build.

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/ros2_fp
colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

Jika dipindahkan ke komputer lain, pasang dependency dengan `rosdep install
--from-paths src --ignore-src -r -y --rosdistro humble` dan pasang Livox SDK2
sebelum build. Source driver asli dan lisensinya tetap disertakan.

## Jalankan arena dan robot

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
cd ~/Documents/ros2_fp
source install/setup.bash
ros2 launch sim_bringup dummy.launch.py
```

Launch juga menambahkan path media/shader Gazebo, mesh, dan plugin Livox.
Tidak ada path runtime yang menunjuk `simulasi_rover` atau `Downloads`.
Sensor Livox diaktifkan dengan `always_on` agar menerbitkan data tanpa
bergantung pada visualisasi sensor di GUI Gazebo.

Tanpa jendela Gazebo:

```bash
ros2 launch sim_bringup dummy.launch.py gui:=false
```

Atur posisi spawn atau mulai dengan simulasi dijeda:

```bash
ros2 launch sim_bringup dummy.launch.py x:=1.0 y:=-1.0 z:=0.1 yaw:=1.57 pause:=true
```

Argumen `world:=/path/arena.sdf` dapat digunakan untuk world lain.
Hentikan simulasi dengan `Ctrl+C` di terminal launch.

## Topik

| Topik | Fungsi |
| --- | --- |
| `/cmd_vel` | Input `geometry_msgs/msg/Twist` untuk menggerakkan rover |
| `/odom` | Odometri robot |
| `/joint_states` | Posisi joint roda |
| `/imu` | Data IMU |
| `/livox/lidar` | Data `livox_ros_driver2/msg/CustomMsg` |
| `/livox/lidar_PointCloud2` | Data `sensor_msgs/msg/PointCloud2` |
| `/clock`, `/tf`, `/tf_static`, `/robot_description` | Waktu dan model robot |

## Pemeriksaan

Setelah source workspace hasil build:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/sim_bringup/test -q
python3 scripts/smoke_test.py
```

Smoke test menjalankan Gazebo tanpa GUI pada ROS domain dan port Gazebo terpisah,
memeriksa spawn dan data sensor, lalu memeriksa respons gerak terhadap `/cmd_vel`.
Hasil launch dicatat di `log/smoke/launch.log`.
