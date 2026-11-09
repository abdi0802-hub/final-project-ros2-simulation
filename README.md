# FP-ROS2-SIMULATION

Proyek ini adalah simulasi sistem navigasi otonom menggunakan ROS 2 Humble dan Gazebo. Robot rover ditugaskan untuk menyelesaikan dua misi utama: mendaki tanjakan (ramp) menggunakan kontrol *open-loop* berbasis sensor IMU, dan melakukan patroli navigasi *waypoint* menggunakan Nav2 untuk memindai penanda ArUco di empat sudut arena.

---

## 1. Prasyarat & Instalasi
Sistem ini dibangun dan diuji pada OS **Ubuntu 22.04** dengan **ROS 2 Humble**. 
Pastikan Anda sudah menginstal paket-paket esensial berikut sebelum menjalankan *workspace*:

```bash
# Instalasi paket ROS 2 standar dan Gazebo
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs

# Instalasi Navigasi (Nav2)
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Instalasi kontrol manual (Teleop)
sudo apt install ros-humble-teleop-twist-keyboard

2. Cara Build Workspace

Buka terminal baru, arahkan ke root directory dari workspace Anda (misalnya ~/Documents/ros2_fp), lalu jalankan perintah colcon build:
Bash

# Masuk ke direktori workspace
cd ~/Documents/ros2_fp

# Build package (direkomendasikan menggunakan symlink agar file Python bisa diedit tanpa build ulang)
colcon build --symlink-install

# Source workspace agar dikenali oleh sistem ROS 2
source install/setup.bash

(Catatan: Perintah source install/setup.bash wajib dijalankan di setiap terminal baru yang akan digunakan).

3. Pemilihan Mode Operasi

Proyek ini mendukung dua metode pergerakan robot:

    Mode Manual (Teleop): Menggunakan keyboard untuk mengendalikan pergerakan robot secara langsung.

    Mode Otonom: Terbagi menjadi dua skenario misi:

        Misi 1 (Ramp Climbing): Robot mendaki bidang miring secara mandiri. Misi ini menggunakan kontrol kecepatan langsung (/cmd_vel) yang dipantau oleh sensor kemiringan (IMU) untuk menghindari error/aborted dari costmap Nav2 akibat sudut elevasi lantai.

        Misi 2 (ArUco Patrol): Menggunakan Nav2 (BasicNavigator) untuk turun dari platform dan menelusuri 4 titik waypoint secara presisi untuk memindai papan ArUco.

4. Cara Menjalankan Skenario Misi Otonom

Gunakan terminal terpisah untuk masing-masing langkah di bawah ini. Pastikan Anda sudah menjalankan source install/setup.bash di setiap terminal.

Langkah 1: Jalankan Simulasi Gazebo
Bash

ros2 launch <NAMA_PACKAGE_LAUNCH> sim.launch.py

(Robot akan di-spawn pada posisi awal X: -11.0, Y: 0.0 di bawah ramp).

Langkah 2: Jalankan Nav2
Bash

ros2 launch <NAMA_PACKAGE_LAUNCH> nav2.launch.py

Langkah 3: Eksekusi Misi 1 (Naik ke Platform)
Bash

python3 src/rover_mission/src/misi_1.py --ros-args -p use_sim_time:=true

(Tunggu hingga robot berhasil mendaki dan berhenti sepenuhnya di tengah platform).

Langkah 4: Reset Lingkungan Nav2 (Wajib)
Karena elevasi ramp seringkali meninggalkan jejak rintangan palsu (halusinasi debu LiDAR) pada costmap, matikan proses Nav2 (Langkah 2) dengan menekan Ctrl + C, lalu jalankan ulang perintah peluncuran Nav2 tersebut agar costmap kembali bersih.

Langkah 5: Eksekusi Misi 2 (Keliling ArUco)
Bash

python3 src/rover_mission/src/misi_2.py --ros-args -p use_sim_time:=true


5. Tombol Kontrol (Mode Manual)

Jika Anda ingin mengendalikan robot secara manual (misalnya untuk pengujian pergerakan), buka terminal baru dan jalankan:
Bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Gunakan susunan tombol berikut untuk menggerakkan rover:

    i : Maju lurus

    , : Mundur lurus

    j : Berputar di tempat ke Kiri (Spin)

    l : Berputar di tempat ke Kanan (Spin)

    k : Rem / Berhenti total

    u / o : Maju sambil berbelok (Kiri / Kanan)

    m / . : Mundur sambil berbelok (Kiri / Kanan)
(Robot akan turun dari platform dan secara berurutan mengunjungi keempat titik ArUco).
