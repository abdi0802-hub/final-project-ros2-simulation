# Validasi migrasi Harmonic

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
