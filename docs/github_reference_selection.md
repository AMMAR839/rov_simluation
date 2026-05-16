# Seleksi Referensi GitHub Untuk Simulasi ROV KKI

Tujuan dokumen ini adalah memilih referensi yang benar-benar berguna untuk workspace `WS_ROV`, bukan memasukkan semua repository besar ke `src/`.

## Rekomendasi Utama

## Keputusan Implementasi Saat Ini

Yang dipakai langsung di workspace:

- `evan-palmer/blue`: mesh BlueROV2 dan T200 propeller. Alasannya: ROS 2 underwater, layout 6 thruster jelas, dan lisensi MIT.
- `AlePuglisi/ROV-Ricketts-ros2`: mesh `grip_claw.stl` dan `arm_link5.stl` untuk visual gripper. Alasannya: punya mesh gripper terpisah dan lisensi MIT.

Yang tidak dipakai langsung:

- `rov_gamantaray_1` Beaumont: sudah dicoba sebagai varian lokal, tetapi visualnya tidak rapi di arena KKI sehingga tidak lagi dijadikan opsi launch.
- `Robotic-Decision-Making-Lab/reach` dan dependency `alpha`: konsepnya bagus untuk Reach Alpha 5, tetapi lisensi mesh membatasi penggunaan untuk produk Reach Robotics. Karena itu aset mesh-nya tidak dicopy ke workspace ini.
- Full stack Angler/Blue/Ricketts: tidak dimasukkan penuh karena terlalu besar untuk target latihan KKI. Workspace hanya mengambil model/mesh kecil yang relevan, sedangkan kontrol, world KKI, QR, dan gripper manager tetap kode lokal.

### 1. `clydemcqueen/bluerov2_gz`

Link: https://github.com/clydemcqueen/bluerov2_gz

Kegunaan:

- referensi BlueROV2 untuk Gazebo Harmonic,
- memakai Buoyancy, Hydrodynamics, dan Thruster plugin,
- cocok sebagai pembanding untuk model heavy/base BlueROV2 dan world underwater.

Yang layak diambil:

- pola `models/` dan `worlds/`,
- konfigurasi plugin buoyancy/hydrodynamics/thruster,
- pendekatan model BlueROV2 Harmonic.

Yang tidak perlu diambil dulu:

- ArduSub, MAVProxy, dan `ardupilot_gazebo`, karena terlalu berat untuk latihan misi KKI berbasis ROS topic/stik.

### 2. `CentraleNantesROV/bluerov2`

Link: https://github.com/CentraleNantesROV/bluerov2

Kegunaan:

- ROS 2 + Gazebo/Ignition BlueROV2,
- punya robot description, hydrodynamics plugin, dan topik thruster Newton,
- mirip dengan folder lokal `/home/ammar/Documents/rov_gamantaray_2`.

Yang layak diambil:

- `thrusters.xacro` untuk layout 6 thruster,
- `hydrodynamics.xacro` untuk parameter drag/added mass awal,
- pola bridge topic sensor/thruster.

Yang tidak perlu diambil dulu:

- seluruh controller eksternal jika dependency-nya membuat workspace berat. Untuk KKI, allocator sederhana + joystick sudah cukup sebagai baseline.

### 3. Gazebo Sim Official Underwater Vehicle Tutorial

Link: https://gazebosim.org/api/sim/9/underwater_vehicles.html

Kegunaan:

- referensi resmi Gazebo untuk buoyancy, thruster, hydrodynamics, dan added mass,
- menjelaskan bahwa buoyancy perlu volume/massa yang masuk akal,
- menjelaskan thruster command dalam Newton dan damping agar robot tidak terus menambah kecepatan.

Yang layak diambil:

- formula netral buoyancy `volume = mass / water_density`,
- pola plugin `gz-sim-buoyancy-system`, `gz-sim-thruster-system`, dan `gz-sim-hydrodynamics-system`,
- ide damping dari parameter `xUabsU`, `yVabsV`, `zWabsW`, `nRabsR`.

### 4. `osrf/vrx`

Link: https://github.com/osrf/vrx

Kegunaan:

- referensi world maritim Gazebo/ROS,
- bagus untuk visual wave/surface dan environment laut.

Yang layak diambil:

- konsep water surface / wave visual jika nanti ingin surface lebih hidup.

Yang tidak perlu diambil dulu:

- seluruh VRX, karena fokusnya kapal permukaan WAM-V, bukan ROV kolam bawah air. Untuk KKI, kolam dangkal lebih cocok memakai visual air lokal dan physics ROV sendiri.

### 5. `AlePuglisi/MBARI-vehicles-sim-ros2`

Link: https://github.com/AlePuglisi/MBARI-vehicles-sim-ros2

Kegunaan:

- referensi underwater world ROS 2 Jazzy,
- memberi pembanding bahwa simulasi visual bawah air yang lebih realistis sering memakai Stonefish, bukan Gazebo saja.

Yang layak diambil:

- konsep world bawah air, marker, sensor, dan dokumentasi pipeline.

Yang tidak perlu diambil dulu:

- Stonefish stack penuh, karena itu simulator berbeda dari Gazebo. Jika dipakai, workspace akan berubah arah menjadi Stonefish ROS 2, bukan Gazebo Harmonic murni.

### 6. `Robotic-Decision-Making-Lab/angler`

Link: https://github.com/Robotic-Decision-Making-Lab/angler

Kegunaan:

- referensi ROS 2 untuk underwater vehicle manipulator system,
- punya arah BlueROV2 Heavy + Reach Alpha 5 manipulator,
- cocok sebagai pembanding konsep bahwa ROV + manipulator sebaiknya diperlakukan sebagai satu sistem UVMS.

Yang layak diambil:

- konsep pemisahan vehicle control dan manipulator control,
- ide launch/konfigurasi platform yang modular.

Yang tidak perlu diambil dulu:

- full stack Angler, ArduSub, dan dependency manipulator karena terlalu besar untuk workspace KKI yang sekarang butuh stabil dan ringan.

### 7. Blue Robotics Newton Subsea Gripper

Link: https://bluerobotics.com/learn/newton-subsea-gripper-installation/

Kegunaan:

- bukan package GitHub, tetapi referensi bentuk gripper ROV yang nyata,
- cocok untuk memperbaiki visual gripper sederhana di workspace ini.

Yang layak diambil:

- bentuk gripper single-function open/close,
- posisi mounting di depan/bawah BlueROV2,
- logika kontrol momentary open/close.

## Keputusan Untuk Workspace Ini

Yang paling masuk akal untuk lomba KKI:

1. Tetap pakai `WS_ROV` sebagai workspace utama.
2. Ambil konsep/model kecil, bukan clone semua repo besar.
3. Prioritas perbaikan berikutnya:
   - pilihan model ROV `rov_variant:=github_blue|bluerov`,
   - gripper visual berbasis mesh Ricketts claw,
   - motion model kinematic dengan lag/damping agar terasa di air,
   - mode hydro yang memakai parameter dari BlueROV2/Gazebo official kalau sudah ada data massa dan buoyancy ROV.

Alasan tidak langsung full hydrodynamics:

- full physics sering lebih lambat dan mudah tidak stabil pada model kecil,
- data massa, volume displacement, dan thrust asli belum tersedia,
- untuk misi KKI, stabilitas latihan scan QR, ambil payload, dan pindah ke hook lebih penting daripada klaim gaya fluida absolut.
