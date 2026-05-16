# Panduan Membuat Model ROV Sendiri

Dokumen ini menjelaskan cara membuat model ROV sendiri di workspace ini supaya nanti posisi thruster, propeller, kamera, lampu, dan gripper bisa kamu sesuaikan.

## Prinsip Utama

Model ROV jangan dibuat sebagai satu mesh besar tanpa struktur. Untuk simulasi yang mudah dikembangkan, pisahkan menjadi beberapa bagian:

- body/frame utama,
- collision sederhana untuk fisika,
- thruster dan propeller,
- kamera depan dan bawah,
- lampu,
- gripper mount dan rahang,
- sensor tambahan seperti IMU, DVL, depth sensor, atau sonar.

Mesh visual boleh detail, tetapi collision sebaiknya sederhana: box, cylinder, atau gabungan beberapa collision primitive. Buoyancy Gazebo dihitung dari collision volume, jadi collision harus mewakili volume displacement ROV, bukan sekadar bentuk visual.

## Struktur Folder Model

Buat folder baru di:

```text
src/rov_gamantaray_description/models/nama_rov_kamu/
```

Isi minimal:

```text
nama_rov_kamu/
├── model.config
├── model.sdf
└── meshes/
    ├── body.dae
    ├── propeller_cw.dae
    └── propeller_ccw.dae
```

`model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>nama_rov_kamu</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Tim Kamu</name>
  </author>
  <description>Model ROV custom untuk simulasi KKI.</description>
</model>
```

## Body Dan Collision

Di `model.sdf`, mulai dari satu `base_link`:

```xml
<model name="nama_rov_kamu">
  <static>true</static>
  <self_collide>false</self_collide>
  <link name="base_link">
    <inertial>
      <mass>13.0</mass>
      <inertia>
        <ixx>0.10</ixx><ixy>0</ixy><ixz>0</ixz>
        <iyy>0.18</iyy><iyz>0</iyz>
        <izz>0.27</izz>
      </inertia>
    </inertial>

    <collision name="body_collision">
      <geometry>
        <box>
          <size>0.35 0.26 0.14</size>
        </box>
      </geometry>
    </collision>

    <visual name="body_visual">
      <geometry>
        <mesh>
          <uri>model://nama_rov_kamu/meshes/body.dae</uri>
          <scale>1 1 1</scale>
        </mesh>
      </geometry>
    </visual>
  </link>
</model>
```

Untuk mode `kinematic`, model bisa `static=true` karena pose ROV digerakkan oleh node `kinematic_driver`. Untuk mode `hydro`, model harus `static=false`, punya massa, collision volume, buoyancy, dan hydrodynamics yang dituning.

## Thruster Dan Propeller

Tetapkan sistem koordinat body:

- `+X`: depan ROV,
- `+Y`: kiri ROV,
- `+Z`: atas.

Untuk 6 thruster BlueROV-style, simpan posisi relatif body:

```text
t1 depan-kanan horizontal
t2 depan-kiri horizontal
t3 belakang-kanan horizontal
t4 belakang-kiri horizontal
t5 vertikal kanan/kiri
t6 vertikal kanan/kiri
```

Di workspace ini, posisi propeller untuk mode kinematic dihitung di:

```text
src/rov_gamantaray_control/rov_gamantaray_control/kinematic_driver.py
```

Cari fungsi:

```python
make_prop_specs()
```

Kalau kamu membuat model baru, tambahkan layout baru, misalnya:

```python
if layout == "rov_custom":
    return [
        PropSpec("gamantaray_rov_thruster1_prop_visual", (0.11, -0.07, 0.00), ...),
        ...
    ]
```

Lalu di launch:

```text
src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
```

tambahkan `rov_variant` baru dan isi:

```python
rov_model_uri = "nama_rov_kamu"
prop_layout = "rov_custom"
```

## Kamera

Tambahkan sensor kamera sebagai bagian dari `base_link`.

Kamera depan:

```xml
<sensor name="wall_camera" type="camera">
  <pose>0.165 0 0.052 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>12</update_rate>
  <topic>/rov/camera/wall/image</topic>
  <camera>
    <horizontal_fov>1.20</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.02</near>
      <far>12.0</far>
    </clip>
  </camera>
</sensor>
```

Kamera bawah:

```xml
<sensor name="bottom_camera" type="camera">
  <pose>0.025 0 -0.105 0 1.5708 0</pose>
  <always_on>true</always_on>
  <update_rate>12</update_rate>
  <topic>/rov/camera/bottom/image</topic>
  ...
</sensor>
```

Pastikan topic sama dengan bridge di launch:

```text
/rov/camera/wall/image
/rov/camera/bottom/image
```

## Lampu

Lampu bisa dibuat sebagai visual dan sensor/spotlight. Untuk simulasi ringan, mulai dari visual dulu:

```xml
<visual name="left_light_glass">
  <pose>0.16 0.055 0.02 0 1.5708 0</pose>
  <geometry>
    <cylinder>
      <radius>0.014</radius>
      <length>0.010</length>
    </cylinder>
  </geometry>
  <material>
    <ambient>0.80 0.95 1.0 1</ambient>
    <diffuse>0.75 0.90 1.0 1</diffuse>
    <specular>1.0 1.0 1.0 1</specular>
  </material>
</visual>
```

Kalau butuh cahaya yang benar-benar menerangi scene, tambahkan `<light type="spot">`, tetapi hati-hati karena banyak lampu bisa menurunkan real-time factor.

## Gripper

Untuk visual yang menyatu dengan ROV, buat bagian tetap di `base_link`:

- mount/saddle,
- actuator housing,
- cheek plate,
- pivot pin,
- pivot cap.

Rahang yang bergerak di workspace ini masih dibuat sebagai model visual terpisah supaya mudah dianimasikan oleh `gripper_manager`. Secara visual, pivot-nya harus masuk ke cheek plate ROV. Parameter yang perlu disesuaikan:

```text
capture_forward_offset_m
capture_vertical_offset_m
closed_gap_m
open_gap_m
```

File:

```text
src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py
```

Kalau nanti ingin gripper benar-benar menjadi satu model fisik dengan ROV, jalur berikutnya adalah membuat dua `link` rahang dan dua `revolute joint` di `model.sdf`, lalu mengontrol joint dengan `gz-sim-joint-position-controller-system`. Itu lebih rapi secara model, tetapi butuh bridge command joint dan tuning PID joint.

## Mode Air/Fisika

Ada dua jalur:

### 1. Mode Kinematic

Ini default. Cocok untuk latihan misi, joystick, QR, pickup, dan pindah payload. Air dibuat visual realistis, sementara gerakan ROV diberi damping/lag oleh kode.

### 2. Mode Hydro

Ini untuk eksperimen fisika. Model perlu:

- `static=false`,
- massa dan inertia yang masuk akal,
- collision volume yang mendekati displacement,
- world plugin `gz-sim-buoyancy-system`,
- model plugin `gz-sim-hydrodynamics-system`,
- gaya thruster dari driver atau plugin thruster.

Untuk klaim yang kuat, parameter hydro harus ditune dari data uji kolam:

- massa aktual ROV,
- volume displacement,
- net buoyancy,
- kurva thrust motor di air,
- kecepatan maju/geser/naik/yaw dari ROV asli.

## Checklist Model Baru

Sebelum dipakai:

1. `xmllint --noout model.sdf`
2. Pastikan mesh bisa ditemukan lewat `model://`.
3. Pastikan ROV tidak lebih besar dari batas dimensi yang dipakai di lomba.
4. Pastikan propeller berada di tengah duct/ring.
5. Pastikan kamera depan/wall melihat QR samping payload saat ROV mendekat.
6. Pastikan gripper sejajar dengan payload.
7. `colcon build --symlink-install`
8. Jalankan launch dengan `rov_variant` baru.
