# Metode dan Logika Workspace ROV KKI

Dokumen ini menjelaskan metode, algoritma, dan alur kerja yang dipakai di workspace `WS_ROV`. Tujuannya adalah membuat simulasi ROV bawah air untuk latihan misi KKI: bergerak di kolam, membaca QR Code, mengambil payload, dan memindahkan payload ke gantungan A/B/C/D.

## 1. Ringkasan Arsitektur

Workspace ini memakai ROS 2 Jazzy dan Gazebo Harmonic. Simulasi dibagi menjadi beberapa package:

- `rov_gamantaray_bringup`: launch utama dan konfigurasi runtime.
- `rov_gamantaray_description`: model ROV, gripper, propeller, sensor, dan collision ROV.
- `rov_gamantaray_gazebo`: world kolam, payload QR, hook A/B/C/D, air, wake, dan collision proxy.
- `rov_gamantaray_control`: input stik/keyboard, allocator thruster, driver gerak, gripper, collision response, efek air, dan misi sederhana.
- `rov_gamantaray_vision`: deteksi QR Code dari kamera bawah.

Alur data utama:

```text
stik / keyboard
  -> /rov/cmd_vel
  -> thruster_allocator
  -> /rov/thruster_status
  -> kinematic_driver atau hydro_wrench_driver
  -> Gazebo ROV bergerak
  -> /model/gamantaray_rov/odometry
  -> gripper_manager, water_effects_driver, mission_supervisor
```

Alur payload:

```text
kamera bawah
  -> qr_detector
  -> /rov/qr_code
  -> gripper_manager / mission_supervisor
  -> payload dipilih, dijepit, dibawa, dan dilepas
```

## 2. Launch dan Pembentukan World

Launch utama:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/js0
```

File launch:

```text
src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
```

Metodenya:

1. Membaca argumen launch seperti `payload_code`, `rov_variant`, `physics_mode`, `joystick`, dan `use_vision`.
2. Memilih model ROV:
   - `rov_variant:=github_blue`: default, BlueROV2-style dengan gripper custom terintegrasi.
   - `rov_variant:=bluerov`: model lama dari referensi lokal untuk pembanding.
3. Memilih mode fisika:
   - `physics_mode:=kinematic`: mode default, ringan dan stabil.
   - `physics_mode:=hydro`: mode eksperimen memakai plugin buoyancy dan wrench.
4. Membaca template world:

```text
src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf
```

5. Mengganti placeholder:
   - `@PAYLOAD_CODE@`
   - `@ROV_MODEL_URI@`
   - `@LEFT_GRIPPER_JAW_URI@`
   - `@RIGHT_GRIPPER_JAW_URI@`
   - `@PHYSICS_STEP_SIZE@`
   - `@HYDRO_WORLD_PLUGINS@`
   - `@KINEMATIC_PROP_VISUALS@`
6. Menulis world hasil generate ke:

```text
/tmp/kki_rov_pool_<payload>_<mode>.sdf
```

7. Menjalankan Gazebo, bridge ROS-Gazebo, dan node-node ROS.

Keuntungan metode template ini adalah world tetap satu sumber utama, tetapi payload, model ROV, dan mode fisika bisa diganti dari launch tanpa membuat banyak file world terpisah.

## 3. World Kolam dan Arena Misi

World kolam berada di:

```text
src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf
```

Elemen utamanya:

- lantai kolam 10 m x 10 m,
- dinding kolam,
- permukaan air,
- volume air transparan,
- fog bawah air,
- ripple dan wavefield visual,
- bubble dan caustic lantai,
- hook A/B/C/D di empat sisi kolam,
- payload QR A/B/C/D,
- ROV dan gripper,
- visual wake dan thruster wash.

Metode air yang dipakai di world ini adalah visual representatif, bukan CFD. Air dibuat dengan:

- material transparan kebiruan,
- fog untuk memberi efek jarak pandang di air,
- caustic di lantai,
- bubble statis,
- ripple dan wavefield ringan,
- wake dan thruster wash yang digerakkan oleh node ROS.

Batas klaim:

- Cocok untuk menggambarkan arena bawah air, latihan kendali, QR, gripper, dan alur misi.
- Belum boleh diklaim sebagai model hidrodinamika asli sebelum dikalibrasi memakai data uji kolam.
- Efek air belum menghitung gelombang fluida 3D, turbulensi, atau tekanan fluida nyata.

## 4. Model ROV

Model default:

```text
src/rov_gamantaray_description/models/gamantaray_rov_github_blue_gripper/model.sdf
```

Model ini memakai bentuk BlueROV2-style dan gripper bawah-depan custom. Gripper dibuat menyatu dengan ROV melalui:

- saddle/mount ke rangka,
- housing aktuator,
- cheek plate kiri-kanan,
- pin pivot,
- rahang kiri dan kanan yang digerakkan terpisah.

ROV default di mode kinematic dibuat `static=true`. Alasannya:

- ROV digerakkan langsung oleh node ROS lewat service Gazebo `set_pose`.
- Simulasi lebih ringan dan real-time factor lebih stabil.
- Kontrol stik lebih responsif untuk latihan misi.

Konsekuensi:

- ROV tidak bergerak karena solver fisika murni.
- Gaya hidrodinamika penuh tidak dihitung di mode default.
- Collision ROV tetap ada untuk bentuk dan pengecekan, tetapi respons dorong payload perlu dibantu oleh `gripper_manager`.

Collision ROV meliputi:

- `body_collision`,
- `gripper_saddle_collision`,
- `gripper_neck_collision`,
- `gripper_left_cheek_collision`,
- `gripper_right_cheek_collision`,
- `gripper_cross_pin_collision`.

Collision capit kiri/kanan meliputi:

- `pivot_hub_collision`,
- `outer_finger_collision`,
- `inner_finger_collision`,
- `front_hook_tip_collision`,
- `rear_bridge_collision`.

## 5. Input Stik Xbox

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/joystick_driver.py
```

Topik output:

```text
/rov/cmd_vel
/rov/gripper_cmd
```

Metode yang dipakai:

1. Node membaca langsung device Linux `/dev/input/js0` memakai format event joystick.
2. Axis joystick dinormalisasi dari integer `-32767..32767` menjadi `-1.0..1.0`.
3. Deadzone diterapkan agar noise kecil tidak menggerakkan ROV.
4. Axis dipetakan menjadi:
   - surge/maju-mundur,
   - sway/kiri-kanan,
   - heave/naik-turun,
   - yaw/putar.
5. Nilai axis dikalikan skala:
   - `linear_scale`,
   - `vertical_scale`,
   - `yaw_scale`.
6. Tombol `A` default menutup gripper.
7. Tombol `B` default membuka gripper.

Karena pembacaan stik dilakukan dari `/dev/input/js0`, operator tidak perlu fokus ke terminal teleop. Gazebo boleh aktif di depan, stik tetap terbaca selama node joystick berjalan.

## 6. Keyboard Teleop

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/teleop_keyboard.py
```

Keyboard bersifat momentary. Artinya ROV bergerak hanya saat tombol ditekan/di-repeat oleh terminal. Kalau tombol dilepas, command otomatis nol setelah `key_hold_timeout_s`.

Mapping:

- `w/s`: maju/mundur,
- `a/d`: geser kiri/kanan,
- `r/f`: naik/turun,
- `q/e`: yaw kiri/kanan,
- `o`: buka gripper,
- `p`: tutup gripper,
- `space`: stop.

Batasan keyboard:

- Terminal teleop harus aktif/fokus.
- Kalau fokus pindah ke Gazebo, key event tidak masuk ke terminal.
- Untuk operasi lomba/simulasi serius, stik lebih cocok.

## 7. Allocator Thruster

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/thruster_allocator.py
```

Input:

```text
/rov/cmd_vel
```

Output:

```text
/rov/thruster1/cmd
/rov/thruster2/cmd
/rov/thruster3/cmd
/rov/thruster4/cmd
/rov/thruster5/cmd
/rov/thruster6/cmd
/rov/thruster_status
```

Metode allocator:

```text
surge = cmd.linear.x * max_horizontal
sway  = cmd.linear.y * max_horizontal
heave = cmd.linear.z * max_vertical
yaw   = cmd.angular.z * max_horizontal * yaw_scale
```

Empat thruster horizontal:

```text
t1 = surge - sway - yaw
t2 = surge + sway + yaw
t3 = surge + sway - yaw
t4 = surge - sway + yaw
```

Dua thruster vertikal:

```text
t5 = heave
t6 = heave
```

Normalisasi:

- Jika nilai thruster horizontal melebihi `max_horizontal`, semua thruster horizontal diskalakan supaya tidak lewat batas.
- Thruster vertikal dikunci di `-max_vertical..max_vertical`.
- Watchdog mengirim nol jika tidak ada command baru selama `command_timeout_s`.

Fungsi allocator ini bukan model elektrik motor detail. Ini mapping kontrol 6-DOF sederhana agar command operator menjadi nilai thruster yang konsisten.

## 8. Driver Gerak Kinematic

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/kinematic_driver.py
```

Mode ini adalah default.

Input:

```text
/rov/thruster_status
```

Output:

```text
/model/gamantaray_rov/odometry
service Gazebo /world/kki_rov_pool/set_pose
```

Metode:

1. Membaca enam nilai thruster.
2. Mengubah kembali thruster menjadi command gerak normalized:

```text
surge  = (t1 + t2 + t3 + t4) / (4 * max_horizontal)
sway   = (-t1 + t2 + t3 - t4) / (4 * max_horizontal)
yaw    = (-t1 + t2 - t3 + t4) / (4 * max_horizontal * yaw_scale)
heave  = (t5 + t6) / (2 * max_vertical)
```

3. Mengubah command menjadi target velocity:

```text
target_vx       = surge * max_xy_speed
target_vy       = sway * max_xy_speed
target_vz       = heave * max_z_speed
target_yaw_rate = yaw * max_yaw_rate
```

4. Memberi lag/damping memakai fungsi first-order response:

```text
alpha = 1 - exp(-dt / time_constant)
current = current + (target - current) * alpha
```

5. Mengintegrasikan pose:

```text
x += (body_vx * cos(yaw) - body_vy * sin(yaw)) * dt
y += (body_vx * sin(yaw) + body_vy * cos(yaw)) * dt
z += body_vz * dt
yaw += yaw_rate * dt
```

6. Membatasi posisi agar ROV tidak keluar kolam:

```text
x,y: -4.65..4.65
z: bottom_limit_z..surface_limit_z
```

7. Mengirim pose ROV ke Gazebo memakai `set_pose`.
8. Publish odometry untuk node lain.

Alasan memakai kinematic:

- Lebih stabil untuk misi dan training operator.
- Real-time factor lebih baik.
- Pergerakan bisa dibuat terasa seperti air lewat damping tanpa solver fisika berat.

Batasan:

- Bukan perhitungan gaya fluida penuh.
- Tidak menghitung added mass, vortex, turbulence, atau thrust curve motor asli.

## 9. Animasi Propeller

Masih di `kinematic_driver.py`.

Metode:

1. Setiap propeller adalah model visual terpisah.
2. Node menyimpan `prop_spin_angles`.
3. Nilai thruster dinormalisasi terhadap limit.
4. Sudut propeller diperbarui:

```text
prop_angle += normalized_thrust * prop_spin_gain * direction * dt
```

5. Pose propeller dihitung dari:

- pose ROV,
- offset lokal propeller,
- orientasi lokal propeller,
- spin angle,
- perkalian quaternion.

Tujuan:

- Memberi indikator visual bahwa thruster sedang aktif.
- Membuat propeller terlihat berputar di duct/ring.

Batasan:

- Propeller visual bukan sumber gaya fisika.
- Gaya gerak tetap berasal dari `thruster_allocator` dan driver gerak.

## 10. Mode Hydro Eksperimen

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/hydro_wrench_driver.py
```

Aktif dengan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py physics_mode:=hydro
```

Default `physics_mode:=hydro` sekarang memakai:

```text
hydro_control_mode:=kinematic
```

Alasannya praktis: world tetap menampilkan suasana bawah air, tetapi gerak ROV tetap responsif untuk latihan misi. Pada mode ini, `kinematic_driver` tetap membaca `/rov/thruster_status` dan menggerakkan pose ROV, sama seperti mode utama.

Mode wrench hydro murni diaktifkan dengan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py physics_mode:=hydro hydro_control_mode:=wrench
```

Metode:

1. Launch memasukkan plugin Gazebo:
   - `gz-sim-buoyancy-system`,
   - `gz-sim-apply-link-wrench-system`.
2. Node membaca `/rov/thruster_status`.
3. Thruster dikonversi menjadi wrench:

```text
body_fx = (t1 + t2 + t3 + t4) * horizontal_gain
body_fy = (-t1 + t2 + t3 - t4) * horizontal_gain
body_fz = (t5 + t6) * vertical_gain
body_tz = (-t1 + t2 - t3 + t4) * yaw_gain
```

4. Gaya body frame diubah ke world frame memakai yaw ROV.
5. Wrench dipublish ke:

```text
/world/kki_rov_pool/wrench/persistent
```

Entity wrench dipublish ke model:

```text
gamantaray_rov
```

Tipe entity:

```text
MODEL
```

Parameter gain hydro bisa dituning dari launch:

```text
hydro_horizontal_force_gain
hydro_vertical_force_gain
hydro_yaw_torque_gain
```

Jika analog stik sudah menghasilkan `/rov/thruster_status`, tetapi ROV hampir tidak maju pada `hydro_control_mode:=wrench`, penyebabnya ada di tuning gaya wrench, buoyancy, damping, dan hydrodynamics. Mode ini belum menjadi jalur latihan utama.

Batasan mode hydro:

- Masih eksperimen.
- Perlu tuning massa, buoyancy, damping, dan gain dengan data asli.
- Belum menjadi mode utama karena lebih berat dan bisa kurang stabil untuk latihan misi.

## 11. Gripper, Alignment, dan Attach Payload

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py
```

Input:

```text
/rov/gripper_cmd
/model/gamantaray_rov/odometry
/rov/qr_code
/world/kki_rov_pool/pose/info
```

Output:

```text
/rov/gripper_status
service Gazebo /world/kki_rov_pool/set_pose
```

Metode animasi gripper:

1. Command gripper `0.0` berarti buka, `1.0` berarti tutup.
2. Posisi gripper tidak berubah instan. Nilainya mengikuti response:

```text
gripper_position = approach(gripper_position, command, dt, gripper_response_s)
```

3. Sudut rahang dihitung dari interpolasi:

```text
jaw_angle = open_angle + (closed_angle - open_angle) * gripper_position
```

4. Pose rahang kiri/kanan dihitung dari pose ROV dan offset lokal.
5. Pose rahang dikirim ke Gazebo dengan `set_pose`.

Metode alignment:

1. Pose payload dibaca dari Gazebo.
2. Payload diubah dari world frame ke body frame ROV:

```text
dx = payload_x - rov_x
dy = payload_y - rov_y
local_x = dx * cos(yaw) + dy * sin(yaw)
local_y = -dx * sin(yaw) + dy * cos(yaw)
local_z = payload_z - rov_z
```

3. Error dihitung terhadap titik capture:

```text
err_x = local_x - capture_forward_offset
err_y = local_y
err_z = local_z - capture_vertical_offset
```

4. Payload dianggap sejajar jika error masih di dalam toleransi:

```text
abs(err_x) <= capture_forward_tolerance
abs(err_y) <= capture_lateral_tolerance
abs(err_z) <= capture_vertical_tolerance
```

5. Payload baru attached jika:

- gripper sedang diperintah menutup,
- posisi gripper sudah hampir tertutup,
- payload sejajar dengan titik capture,
- jaw gap sudah cukup kecil untuk menjepit payload.

Ini mencegah kasus lama: tombol `A` ditekan lalu payload langsung terambil walaupun posisi belum benar.

Saat attached:

1. Pose payload dipasang ke titik tengah rahang.
2. Model payload dipindahkan dengan `set_pose`.
3. `held_payload_collision_proxy` ikut dipindahkan agar collision payload yang sedang diangkat tetap terlihat/terwakili.

Saat release:

1. Command gripper dibuka.
2. `attached=false`.
3. Collision proxy disembunyikan ke `z=6`.
4. Payload tertinggal di pose terakhir.

## 12. Collision Payload dan Respons Dorong

Payload berada di:

```text
src/rov_gamantaray_gazebo/models/kki_payload_A/model.sdf
src/rov_gamantaray_gazebo/models/kki_payload_B/model.sdf
src/rov_gamantaray_gazebo/models/kki_payload_C/model.sdf
src/rov_gamantaray_gazebo/models/kki_payload_D/model.sdf
```

Setiap payload:

- `static=false`,
- punya massa 0.30 kg,
- punya inertia,
- punya `body_collision`,
- punya `qr_top_plate_collision`,
- punya friction dan contact stiffness.

Masalah teknis:

- ROV default digerakkan kinematic memakai `set_pose`.
- Objek yang digerakkan kinematic tidak selalu memberi respons kontak realistis ke payload kecil.
- Karena itu, collision geometry saja belum cukup untuk membuat payload pasti bergeser.

Solusi yang dipakai:

`gripper_manager` menambahkan kinematic contact response.

Logikanya:

1. Hitung posisi payload dalam body frame ROV.
2. Tentukan apakah payload masuk zona kontak body ROV:

```text
0.10 <= local_x <= 0.36
abs(local_y) <= 0.20
-0.30 <= local_z <= 0.12
```

3. Tentukan apakah payload masuk zona kontak capit:

```text
0.22 <= local_x <= 0.47
abs(local_y) <= 0.13
abs(local_z - capture_vertical_offset) <= 0.13
```

4. Cek apakah ROV sedang bergerak ke arah payload:

```text
vx * local_x + vy * local_y > 0
```

5. Jika kontak dan bergerak ke arah payload, payload digeser:

```text
push_x = vx * dt * payload_contact_push_gain
push_y = vy * dt * payload_contact_push_gain
```

6. Delta body frame diubah ke world frame.
7. Pose payload dikirim ke Gazebo.
8. Status publish:

```text
collision=true
contact=claw atau contact=rov
```

Makna metode ini:

- Collision payload memang ada secara SDF.
- Payload bisa bergeser ketika ditabrak ROV/capit.
- Respons kontak dibuat stabil untuk mode kinematic.
- Ini bukan solver kontak fluida penuh, tetapi cukup untuk misi simulasi dan latihan operator.

## 13. Deteksi QR Code

Node:

```text
src/rov_gamantaray_vision/rov_gamantaray_vision/qr_detector.py
```

Input:

```text
/rov/camera/bottom/image
```

Output:

```text
/rov/qr_code
/rov/qr_debug/image
```

Metode:

1. Image ROS dikonversi ke OpenCV memakai `cv_bridge`.
2. Detector yang dipakai:

```text
cv2.QRCodeDetector()
```

3. Algoritma mencoba multi QR lebih dulu:

```text
detectAndDecodeMulti(image)
```

4. Jika multi QR gagal, fallback ke single QR:

```text
detectAndDecode(image)
```

5. Jika berhasil, huruf QR dipublish ke `/rov/qr_code`.
6. Titik sudut QR digambar di debug image.

Batasan:

- Deteksi bergantung pada resolusi kamera, jarak, sudut pandang, pencahayaan, dan ukuran QR.
- Node belum menghitung pose 3D QR; saat ini hanya membaca isi kode.

## 14. Mission Supervisor

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/mission_supervisor.py
```

Aktif dengan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py mission_autonomy:=true use_vision:=true
```

State machine:

```text
scan_payload -> pick_payload -> go_to_hook -> release_payload -> surface
```

Target hook:

```text
A: (-4.45, 0.0, -0.45)
B: ( 4.45, 0.0, -0.45)
C: ( 0.0, 4.45, -0.45)
D: ( 0.0,-4.45, -0.45)
```

Metode navigasi:

1. Baca odometry ROV.
2. Hitung error posisi:

```text
ex = target_x - rov_x
ey = target_y - rov_y
ez = target_z - rov_z
```

3. Publish command proportional:

```text
cmd.linear.x = clamp(linear_gain * ex, -1.0, 1.0)
cmd.linear.y = clamp(linear_gain * ey, -1.0, 1.0)
cmd.linear.z = clamp(linear_gain * ez, -0.7, 0.7)
```

4. Jika jarak ke target kurang dari `position_tolerance_m`, pindah state.

Batasan:

- Ini baseline autonomous sederhana, bukan path planner penuh.
- Belum ada obstacle avoidance.
- Belum ada visual servoing QR.
- Cocok sebagai kerangka awal untuk dikembangkan.

## 15. Efek Air Dinamis

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/water_effects_driver.py
```

Model visual:

```text
src/rov_gamantaray_gazebo/models/rov_surface_wake
src/rov_gamantaray_gazebo/models/rov_thruster_wash
```

Metode:

1. Baca odometry ROV.
2. Hitung kecepatan horizontal:

```text
speed_xy = sqrt(vx^2 + vy^2)
```

3. Hitung kecepatan 3D:

```text
speed_3d = sqrt(vx^2 + vy^2 + vz^2)
```

4. Jika ROV dekat permukaan dan `speed_xy` cukup besar, tampilkan `rov_surface_wake` di permukaan air.
5. Jika ROV bergerak cukup cepat di bawah air, tampilkan `rov_thruster_wash` di sekitar ROV.
6. Jika tidak aktif, model disembunyikan ke `z=6`.

Efek ini visual-only. Ia tidak memberi gaya balik ke ROV dan tidak mengubah fisika fluida.

## 16. Kamera dan Sensor

Di model ROV terdapat:

- kamera depan/wall camera untuk melihat arah depan,
- kamera bawah/bottom camera untuk QR,
- IMU.

Bridge ROS-Gazebo mengirim:

```text
/rov/camera/wall/image
/rov/camera/wall/camera_info
/rov/camera/bottom/image
/rov/camera/bottom/camera_info
/rov/imu
```

Kamera bisa dilihat dengan:

```bash
ros2 run rqt_image_view rqt_image_view /rov/camera/bottom/image
ros2 run rqt_image_view rqt_image_view /rov/camera/wall/image
```

## 17. Topik Penting

Kontrol:

```text
/rov/cmd_vel
/rov/gripper_cmd
```

Thruster:

```text
/rov/thruster1/cmd
/rov/thruster2/cmd
/rov/thruster3/cmd
/rov/thruster4/cmd
/rov/thruster5/cmd
/rov/thruster6/cmd
/rov/thruster_status
```

State dan status:

```text
/model/gamantaray_rov/odometry
/rov/gripper_status
/rov/mission_state
```

Vision:

```text
/rov/camera/bottom/image
/rov/camera/wall/image
/rov/qr_code
/rov/qr_debug/image
```

## 18. Parameter yang Paling Sering Dituning

Stik:

- `joystick_axis_surge`
- `joystick_axis_sway`
- `joystick_axis_heave`
- `joystick_axis_yaw`
- `joystick_deadzone`
- `joystick_linear_scale`
- `joystick_vertical_scale`
- `joystick_yaw_scale`

Gerak kinematic:

- `max_xy_speed_mps`
- `max_z_speed_mps`
- `max_yaw_rate_rps`
- `linear_response_s`
- `vertical_response_s`
- `yaw_response_s`
- `attitude_response_s`
- `bottom_limit_z`
- `surface_limit_z`

Gripper:

- `capture_forward_offset_m`
- `capture_vertical_offset_m`
- `capture_forward_tolerance_m`
- `capture_lateral_tolerance_m`
- `capture_vertical_tolerance_m`
- `closed_gap_m`
- `open_gap_m`
- `payload_contact_push_gain`

Vision:

- `image_topic`
- `debug_image_topic`

## 19. Validasi yang Sudah Masuk Akal

Validasi workspace yang bisa dilakukan:

```bash
xmllint --noout src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/*.py
colcon build --symlink-install
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false
```

Untuk cek gripper dan collision:

```bash
ros2 topic echo /rov/gripper_status
```

Field penting:

- `aligned=true/false`: payload sudah sejajar atau belum.
- `collision=true/false`: payload sedang menyentuh zona body/capit atau tidak.
- `contact=claw`: kontak dari capit.
- `contact=rov`: kontak dari body ROV.
- `err=(x,y,z)`: error payload terhadap titik capture gripper.

## 20. Batas Klaim untuk Laporan

Kalimat aman:

```text
Simulasi ini merepresentasikan alur misi ROV KKI di Gazebo: kendali ROV, kamera bawah untuk QR, mekanisme gripper, collision payload, dan pemindahan payload ke hook. Mode default memakai model kinematic dengan damping agar real-time dan stabil untuk latihan operator. Visual air dibuat representatif bawah air, sedangkan klaim hidrodinamika penuh memerlukan kalibrasi tambahan menggunakan data uji kolam dan parameter fisik ROV asli.
```

Yang boleh diklaim:

- Workspace sesuai untuk simulasi misi KKI.
- ROV bisa digerakkan dengan stik.
- Payload QR bisa dibaca kamera bawah.
- Gripper tidak mengambil payload kalau belum sejajar.
- Payload bisa bergeser saat tersentuh ROV/capit.
- Visual air menggambarkan kondisi bawah air.

Yang belum boleh diklaim tanpa data uji:

- Gaya thruster sama dengan ROV asli.
- Drag air sama dengan dunia nyata.
- Buoyancy dan massa sudah identik dengan hardware.
- Efek gelombang/wake sama dengan fluida asli.
- Collision kecil sama persis dengan eksperimen kolam.

## 21. Cara Mengembangkan Workspace

Untuk model ROV sendiri:

1. Buat folder model baru di `src/rov_gamantaray_description/models`.
2. Buat `model.config` dan `model.sdf`.
3. Tambahkan body, thruster, kamera, lampu, gripper, collision, dan inertial.
4. Tambahkan opsi `rov_variant` baru di launch.
5. Sesuaikan offset propeller di `kinematic_driver.py`.
6. Sesuaikan offset gripper di `gripper_manager.py`.

Untuk gripper lebih realistis:

1. Ganti jaw visual menjadi mesh CAD sendiri.
2. Tambahkan joint fisika bila ingin mode dynamic penuh.
3. Tambahkan sensor kontak jika ingin attach berbasis contact event.
4. Kalibrasi toleransi alignment berdasarkan ukuran payload asli.

Untuk QR lebih kuat:

1. Tambahkan pre-processing image seperti threshold/adaptive threshold.
2. Tambahkan filter hasil QR agar tidak berubah cepat.
3. Tambahkan estimasi posisi QR dari titik sudut QR.
4. Pakai visual servoing untuk mendekati payload otomatis.

Untuk hidrodinamika lebih kuat:

1. Ukur massa, volume, titik apung, dan pusat massa ROV asli.
2. Masukkan inertia yang lebih benar ke SDF.
3. Tune buoyancy dan damping.
4. Validasi kecepatan ROV di kolam nyata.
5. Baru klaim kecocokan fisika terhadap hardware.

## 22. Kesimpulan

Workspace ini memakai pendekatan pragmatis:

- mode kinematic sebagai baseline yang stabil dan mudah dijalankan,
- allocator thruster sederhana untuk mapping kontrol,
- damping first-order agar gerak terasa seperti ROV di air,
- gripper alignment agar payload tidak terambil sembarangan,
- contact response kinematic agar payload bisa bergeser saat collision,
- QR detection OpenCV sebagai baseline vision,
- mission supervisor proportional sebagai baseline autonomous,
- visual air representatif untuk konteks bawah air.

Pendekatan ini cocok untuk latihan misi, pengembangan software, dan demonstrasi alur KKI. Untuk klaim fisika dunia nyata, workspace perlu dikalibrasi lagi dengan data ROV asli dan uji kolam.
