# Metode dan Logika Workspace ROV KKI

Dokumen ini menjelaskan metode, algoritma, dan alur kerja yang dipakai di workspace `WS_ROV`. Tujuannya adalah membuat simulasi ROV bawah air untuk latihan misi KKI: bergerak di kolam, membaca QR Code, mengambil payload, dan memindahkan payload ke gantungan A/B/C/D.

## 1. Ringkasan Arsitektur

Workspace ini memakai ROS 2 Jazzy dan Gazebo Harmonic. Simulasi dibagi menjadi beberapa package:

- `rov_gamantaray_bringup`: launch utama dan konfigurasi runtime.
- `rov_gamantaray_description`: model ROV, gripper, propeller, sensor, dan collision ROV.
- `rov_gamantaray_gazebo`: world kolam, payload QR, hook A/B/C/D, air, wake, dan collision proxy.
- `rov_gamantaray_control`: input stik/keyboard, allocator thruster, driver gerak, gripper, collision response, efek air, dan misi sederhana.
- `rov_gamantaray_vision`: deteksi QR Code dari kamera depan/wall.

Alur data utama:

```text
stik / keyboard
  -> /rov/manual_cmd_vel
mission_supervisor
  -> /rov/auto_cmd_vel
cmd_vel_mux
  -> /rov/cmd_vel
  -> thruster_allocator
  -> /rov/thruster_pwm
  -> estimasi thrust /rov/thruster_status
  -> kinematic_driver atau hydro_wrench_driver
  -> Gazebo ROV bergerak
  -> /model/gamantaray_rov/odometry
  -> gripper_manager, water_effects_driver, mission_supervisor
```

Alur payload:

```text
kamera depan/wall
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
   - `rov_variant:=github_blue_joint`: BlueROV2-style stabil dengan gripper compact menyatu, tetapi rahang tetap digerakkan kinematic agar aman untuk latihan misi.
   - `rov_variant:=github_blue_joint_experimental`: BlueROV2-style dengan rahang gripper sebagai link fisika dan revolute joint.
   - `rov_variant:=beaumont`: model Beaumont dari referensi lama untuk pembanding.
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

Varian joint fisika eksperimen:

```text
src/rov_gamantaray_description/models/gamantaray_rov_github_blue_joint_gripper/model.sdf
```

Pada varian ini, rahang kiri dan kanan bukan model visual terpisah. Keduanya menjadi link:

```text
left_gripper_jaw_link
right_gripper_jaw_link
```

Keduanya terhubung ke `base_link` dengan:

```text
left_gripper_hinge
right_gripper_hinge
```

Joint digerakkan oleh plugin Gazebo:

```text
gz-sim-joint-position-controller-system
```

`gripper_manager` tetap menerima `/rov/gripper_cmd`, tetapi pada mode `gripper_actuation_mode:=joint` node tidak memanggil `set_pose` untuk rahang. Node mempublish target posisi joint ke topic Gazebo:

```text
/model/gamantaray_rov/joint/left_gripper_hinge/0/cmd_pos
/model/gamantaray_rov/joint/right_gripper_hinge/0/cmd_pos
```

Dengan cara ini, rahang punya collision link sendiri dan kontak dengan payload dapat dihitung oleh contact solver Gazebo. Batasnya: gerak translasi ROV secara keseluruhan masih memakai driver kinematic/PWM agar latihan joystick tetap stabil. Karena dua pendekatan ini dicampur, varian ini diberi nama `rov_variant:=github_blue_joint_experimental` dan bukan varian utama untuk latihan misi. Command lama `rov_variant:=github_blue_joint` sekarang diarahkan ke model compact stabil supaya gripper tidak terlihat lepas atau offset di GUI.

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
- `inner_pad_collision`,
- `front_hook_tip_collision`,
- `rear_link_collision`.

## 5. Input Stik Xbox

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/joystick_driver.py
```

Topik output:

```text
/rov/manual_cmd_vel
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
/rov/thruster_pwm
/rov/thruster1/cmd
/rov/thruster2/cmd
/rov/thruster3/cmd
/rov/thruster4/cmd
/rov/thruster5/cmd
/rov/thruster6/cmd
/rov/thruster1/pwm
/rov/thruster2/pwm
/rov/thruster3/pwm
/rov/thruster4/pwm
/rov/thruster5/pwm
/rov/thruster6/pwm
/rov/thruster_status
```

Metode allocator:

```text
surge = clamp(cmd.linear.x, -1, 1)
sway  = clamp(cmd.linear.y, -1, 1)
heave = clamp(cmd.linear.z, -1, 1)
yaw   = clamp(cmd.angular.z, -1, 1) * yaw_scale
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

- Jika nilai horizontal melebihi `-1..1`, semua thruster horizontal diskalakan supaya tidak lewat batas.
- Thruster vertikal dikunci di `-1..1`.
- Watchdog mengirim nol jika tidak ada command baru selama `command_timeout_s`.

Setelah mixing, nilai normalized tidak langsung menggerakkan ROV. Nilai itu dikonversi menjadi PWM ESC:

```text
pwm = 1500 us + command * range
range maju  = pwm_max_us - pwm_neutral_us
range mundur = pwm_neutral_us - pwm_min_us
```

Default:

```text
pwm_neutral_us = 1500
pwm_min_us     = 1100
pwm_max_us     = 1900
pwm_deadband_us = 25
```

Estimasi thrust dihitung dari PWM dengan deadband dan kurva kuadratik sederhana:

```text
normalized_pwm = 0 jika |pwm - 1500| <= deadband
thrust_n = max_thrust_n * normalized_pwm * abs(normalized_pwm)
```

Artinya di workspace ini alur geraknya sudah:

```text
/rov/cmd_vel -> mixer thruster -> PWM ESC -> estimasi thrust -> driver gerak ROV
```

`/rov/thruster_pwm` adalah enam nilai PWM dalam microsecond. `/rov/thruster_status` adalah estimasi gaya Newton hasil konversi PWM. Fungsi allocator ini belum model elektrik motor detail seperti data pabrikan T200 lengkap, tetapi sudah lebih mendekati jalur asli daripada menggerakkan ROV langsung dari `cmd_vel`.

## 8. Driver Gerak Kinematic

Node:

```text
src/rov_gamantaray_control/rov_gamantaray_control/kinematic_driver.py
```

Mode ini adalah default.

Input:

```text
/rov/thruster_pwm
```

Output:

```text
/model/gamantaray_rov/odometry
service Gazebo /world/kki_rov_pool/set_pose
```

Metode:

1. Membaca enam nilai PWM dari `/rov/thruster_pwm`.
2. Mengubah PWM menjadi estimasi gaya thruster memakai model yang sama dengan allocator.
3. Mengubah kembali thrust menjadi command gerak normalized:

```text
surge  = (t1 + t2 + t3 + t4) / (4 * max_horizontal)
sway   = (-t1 + t2 + t3 - t4) / (4 * max_horizontal)
yaw    = (-t1 + t2 - t3 + t4) / (4 * max_horizontal * yaw_scale)
heave  = (t5 + t6) / (2 * max_vertical)
```

4. Mengubah command menjadi target velocity:

```text
target_vx       = surge * max_xy_speed
target_vy       = sway * max_xy_speed
target_vz       = heave * max_z_speed
target_yaw_rate = yaw * max_yaw_rate
```

5. Memberi lag/damping memakai fungsi first-order response:

```text
alpha = 1 - exp(-dt / time_constant)
current = current + (target - current) * alpha
```

6. Mengintegrasikan pose:

```text
x += (body_vx * cos(yaw) - body_vy * sin(yaw)) * dt
y += (body_vx * sin(yaw) + body_vy * cos(yaw)) * dt
z += body_vz * dt
yaw += yaw_rate * dt
```

7. Membatasi posisi agar ROV tidak keluar kolam:

```text
x,y: -4.65..4.65
z: bottom_limit_z..surface_limit_z
```

8. Mengirim pose ROV ke Gazebo memakai `set_pose`.
9. Publish odometry untuk node lain.

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

Alasannya praktis: world tetap menampilkan suasana bawah air, tetapi gerak ROV tetap responsif untuk latihan misi. Pada mode ini, `kinematic_driver` tetap membaca `/rov/thruster_pwm`, mengubahnya menjadi estimasi thrust, lalu menggerakkan pose ROV.

Mode wrench hydro diaktifkan dengan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py physics_mode:=hydro hydro_control_mode:=wrench
```

Metode:

1. Launch memasukkan plugin Gazebo:
   - `gz-sim-buoyancy-system`,
   - `gz-sim-apply-link-wrench-system`.
2. Node membaca `/rov/thruster_pwm`.
3. PWM dikonversi menjadi estimasi thrust.
4. Thruster dikonversi menjadi wrench:

```text
body_fx = (t1 + t2 + t3 + t4) * horizontal_gain
body_fy = (-t1 + t2 + t3 - t4) * horizontal_gain
body_fz = (t5 + t6) * vertical_gain
body_tz = (-t1 + t2 - t3 + t4) * yaw_gain
```

5. Gaya body frame diubah ke world frame memakai yaw ROV.
6. Wrench dipublish ke:

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

Jika analog stik sudah menghasilkan `/rov/thruster_pwm`, tetapi ROV hampir tidak maju pada `hydro_control_mode:=wrench`, penyebabnya ada di tuning gaya wrench, buoyancy, damping, dan hydrodynamics. Mode ini belum menjadi jalur latihan utama.

Supaya mode `wrench` tetap bisa dilihat bergerak sebelum kalibrasi selesai, `hydro_wrench_driver` juga punya `pose_assist_enabled=true` secara default. Pose assist membaca PWM yang sama, menghitung estimasi thrust dan kecepatan surge/sway/heave/yaw dengan damping, lalu mengirim pose ROV ke service Gazebo:

```text
/world/kki_rov_pool/set_pose
```

Jadi mode `wrench` saat ini adalah mode eksperimen gaya + pose assist, bukan klaim hidrodinamika penuh yang sudah tervalidasi.

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

Geometri capit aktif dibuat compact supaya proporsional dengan body BlueROV:

```text
jaw_pivot_x_m       = 0.244
jaw_pivot_y_m       = 0.050
jaw_open_angle_rad  = 0.38
open_gap_m          = 0.125
closed_gap_m        = 0.050
mouth_clearance_m   = 0.006
```

Mount di body ROV dibuat lebih sempit daripada versi lebar sebelumnya: saddle, cheek plate, cross pin, side rail, dan pivot cap mengikuti posisi pivot compact. Tujuannya supaya capit terlihat sebagai mekanik yang menyatu dengan rangka ROV, tetapi tidak terlalu besar secara visual.

Visual body ROV juga ditambah bridge, side rail, front yoke, actuator cylinder, dan pushrod. Bagian ini tidak bergerak seperti sendi, tetapi menjadi struktur tetap yang menghubungkan rahang animasi ke rangka ROV.

Saat payload berada di antara rahang, node memakai jaw-stop virtual:

```text
desired_gap = payload_width + 2 * mouth_clearance
effective_gripper_position = min(gripper_position, position_for_desired_gap)
```

`effective_gripper_position` dipakai untuk visual rahang dan perhitungan `gap`. Jadi walaupun tombol tutup ditahan sampai command `1.0`, visual rahang tidak menutup melewati payload. Selain limit gap, node juga melakukan binary search kecil terhadap overlap `inner_pad_collision` kiri/kanan. Jika salah satu pad mulai menembus payload, posisi visual rahang dikurangi sampai penetrasi visual berada di bawah `jaw_visual_stop_clearance_m`. Ini penting karena rahang digerakkan kinematic dengan `set_pose`, bukan sendi Gazebo yang otomatis tertahan oleh collision solver.

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
- alignment bertahan minimal `capture_alignment_hold_s`, default 0.15 s,
- jaw gap sudah cukup kecil untuk menjepit payload.

Ini mencegah kasus lama: tombol `A` ditekan lalu payload langsung terambil walaupun posisi belum benar.

Saat attached:

1. Target pose payload dihitung dari titik tengah rahang.
2. Payload mengikuti target dengan constraint lunak:

```text
payload_pose = approach(payload_pose, held_target_pose, dt, held_payload_response_s)
```

3. Jarak lag payload dibatasi oleh `held_payload_max_sway_m`, sehingga payload masih terlihat sedikit tertarik/tertinggal saat ROV bergerak tetapi tidak lepas liar.
4. Field `grip_stress` dihitung dari rasio lag terhadap `held_payload_max_sway_m`.
5. Jika target gripper berada di bawah `payload_floor_z`, attach ditolak; saat payload sudah attached, posisi z payload tetap di-clamp minimal `payload_floor_z`. Ini mencegah payload tenggelam ke lantai ketika ROV terlalu rendah saat mencengkeram.
6. Model payload dipindahkan dengan `set_pose`.
7. `held_payload_collision_proxy` ikut dipindahkan agar collision payload yang sedang diangkat tetap terlihat/terwakili.

Saat release manual/autonomous:

1. Command gripper dibuka.
2. Jika QR belum diketahui, hook target tetap `unknown` dan payload tidak bisa dianggap valid masuk hook.
3. Jika QR sudah diketahui, node menghitung posisi lubang payload terhadap pasak hook.
4. Payload baru masuk mode `hung` jika empat syarat terpenuhi:

```text
radial_error_to_peg_axis <= hook_snap_hole_tolerance_m   # default 0.024 m
axial_error_along_peg <= hook_snap_axial_tolerance_m     # default 0.026 m
```

5. Jika syarat terpenuhi, titik gantung disimpan pada titik terdekat di sumbu pasak, bukan selalu di tengah pasak. Ini membuat payload terlihat menggantung pada lokasi masuknya lubang.
6. Jika syarat tidak terpenuhi, `attached=false`, collision proxy disembunyikan ke `z=6`, dan payload masuk mode `dropping`. Ini sengaja dibuat supaya release yang tidak sejajar tidak otomatis dianggap berhasil.

Mode `dropping`:

1. Payload diberi kecepatan awal dari gerak ROV saat melepas.
2. Node menghitung gaya berat efektif:

```text
net_gravity = -9.81 * (1 - payload_drop_buoyancy_ratio)
```

3. Drag air dihitung dari komponen linear dan kuadratik:

```text
a_drag = -linear_drag * v - quadratic_drag * v * abs(v)
```

4. Kecepatan turun dibatasi oleh `payload_drop_terminal_speed_mps`.
5. Payload diberi tilt kecil mengikuti arah gerak supaya tenggelamnya tidak terlihat seperti teleport.
6. Saat menyentuh `payload_floor_z`, velocity di-nol-kan dan payload kembali bisa didorong/diambil dari dasar.

Parameter default:

```text
payload_drop_buoyancy_ratio = 0.72
payload_drop_linear_drag = 4.2
payload_drop_terminal_speed_mps = 0.22
```

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
- mengikuti geometri PDF: plate 5 cm x 10 cm x 0.6 cm, QR 4 cm x 4 cm di sisi depan, base 3 cm, dan lubang gantung di atas QR,
- punya massa 0.20 kg,
- punya inertia,
- punya collision plate yang dipecah menjadi lower plate, top plate, sisi kiri/kanan lubang, dan base foot,
- punya friction dan contact stiffness,
- punya `velocity_decay` supaya gerak payload cepat teredam,
- punya `allow_auto_disable=true` supaya payload bisa sleep saat sudah diam.

Alasan massa 0.20 kg: ukuran payload PDF kecil, tetapi massanya tetap dibuat lebih besar daripada gaya apung dari volume collision kecilnya agar payload tidak naik-turun sendiri di mode hydro. Nilai ini masih cukup ringan untuk digeser oleh respons kontak kinematic saat ROV/capit menabrak.

Lubang gantung dibuat secara collision dan visual dengan menyisakan bukaan sekitar 3.4 cm pada bagian atas plate. Angka ini sedikit lebih besar dari diameter nominal 3 cm agar peg hook diameter 2 cm punya clearance di Gazebo. Visual hitam penutup lubang sudah dihapus; area lubang benar-benar kosong sehingga yang terlihat adalah air/objek di belakang payload, bukan disk hitam. Hook A/B/C/D berbentuk gantungan PVC dinding: backing ke dinding, clamp di bibir kolam, pipa vertikal, elbow bawah, dan peg horizontal terbuka diameter 2 cm. Ujung cone/lancip dan stopper silinder dihapus supaya lubang payload bisa masuk dari ujung peg. Semua bagian utama punya collision SDF, sehingga objek yang dilepas salah posisi akan menabrak bagian hook, bukan dianggap berhasil otomatis.

Masalah teknis:

- ROV default digerakkan kinematic memakai `set_pose`.
- Objek yang digerakkan kinematic tidak selalu memberi respons kontak realistis ke payload kecil.
- Karena itu, collision geometry saja belum cukup untuk membuat ROV/payload pasti tertahan.
- `kinematic_driver` menambahkan `hook_collision_guard` untuk menahan pose ROV di sekitar pipa vertikal dan peg horizontal.
- `gripper_manager` menambahkan guard kecil untuk payload terhadap pipa vertikal, sementara area peg tetap dibiarkan masuk lewat lubang agar proses menggantung masih bisa terjadi.

Solusi yang dipakai:

`gripper_manager` menambahkan kinematic contact response dengan default `payload_contact_model:=strict`.

Logikanya:

1. Hitung posisi payload dalam body frame ROV.
2. Hitung sudut rahang dari posisi visual efektif gripper. Ini penting supaya kontak tidak dihitung dari posisi raw ketika visual rahang sedang dibatasi oleh payload.
3. Hitung posisi box `inner_pad_collision` kiri dan kanan sesuai geometri SDF aktif:

```text
left_pad_center  = pivot_left  + rotate(jaw_angle)  * (0.105, -0.034)
right_pad_center = pivot_right + rotate(-jaw_angle) * (0.105,  0.034)
```

4. Ubah posisi payload ke frame pad yang sedang berputar, lalu hitung overlap box payload terhadap box pad:

```text
overlap_x = pad_half_x + projected_payload_half_x - abs(payload_x_in_pad)
overlap_y = pad_half_y + projected_payload_half_y - abs(payload_y_in_pad)
overlap_z = pad_half_z + payload_half_height - abs(payload_z_error)
```

Kontak valid hanya jika `overlap_x`, `overlap_y`, dan `overlap_z` sama-sama positif.

5. Jika hanya satu pad menyentuh, payload digeser keluar dari pad tersebut. Ini mensimulasikan tabrakan satu sisi, bukan pengambilan.
6. Jika dua pad menyentuh bersamaan, status menjadi:

```text
collision=true
contact=bilateral_clamp
pinched=true
```

7. Payload baru berubah menjadi `attached` jika `pinched=true`, gap rahang sudah mendekati lebar payload, kedalaman dan tinggi payload benar, dan kontak dua sisi bertahan minimal `grip_min_bilateral_contact_s`.

Pada kontak dua sisi, payload tidak dipindah otomatis ke tengah rahang. Koreksi lateral dihitung sebagai resolusi tabrakan keluar dari pad yang benar-benar overlap:

```text
dy += direction_away_from_left_pad  * left_overlap_y  * payload_contact_resolution_gain
dy += direction_away_from_right_pad * right_overlap_y * payload_contact_resolution_gain
dy = clamp(dy, -payload_contact_max_step_m, payload_contact_max_step_m)
```

Artinya payload hanya bergerak sedikit untuk keluar dari overlap yang lebih besar. Kalau operator datang dari kanan atau kiri, payload tidak tiba-tiba loncat ke `local_y = 0`.

Mode lama `payload_contact_model:=assisted` masih tersedia untuk debugging, tetapi bukan default karena memakai funnel yang menuntun payload ke tengah.

8. Tentukan apakah payload masuk zona kontak body ROV:

```text
0.04 <= local_x <= 0.19
abs(local_y) <= 0.19
-0.30 <= local_z <= 0.12
```

8. Tentukan apakah payload masuk zona kontak frame/front tip capit:

```text
self.jaw_pivot_x - 0.040 <= local_x <= self.jaw_pivot_x + 0.060
0.040 <= abs(local_y) <= 0.070
abs(local_z - capture_vertical_offset) <= 0.060
```

9. Cek apakah ROV sedang bergerak ke arah payload:

```text
vx * local_x + vy * local_y > 0
```

10. Jika kontak body/frame/tip dan bergerak ke arah payload, payload digeser:

```text
push_x = vx * dt * payload_contact_push_gain
push_y = vy * dt * payload_contact_push_gain
```

11. Delta body frame diubah ke world frame.
12. Pose payload dikirim ke Gazebo.
13. Status publish:

```text
collision=true
contact=rov_body, gripper_frame, front_tip, left_pad, right_pad, atau bilateral_clamp
```

Makna metode ini:

- Collision payload memang ada secara SDF.
- Payload bisa bergeser ketika ditabrak ROV/capit.
- Respons kontak dibuat stabil untuk mode kinematic.
- Tidak ada attach otomatis hanya karena payload dekat dengan capit.
- Attach hanya terjadi dari penjepitan dua sisi yang memenuhi syarat.
- Ini bukan solver kontak fluida penuh, tetapi cukup untuk misi simulasi dan latihan operator.

Saat payload sedang dijepit lalu gripper dibuka di dekat hook yang sesuai QR, `gripper_manager` mengaktifkan constraint gantung kinematic. Titik pivot constraint adalah pusat pasak hook, sedangkan titik pada payload yang dikunci adalah lubang gantung di atas QR. Dengan cara ini, lubang payload tetap berada pada pasak.

Setelah masuk mode gantung, body payload tidak hanya ditempel diam. Node menghitung ayunan teredam:

```text
theta_ddot = -(g / L) * sin(theta) - damping * theta_dot
```

Keterangan:

- `theta`: sudut ayunan payload terhadap posisi vertikal,
- `L`: panjang efektif ayunan,
- `damping`: redaman bawah air,
- `theta_dot`: kecepatan sudut awal yang diambil dari kecepatan ROV saat melepas payload.

Pose payload dihitung ulang dari pivot hook dan offset lubang payload:

```text
posisi_pusat_payload = posisi_pivot_hook - rotasi_payload * offset_lubang
```

Artinya lubang tetap sejajar dengan pasak, sedangkan badan payload bisa berayun kecil lalu mereda. Ini lebih realistis daripada snap pose satu kali, tetapi tetap disebut constraint kinematic karena belum membuat joint fisika dinamis Gazebo yang benar-benar baru saat runtime.

## 13. Deteksi QR Code

Node:

```text
src/rov_gamantaray_vision/rov_gamantaray_vision/qr_detector.py
```

Input:

```text
/rov/camera/wall/image
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

5. Hasil dibatasi ke kode payload valid A/B/C/D.
6. Jika beberapa QR terlihat, detector memilih satu QR dengan skor area visual terbesar dan posisi paling dekat tengah kamera.
7. Jika berhasil, huruf QR dipublish ke `/rov/qr_code`.
8. Titik sudut QR digambar di debug image. QR yang dipilih diberi garis hijau, sedangkan kandidat lain berwarna kuning.

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
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py mission_autonomy:=true mission_profile:=full use_vision:=true
```

Profil autonomous:

```text
full                   -> eksperimen end-to-end dari scan sampai surface
release_surface        -> misi nomor 5: payload sudah dekat hook, release, surface
carry_release_surface  -> eksperimen: payload sudah dijepit, lanjut ke hook, release, surface
```

Keputusan target payload dipisah dari konfigurasi world:

- `payload_code` di launch memilih model payload yang muncul di Gazebo untuk skenario uji.
- `/rov/qr_code` adalah sumber keputusan target autonomous.
- `require_qr_for_target:=true` adalah default, sehingga target tetap `unknown` sampai QR A/B/C/D terbaca.
- `use_default_payload_after_scan_timeout:=false` adalah default, sehingga sistem tidak otomatis memakai `payload_code` kalau QR gagal.

Untuk misi nomor 5 sesuai kutipan PDF, profil utama adalah `release_surface`. Operator membawa payload secara manual sampai dekat hook. Autonomous hanya melakukan `release_payload`, menunggu status `hung`, lalu `surface`.

`carry_release_surface` tetap ada sebagai mode eksperimen untuk latihan navigasi ke hook, tetapi jangan dipakai sebagai klaim utama misi nomor 5 jika aturan menilai autonomous hanya pada pelepasan payload dan naik ke permukaan.

Metode `go_to_hook` memakai tahap berikut:

```text
rise_to_transit -> go_to_standoff -> approach_hook
```

- `rise_to_transit`: ROV menjaga kedalaman transit dan mulai menghadap dinding hook.
- `go_to_standoff`: ROV bergerak ke titik aman di depan hook, belum langsung menabrak gantungan.
- `approach_hook`: ROV mendekat pelan ke gantungan dengan `approach_speed_scale`.

Kontrol posisi sekarang dihitung dalam body-frame ROV. Error world `(ex, ey)` diputar menggunakan yaw ROV menjadi `(body_x, body_y)`, lalu dipublish ke `/rov/auto_cmd_vel`. Ini lebih benar daripada langsung memakai error world sebagai command body, karena ROV bisa saja sedang menghadap arah lain.

State machine `full`:

```text
scan_payload -> pick_payload -> go_to_hook -> release_payload -> surface
```

State machine misi nomor 5 langsung:

```text
release_payload -> surface -> complete
```

Target hook:

```text
A: (-4.36, 0.0, -0.31)
B: ( 4.36, 0.0, -0.31)
C: ( 0.0, 4.36, -0.31)
D: ( 0.0,-4.36, -0.31)
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

Logika tambahan untuk misi nomor 5:

1. Jika `command_source:=manual`, `mission_supervisor` menunggu `/rov/active_command_source` berubah menjadi `auto`.
2. Jika QR belum valid, `release_payload` menahan gripper tetap tertutup saat payload masih `attached`.
3. Setelah auto aktif dan QR valid, `release_payload` membaca `hook_aligned` dari `/rov/gripper_status`.
4. Jika `hook_aligned=false`, gripper tetap ditahan tertutup supaya payload tidak jatuh sebelum lubang masuk area pasak.
5. Jika `hook_aligned=true`, `release_payload` mengirim `/rov/gripper_cmd = 0.0`.
6. `gripper_manager` mencoba memasukkan payload ke mode `hung` hanya jika lubang payload sejajar dengan pasak hook, ROV dekat pose release, dan yaw ROV sesuai.
7. `mission_supervisor` membaca `/rov/gripper_status`.
8. Jika status gripper menjadi `hung`, autonomous lanjut ke `surface`.
9. Jika release belum `hung`, ROV menahan posisi dan tidak langsung naik karena `release_surface_on_timeout:=false` secara default.
10. Pada `surface`, ROV menuju titik di atas hook dengan `z = surface_z`.
11. Setelah target permukaan tercapai, state menjadi `complete` dan command gerak dibuat nol.

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
- kamera depan/wall camera juga menjadi kamera utama untuk QR samping payload,
- kamera bawah/bottom camera untuk observasi lantai/dasar kolam,
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
/rov/manual_cmd_vel
/rov/auto_cmd_vel
/rov/command_source
/rov/active_command_source
/rov/cmd_vel
/rov/gripper_cmd
```

Thruster:

```text
/rov/thruster_pwm
/rov/thruster1/pwm
/rov/thruster2/pwm
/rov/thruster3/pwm
/rov/thruster4/pwm
/rov/thruster5/pwm
/rov/thruster6/pwm
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
/rov/camera/wall/image   -> input default QR samping
/rov/camera/bottom/image -> observasi bawah
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
- `hook_collision_guard_enabled`
- `hook_collision_body_radius_m`
- `hook_collision_margin_m`
- `hook_collision_body_half_height_m`
- `hook_collision_velocity_damping`

Gripper:

- `jaw_pivot_y_m`
- `jaw_open_angle_rad`
- `mouth_clearance_m`
- `payload_contact_model` (`strict` default, `assisted` untuk debug lama)
- `payload_contact_resolution_gain`
- `payload_contact_max_step_m`
- `grip_min_bilateral_contact_s`
- `grip_clamp_margin_m`
- `capture_forward_offset_m`
- `capture_vertical_offset_m`
- `capture_forward_tolerance_m`
- `capture_lateral_tolerance_m`
- `capture_vertical_tolerance_m`
- `capture_alignment_hold_s`
- `held_payload_response_s`
- `held_payload_max_sway_m`
- `closed_gap_m`
- `open_gap_m`
- `payload_drop_buoyancy_ratio`
- `payload_drop_linear_drag`
- `payload_drop_terminal_speed_mps`
- `open_gap_m`
- `payload_contact_push_gain`
- `hook_snap_rov_tolerance_m`
- `hook_snap_hole_tolerance_m`
- `hook_snap_axial_tolerance_m`
- `hook_snap_yaw_tolerance_rad`
- `release_surface_on_timeout`

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
- `hook_aligned=true/false`: lubang payload sudah siap masuk pasak hook atau belum.
- `hook_err=(radial,axial,rov,yaw)`: error lubang ke sumbu pasak, error kedalaman sepanjang pasak, jarak ROV ke pose referensi, dan error yaw. Release fisik diblokir oleh radial/axial; nilai `rov/yaw` dipakai untuk panduan operator dan autonomous approach.
- `grip_stress=0..1`: tegangan virtual gripper saat payload sedang dibawa.
- `release_block=...`: alasan release belum valid, misalnya `hole_not_on_peg`, `peg_depth_bad`, atau `unknown_qr`.

## 20. Batas Klaim untuk Laporan

Kalimat aman:

```text
Simulasi ini merepresentasikan alur misi ROV KKI di Gazebo: kendali ROV, kamera depan/wall untuk QR samping, mekanisme gripper, collision payload, dan pemindahan payload ke hook. Mode default memakai model kinematic dengan damping agar real-time dan stabil untuk latihan operator. Visual air dibuat representatif bawah air, sedangkan klaim hidrodinamika penuh memerlukan kalibrasi tambahan menggunakan data uji kolam dan parameter fisik ROV asli.
```

Yang boleh diklaim:

- Workspace sesuai untuk simulasi misi KKI.
- ROV bisa digerakkan dengan stik.
- Payload QR samping bisa dibaca kamera depan/wall.
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
