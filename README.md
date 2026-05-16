# WS_ROV - Simulasi KKI 2026 ROV

Workspace ini adalah simulasi ROS 2 Jazzy + Gazebo Harmonic untuk ROV bawah air KKI 2026. Fokus default sekarang adalah simulasi yang ringan, responsif, dan mudah dikendalikan dengan stik.

Fitur utama:

- arena kolam 10 m x 10 m dengan kedalaman representatif 0.85 m,
- visual bawah air: volume air transparan, permukaan air, ripple, gelembung, caustic lantai, marka dasar, dan dinding kolam,
- ROV default `rov_variant:=github_blue` memakai mesh BlueROV2 + T200 propeller dari GitHub `evan-palmer/blue`, dengan claw gripper dari GitHub Ricketts yang dianimasikan,
- 6 thruster dengan propeller visual yang ikut berputar saat command aktif, kamera depan, kamera bawah, gripper, payload QR A/B/C/D, dan hook A/B/C/D,
- kontrol manual dengan stik/gamepad, kontrol keyboard opsional, deteksi QR opsional, dan mission supervisor sederhana.

Default launch memakai `physics_mode:=kinematic`, bukan hydro. Mode ini sengaja dipakai agar real-time lebih tinggi dan ROV berhenti saat command nol. Mode hydro masih tersedia sebagai eksperimen, tetapi bukan jalur utama untuk latihan misi.

## Jalankan Cepat Dengan Stik

Gunakan ini sebagai cara utama. Tidak perlu membuka terminal teleop keyboard terpisah.

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/js0
```

Path stik yang lebih stabil di laptop ini:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/by-id/usb-SHANWAN_Android_Gamepad-joystick
```

Perilaku stik:

- ROV maju/mundur/geser/naik/yaw hanya selama stick analog digeser.
- Saat stick dilepas ke tengah, `/rov/cmd_vel` kembali nol dan ROV berhenti.
- Tombol `A` menutup gripper.
- Tombol `B` membuka gripper.
- Kalau `rov_teleop_keyboard` masih hidup di terminal lain, tutup dulu karena dua node yang sama-sama publish `/rov/cmd_vel` bisa saling mengganggu.

Mapping default:

- left stick atas/bawah: maju/mundur,
- left stick kiri/kanan: geser kiri/kanan,
- right stick atas/bawah: naik/turun,
- right stick kiri/kanan: yaw kiri/kanan.

Jika axis stik berbeda:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true \
  joystick_axis_surge:=1 joystick_axis_sway:=0 joystick_axis_heave:=3 joystick_axis_yaw:=2
```

Jika ingin deadman button, misalnya ROV hanya bergerak saat tombol `LB` ditahan, isi nomor button sesuai stik:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_enable_button:=4
```

## Opsi Launch Penting

Default ringan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true
```

Tanpa GUI untuk uji cepat:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=true
```

Aktifkan deteksi QR:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true use_vision:=true
```

Pilih payload:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true payload_code:=C
```

Pilih model ROV:

```bash
# default: GitHub BlueROV2-style + Ricketts gripper claw
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true rov_variant:=github_blue

# alternatif lama dari folder lokal, tetap tersedia untuk pembanding visual
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true rov_variant:=bluerov
```

Mode hydro eksperimental:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true physics_mode:=hydro
```

## Kontrol Keyboard Opsional

Keyboard tetap bisa dipakai, tetapi harus fokus di terminal `rov_teleop_keyboard`, bukan di window Gazebo.

Terminal 1:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py
```

Terminal 2:

```bash
ros2 run rov_gamantaray_control rov_teleop_keyboard
```

Tombol:

- `w/s`: maju/mundur,
- `a/d`: geser kiri/kanan,
- `r/f`: naik/turun,
- `q/e`: yaw kiri/kanan,
- `o/p`: buka/tutup gripper,
- `space`: stop.

Keyboard teleop sekarang dibuat momentary berbasis timeout: ROV bergerak saat tombol ditahan atau key repeat aktif, lalu otomatis stop setelah tombol dilepas.

## Struktur Package

- `src/rov_gamantaray_description`: model ROV, mesh, sensor, thruster visual, dan model hydro opsional.
- `src/rov_gamantaray_gazebo`: template world kolam, payload QR, hook A/B/C/D, dan visual air.
- `src/rov_gamantaray_control`: joystick driver, keyboard teleop, allocator thruster, kinematic driver, hydro wrench driver, gripper manager, dan mission supervisor.
- `src/rov_gamantaray_vision`: deteksi QR dari kamera bawah.
- `src/rov_gamantaray_bringup`: launch utama.
- `docs/reused_references.md`: ringkasan bagian yang dipakai dari PDF dan dua folder referensi.
- `docs/kki_mission_alignment.md`: checklist kesesuaian workspace terhadap misi PDF.
- `docs/validation_notes.md`: batas klaim simulasi, air, thruster, dan kebutuhan data uji asli.
- `docs/github_reference_selection.md`: referensi GitHub/official yang cocok untuk pengembangan ROV, air, hydrodynamics, dan gripper.

## Kesesuaian Dengan Misi PDF

Berdasarkan `Sosialisasi KKI 2026 ROV.pdf`, workspace ini sudah mencakup bagian inti simulasi misi:

- kolam 10 m x 10 m dengan kedalaman 0.7-0.9 m,
- ROV berukuran representatif di bawah batas 35 x 35 x 35 cm,
- dua kamera ROV: kamera bawah untuk QR dan kamera depan/dinding,
- payload dengan QR Code A/B/C/D,
- gripper untuk mengambil dan melepas payload,
- hook/gantungan di sisi A/B/C/D,
- teleoperation memakai stik,
- baseline autonomous untuk scan, pickup, menuju hook, release, dan surface.

Bagian yang belum dibuat penuh seperti kebutuhan PDF:

- GUI lengkap yang menampilkan dua kamera, hasil QR, ketinggian, waktu, identitas tim, desain ROV, dan trajectory,
- screenshot/logging/replay otomatis,
- alarm audio kedalaman,
- randomisasi posisi A/B/C/D saat launch.

Jadi statusnya: workspace ini sudah sesuai untuk simulasi teknis misi ROV dan pengembangan kontrol, tetapi belum menjadi paket lomba penuh karena GUI dan fitur advanced masih perlu ditambahkan.

## Metode Dan Algoritma

### 1. World kolam

Launch membaca `src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf`, lalu membuat world sementara di `/tmp/kki_rov_pool_<payload>_<mode>.sdf`.

Placeholder yang diganti saat launch:

- `@PAYLOAD_CODE@`: memilih payload QR A/B/C/D,
- `@ROV_MODEL_URI@`: memilih model `gamantaray_rov` atau `gamantaray_rov_hydro`,
- `@PHYSICS_STEP_SIZE@`: `0.005` untuk kinematic, `0.001` untuk hydro,
- `@HYDRO_WORLD_PLUGINS@`: plugin buoyancy dan apply wrench hanya dimasukkan di mode hydro.

Visual air bukan CFD. Ini visual representatif bawah air agar arena terbaca: air transparan, permukaan, fog, haze kedalaman, ripple, bubble, caustic, warna gelap kebiruan, dan lampu kolam. Ada juga node `water_effects_driver` yang menggerakkan visual wake/riak permukaan saat ROV bergerak dekat permukaan. World air dari `rov_gamantaray_1` dan `rov_gamantaray_2` tidak disalin langsung karena di referensi tersebut visual air utamanya hanya `water_plane` biru sederhana.

### 2. Model ROV

Model utama yang dikendalikan Gazebo tetap bernama `gamantaray_rov`, tetapi URI modelnya bisa dipilih lewat `rov_variant`.

- `rov_variant:=github_blue`: default baru. Body ROV dan T200 propeller diambil dari GitHub `evan-palmer/blue` karena lisensinya MIT dan layout thruster-nya jelas. Mesh body diskalakan agar body + thruster berada dalam batas 35 x 35 x 35 cm dari PDF; gripper boleh berada di luar dimensi ROV sesuai PDF. Gripper memakai mesh claw dari GitHub Ricketts karena punya model gripper terpisah dan juga MIT.
- `rov_variant:=bluerov`: alternatif lama dari `rov_gamantaray_2`. Ini tetap ada untuk pembanding, tetapi bukan default karena gripper aslinya tidak sepaket dengan mesh tersebut.

Varian Beaumont dari `rov_gamantaray_1` tidak lagi dibuat sebagai opsi launch karena bentuknya tidak rapi saat dipakai di arena ini. Folder referensi lokal tetap berguna untuk memahami ide claw, tetapi model yang dipakai sekarang berasal dari GitHub yang lisensinya jelas.

Model dibuat static agar bisa digerakkan langsung oleh ROS melalui service Gazebo `set_pose`. Mekanisme attach/release payload tetap mengikuti kebutuhan gripper misi KKI.

Model berisi:

- body utama,
- 4 thruster horizontal,
- 2 thruster vertikal,
- kamera depan,
- kamera bawah,
- IMU,
- gripper visual.

Di mode kinematic, propeller dibuat sebagai enam model visual terpisah: `gamantaray_rov_thruster1_prop_visual` sampai `gamantaray_rov_thruster6_prop_visual`. Pada `rov_variant:=github_blue`, posisi dan orientasi propeller mengikuti `blue_description/description/bluerov2/urdf.xacro` dari GitHub `evan-palmer/blue`, sehingga pusat propeller berada di duct/ring model BlueROV2. Node `kinematic_driver` menghitung pose setiap propeller dari pose ROV, offset thruster, orientasi referensi, dan spin angle berdasarkan nilai `/rov/thruster_status`. Jadi propeller yang terlihat berputar adalah indikator visual command thruster, bukan sumber gaya fisika.

Default ROV dimulai di `z=-0.28`, bukan dekat lantai kolam. Pada mode kinematic batas bawahnya dikunci di `bottom_limit_z=-0.72`, sedangkan lantai kolam berada di sekitar `z=-0.85`. Jadi operator bisa menyentuh area dasar untuk mengambil payload, tetapi pusat ROV tetap dibatasi agar visual tidak menembus lantai.

Kamera diturunkan ke 12 Hz supaya beban rendering lebih ringan. Deteksi QR tidak aktif default; aktifkan dengan `use_vision:=true`.

### 3. Input stik

Node `rov_joystick` membaca langsung event Linux `/dev/input/js*`, jadi tidak tergantung fokus window Gazebo atau terminal keyboard.

Algoritma:

1. Buka device joystick secara nonblocking.
2. Baca event axis dan button.
3. Normalisasi axis dari `-32767..32767` menjadi `-1.0..1.0`.
4. Terapkan deadzone, default `0.08`.
5. Kalikan skala gerak:
   - horizontal `0.75`,
   - vertikal `0.55`,
   - yaw `0.65`.
6. Publish `geometry_msgs/Twist` ke `/rov/cmd_vel` setiap 0.05 s.

Karena command selalu dihitung dari posisi axis terbaru, stick yang kembali tengah menghasilkan command nol. Parameter `joystick_enable_button` bisa dipakai sebagai deadman button tambahan.

### 4. Allocator thruster

Node `thruster_allocator` mengubah `/rov/cmd_vel` menjadi 6 nilai thruster.

Rumus thruster horizontal:

```text
t1 = surge - sway - yaw
t2 = surge + sway + yaw
t3 = surge + sway - yaw
t4 = surge - sway + yaw
```

Rumus thruster vertikal:

```text
t5 = heave
t6 = heave
```

Semua command di-clamp ke batas thrust. Jika tidak ada command baru selama `0.5 s`, watchdog otomatis mengirim nol ke semua thruster.

### 5. Driver kinematic

Node `kinematic_driver` membaca `/rov/thruster_status`, mengubahnya kembali menjadi surge, sway, heave, dan yaw, lalu mengintegrasikan posisi ROV setiap 0.05 s.

Urutan logika:

1. Hitung command body-frame dari output thruster.
2. Masukkan command ke model respons lambat:
   `v = v + (target_v - v) * (1 - exp(-dt/tau))`.
3. Tambahkan roll/pitch visual kecil sesuai surge/sway agar ROV tidak terlihat terlalu kaku.
4. Ubah body velocity ke world-frame memakai yaw ROV.
5. Integrasikan `x`, `y`, `z`, dan yaw.
6. Batasi posisi agar tetap di dalam kolam.
7. Publish odometry ke `/model/gamantaray_rov/odometry`.
8. Kirim pose ke Gazebo lewat `/world/kki_rov_pool/set_pose`.

Ini bukan fisika hidrodinamika penuh, tetapi sekarang geraknya diberi lag/damping agar lebih terasa seperti ROV di air. Parameter yang bisa dituning: `linear_response_s`, `vertical_response_s`, `yaw_response_s`, `attitude_response_s`, dan `max_visual_tilt_rad`.

### 5a. Animasi propeller

Propeller visual menerima spin angle dari nilai thruster:

```text
prop_speed = clamp(thrust / max_thrust) * prop_spin_gain
prop_angle = prop_angle + prop_speed * dt
```

Nilai default `prop_spin_gain_rad_s = 60.0`. Setiap propeller adalah model visual terpisah yang posenya dihitung ulang dari pose ROV, offset thruster, orientasi referensi BlueROV2, dan `prop_angle`. Kalau command nol, `prop_speed` nol sehingga propeller berhenti di sudut terakhir.

### 6. Gripper dan payload

Node `gripper_manager` menerima `/rov/gripper_cmd`:

- `0.0`: buka,
- `1.0`: tutup.

Rahang gripper dianimasikan oleh `gripper_manager` sebagai dua model visual terpisah: `gamantaray_rov_left_gripper_jaw` dan `gamantaray_rov_right_gripper_jaw`. Nilai `/rov/gripper_cmd` tidak lagi langsung mengambil payload. Payload baru dianggap terjepit jika semua syarat ini terpenuhi:

- gripper sedang menutup dan sudah hampir mencapai posisi tertutup,
- pusat payload berada di depan gripper, bukan sekadar dekat dengan ROV,
- error depan-belakang, kiri-kanan, dan tinggi masih dalam toleransi capture,
- bukaan rahang sudah cukup kecil untuk menjepit payload.

Selama attached, pose payload dipindahkan ke tengah rahang gripper melalui service Gazebo `set_pose`. Saat gripper dibuka, payload dilepas di posisi terakhir. Status alignment bisa dilihat lewat:

```bash
ros2 topic echo /rov/gripper_status
```

### 7. Deteksi QR

Node `qr_detector` memakai OpenCV `QRCodeDetector` pada `/rov/camera/bottom/image`.

Alurnya:

1. Konversi ROS image ke OpenCV image dengan `cv_bridge`.
2. Jalankan `detectAndDecodeMulti`.
3. Jika gagal, coba `detectAndDecode` single QR.
4. Publish hasil huruf QR ke `/rov/qr_code`.
5. Publish debug image ke `/rov/qr_debug/image` jika QR terdeteksi.

### 8. Mission supervisor

Node `mission_supervisor` adalah finite-state machine sederhana:

```text
scan_payload -> pick_payload -> go_to_hook -> release_payload -> surface
```

Kontrol geraknya memakai proportional controller:

```text
cmd = gain * error_posisi
```

Target hook:

- A: sisi kiri kolam,
- B: sisi kanan kolam,
- C: sisi atas kolam,
- D: sisi bawah kolam.

Aktifkan dengan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py mission_autonomy:=true use_vision:=true
```

## Topic Penting

- `/rov/cmd_vel`: input gerak utama.
- `/rov/thruster_status`: output allocator untuk driver kinematic.
- `/rov/thruster1/cmd` sampai `/rov/thruster6/cmd`: command thruster Gazebo.
- `/rov/gripper_cmd`: command buka/tutup gripper.
- `/rov/gripper_status`: status payload.
- `/rov/camera/wall/image`: kamera depan.
- `/rov/camera/bottom/image`: kamera bawah.
- `/rov/qr_code`: hasil deteksi QR.
- `/model/gamantaray_rov/odometry`: odometry ROV.
- `/rov/mission_state`: status autonomous.

## Cara Mengembangkan Nanti

Untuk tuning stik:

- ubah axis lewat launch arg `joystick_axis_surge`, `joystick_axis_sway`, `joystick_axis_heave`, `joystick_axis_yaw`,
- ubah sensitivitas lewat `joystick_linear_scale`, `joystick_vertical_scale`, `joystick_yaw_scale`,
- ubah deadzone lewat `joystick_deadzone`,
- pakai `joystick_enable_button` jika ingin deadman button.

Untuk tuning gerak:

- edit `src/rov_gamantaray_control/rov_gamantaray_control/kinematic_driver.py`,
- parameter utama: `max_xy_speed_mps`, `max_z_speed_mps`, `max_yaw_rate_rps`, `linear_response_s`, `vertical_response_s`, `yaw_response_s`, `max_visual_tilt_rad`, dan batas kolam,
- edit `src/rov_gamantaray_control/rov_gamantaray_control/thruster_allocator.py` untuk batas thrust dan mixing thruster.

Untuk mengubah arena:

- edit `src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf`,
- hook A/B/C/D ada sebagai model static di world,
- payload QR ada di `src/rov_gamantaray_gazebo/models/kki_payload_A` sampai `D`.

Untuk mengubah model ROV:

- GitHub BlueROV2 + Ricketts gripper default ada di `src/rov_gamantaray_description/models/gamantaray_rov_github_blue_gripper`,
- propeller T200 GitHub ada di `gamantaray_blue_t200_prop_cw_visual` dan `gamantaray_blue_t200_prop_ccw_visual`,
- rahang gripper Ricketts ada di `gamantaray_gripper_ricketts_left_jaw_visual` dan `gamantaray_gripper_ricketts_right_jaw_visual`,
- BlueROV2 lokal lama ada di `src/rov_gamantaray_description/models/gamantaray_rov`,
- pilih model saat launch dengan `rov_variant:=github_blue` atau `rov_variant:=bluerov`.

Untuk mengembangkan autonomous:

- mulai dari `mission_supervisor.py`,
- ganti proportional controller dengan PID, pure pursuit, atau behavior tree,
- pakai `/rov/qr_code` untuk keputusan target,
- pakai `/model/gamantaray_rov/odometry` untuk feedback posisi.

Untuk mengembangkan vision:

- mulai dari `qr_detector.py`,
- tambahkan filtering hasil QR, estimasi posisi QR dari kamera bawah, atau tracking payload,
- aktifkan kamera/debug dengan `use_vision:=true`.

Untuk eksperimen fisika hydro:

- jalankan `physics_mode:=hydro`,
- model yang dipakai adalah `src/rov_gamantaray_description/models/gamantaray_rov_hydro/model.sdf`,
- driver gaya ada di `hydro_wrench_driver.py`,
- mode ini lebih berat dan bisa lebih lambat dibanding kinematic.

## Pertanggungjawaban Air Dan Uji Asli

Bagian air di workspace ini bisa dipertanggungjawabkan sebagai representasi visual arena bawah air, bukan sebagai prediksi CFD atau validasi gaya fluida asli.

Yang aman diklaim:

- ukuran kolam, kedalaman air, posisi permukaan, lantai, dinding, payload, dan hook dibuat sesuai kebutuhan arena,
- visual air membantu membaca kondisi bawah air: permukaan, tint, transparansi, gelembung, dan caustic,
- riak permukaan yang mengikuti ROV adalah efek visual berbasis pose, bukan simulasi gelombang Navier-Stokes,
- ROV pada mode kinematic diberi batas bawah `z=-0.72` supaya bisa menyentuh dasar untuk misi payload tanpa menembus lantai visual kolam,
- mode kinematic valid untuk menguji alur misi, mapping kontrol, gripper, QR, dan logika navigasi,
- mode hydro hanya baseline eksperimen untuk buoyancy/drag, bukan hasil akhir yang sudah tervalidasi terhadap kolam asli.

Yang tidak boleh diklaim tanpa data uji:

- gaya thrust motor sama dengan ROV asli,
- drag, arus, turbulensi, dan added mass sama dengan air kolam asli,
- waktu tempuh dan konsumsi daya sama dengan hardware asli,
- visual ripple/caustic sama persis dengan kamera underwater asli.

Kalimat aman untuk laporan:

```text
Simulasi air pada Gazebo digunakan sebagai representasi visual dan batas arena bawah air. Validasi utama simulasi ini adalah alur misi, kontrol ROV, pembacaan QR, dan mekanisme payload. Parameter hidrodinamika dan thrust belum diklaim identik dengan sistem asli sebelum dilakukan kalibrasi menggunakan data uji kolam.
```

Langkah kalibrasi kalau nanti ada uji asli:

1. Ukur dimensi kolam, kedalaman air, dan posisi marker/hook.
2. Ukur massa ROV, volume displacement, dan kondisi buoyancy.
3. Uji thrust tiap motor di air untuk mendapat kurva command vs gaya.
4. Jalankan ROV asli maju, geser, naik, turun, dan yaw; catat kecepatan maksimum.
5. Set `max_xy_speed_mps`, `max_z_speed_mps`, dan `max_yaw_rate_rps` di `kinematic_driver.py` mengikuti data uji.
6. Kalau ingin mode hydro dipakai untuk klaim fisika, tune koefisien damping/added mass di `gamantaray_rov_hydro/model.sdf` memakai data uji, bukan perkiraan visual.

## Troubleshooting

Jika ROV tetap bergerak setelah stick dilepas:

1. Pastikan tidak ada terminal `rov_teleop_keyboard` yang masih jalan.
2. Pastikan `mission_autonomy:=false`.
3. Cek publisher `/rov/cmd_vel`:

```bash
ros2 topic info /rov/cmd_vel
```

4. Uji nilai command:

```bash
ros2 topic echo /rov/cmd_vel
```

Saat semua stick dilepas, nilai `linear` dan `angular` harus nol.

Jika stik tidak terbaca:

```bash
ls /dev/input/js*
sudo chmod a+rw /dev/input/js0
```

Jika ingin permission permanen:

```bash
sudo usermod -a -G input $USER
```

Lalu logout dan login lagi.
# rov_simluation
