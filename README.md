# WS_ROV - Simulasi KKI 2026 ROV

Workspace ini adalah simulasi ROS 2 Jazzy + Gazebo Harmonic untuk ROV bawah air KKI 2026. Fokus default sekarang adalah simulasi yang ringan, responsif, dan mudah dikendalikan dengan stik.

Fitur utama:

- arena kolam 10 m x 10 m dengan kedalaman representatif 0.85 m,
- visual bawah air: volume air transparan, permukaan air, wavefield ringan, ripple, wake, thruster wash, gelembung, caustic lantai, marka dasar, dan dinding kolam,
- ROV default `rov_variant:=github_blue` memakai mesh BlueROV2 + T200 propeller dari GitHub `evan-palmer/blue`, dengan gripper bawah-depan custom yang dibuat menyatu dengan rangka ROV dan rahang claw yang dianimasikan,
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
- Saat stick dilepas ke tengah, `/rov/manual_cmd_vel` kembali nol dan ROV berhenti saat command source aktif adalah `manual`.
- Tombol `A` menutup gripper.
- Tombol `B` membuka gripper.
- `cmd_vel_mux` memilih command manual atau autonomous sebelum diteruskan ke `/rov/cmd_vel`.

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

Jika analog maju/mundur kebalik:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_invert_surge:=false
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

Aktifkan GUI lomba KKI minimal dan tether:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true use_vision:=true kki_gui:=true tether:=true
```

Jika ingin mengganti sumber kamera QR:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true use_vision:=true qr_image_topic:=/rov/camera/wall/image
```

Pilih payload:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true payload_code:=C
```

Pilih model ROV:

```bash
# default: GitHub BlueROV2-style + gripper claw custom compact
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true rov_variant:=github_blue

# varian stabil: gripper compact menyatu, aman untuk latihan misi
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true rov_variant:=github_blue_joint

# varian eksperimen: gripper menjadi link + revolute joint Gazebo
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true rov_variant:=github_blue_joint_experimental

# alternatif dari referensi lama: Beaumont
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true rov_variant:=beaumont

# alternatif lama dari folder lokal, tetap tersedia untuk pembanding visual
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true rov_variant:=bluerov
```

Mode hydro dengan kontrol yang tetap responsif:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true physics_mode:=hydro
```

Catatan: mulai versi ini, `physics_mode:=hydro` tetap memakai `hydro_control_mode:=kinematic` sebagai default. Artinya world memakai suasana bawah air dan plugin air, tetapi gerak ROV tetap dikendalikan oleh driver kinematic agar analog stik langsung terasa dan tidak macet oleh tuning hidrodinamika.

Mode wrench hydro masih ada untuk eksperimen fisika:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true physics_mode:=hydro \
  hydro_control_mode:=wrench \
  hydro_horizontal_force_gain:=2.00 hydro_vertical_force_gain:=0.80 hydro_yaw_torque_gain:=0.20
```

Kalau memakai `hydro_control_mode:=wrench` dan analog terasa tidak maju, itu masalah tuning gaya/damping hidrodinamika, bukan stik.
Driver wrench sekarang juga memakai `pose_assist_enabled` agar ROV tetap bergerak terlihat sambil gaya/damping hydro belum dikalibrasi penuh.

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
- `src/rov_gamantaray_vision`: deteksi QR dari kamera depan/dinding.
- `src/rov_gamantaray_bringup`: launch utama.
- `setup_asli_ROV`: kode dan script terpisah untuk ROV asli, dibagi menjadi ground station, onboard, hardware driver, dan firmware mikrokontroler.
- `docs/reused_references.md`: ringkasan bagian yang dipakai dari PDF dan dua folder referensi.
- `docs/kki_mission_alignment.md`: checklist kesesuaian workspace terhadap misi PDF.
- `docs/validation_notes.md`: batas klaim simulasi, air, thruster, dan kebutuhan data uji asli.
- `docs/github_reference_selection.md`: referensi GitHub/official yang cocok untuk pengembangan ROV, air, hydrodynamics, dan gripper.
- `docs/custom_rov_modeling_guide.md`: panduan membuat model ROV sendiri, termasuk thruster, propeller, kamera, lampu, gripper, dan variant launch.
- `docs/metode_dan_logika_workspace.md`: penjelasan lengkap logika launch, kontrol, gripper, collision payload, QR, air, misi, dan batas klaim simulasi.
- `docs/cara_menjalankan_workspace.md`: panduan lengkap menjalankan workspace, stik Xbox, keyboard, kamera QR, payload, autonomous, hydro, monitoring topic, dan troubleshooting.
- `docs/migrasi_ke_sistem_nyata.md`: panduan memindahkan workspace simulasi ke ROV nyata, termasuk hardware yang perlu disiapkan, mapping topic, driver PWM, kamera, gripper, sensor, tether, dan urutan uji air.
- `README_REAL_ROV.md`: ringkasan cara mengirim PWM dari ROS 2 ke mikrokontroler/ESC nyata.

## Kesesuaian Dengan Misi PDF

Berdasarkan `Sosialisasi KKI 2026 ROV.pdf`, workspace ini sudah mencakup bagian inti simulasi misi:

- kolam 10 m x 10 m dengan kedalaman 0.7-0.9 m,
- ROV berukuran representatif di bawah batas 35 x 35 x 35 cm,
- dua kamera ROV: kamera depan/dinding untuk QR samping dan kamera bawah untuk observasi dasar kolam,
- payload sesuai gambar PDF: lebar 5 cm, tinggi 10 cm, QR 4 cm x 4 cm menghadap samping, dan lubang gantung 3 cm di atas QR,
- gripper untuk mengambil dan melepas payload,
- hook/gantungan di sisi A/B/C/D,
- teleoperation memakai stik,
- GUI lomba minimal dengan dua kamera, QR, altitude, identitas tim/universitas, desain ROV, trajectory, dan status tether,
- tether visual dinamis dari anchor permukaan ke ROV dengan status panjang/tension,
- baseline autonomous untuk scan, pickup, menuju hook, release, dan surface.

Bagian yang belum dibuat penuh seperti kebutuhan PDF:

- screenshot/logging/replay otomatis,
- alarm audio kedalaman,
- randomisasi posisi A/B/C/D saat launch.

Jadi statusnya: workspace ini sudah sesuai untuk simulasi teknis misi ROV dan memiliki GUI minimal sesuai konsep KKI. Fitur advanced seperti logging/replay/alarm dan randomisasi arena masih bisa ditambahkan.

## Migrasi Ke ROV Nyata

Panduan lengkap migrasi dari simulasi ke hardware ada di:

```bash
docs/migrasi_ke_sistem_nyata.md
```

Ringkasan cara mengirim ke mikrokontroler/ESC nyata:

```bash
README_REAL_ROV.md
```

Ringkasnya, bagian ROS yang bisa dipakai langsung adalah joystick, `cmd_vel_mux`, `thruster_allocator`, QR detector, GUI KKI, dan sebagian mission supervisor. Bagian Gazebo seperti `kinematic_driver`, `gripper_manager` simulasi, `water_effects_driver`, `tether_driver`, dan `ros_gz_bridge` harus diganti dengan driver hardware nyata.

Target hardware minimal:

- komputer onboard ROS 2,
- mikrokontroler/PWM controller,
- ESC + 6 thruster,
- driver gripper,
- 2 kamera,
- IMU/depth sensor,
- leak sensor,
- tether komunikasi/power,
- fuse dan emergency stop.

Output penting dari workspace untuk hardware adalah `/rov/thruster_pwm`; topic ini harus dibaca oleh driver hardware untuk mengirim PWM ke ESC.

## Metode Dan Algoritma

### 1. World kolam

Launch membaca `src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf`, lalu membuat world sementara di `/tmp/kki_rov_pool_<payload>_<mode>.sdf`.

Placeholder yang diganti saat launch:

- `@PAYLOAD_CODE@`: memilih payload QR A/B/C/D,
- `@ROV_MODEL_URI@`: memilih model `gamantaray_rov` atau `gamantaray_rov_hydro`,
- `@PHYSICS_STEP_SIZE@`: `0.005` untuk kinematic, `0.001` untuk hydro,
- `@HYDRO_WORLD_PLUGINS@`: plugin buoyancy dan apply wrench hanya dimasukkan di mode hydro.

Visual air bukan CFD. Ini visual representatif bawah air agar arena terbaca: air transparan, permukaan, fog, haze kedalaman, ripple, wavefield ringan, bubble, caustic, warna gelap kebiruan, dan lampu kolam. Ada juga node `water_effects_driver` yang menggerakkan visual wake/riak permukaan saat ROV bergerak dekat permukaan dan visual thruster wash saat ROV bergerak di bawah air. World air dari `rov_gamantaray_1` dan `rov_gamantaray_2` tidak disalin langsung karena di referensi tersebut visual air utamanya hanya `water_plane` biru sederhana.

Referensi yang dipakai untuk keputusan air:

- `gazebosim/gz-sim`: acuan utama untuk buoyancy, hydrodynamics, dan thruster di Gazebo Sim.
- `osrf/vrx`: acuan visual wavefield/wake untuk lingkungan maritim, tetapi tidak dimasukkan penuh karena VRX fokus USV permukaan laut, bukan kolam ROV kecil.
- `rock-gazebo/simulation-gazebo_underwater`: dipakai sebagai referensi konsep damping, buoyancy, dan added inertia; pluginnya tidak langsung dipakai karena itu plugin Gazebo Classic lama, bukan Gazebo Harmonic.

### 2. Model ROV

Model utama yang dikendalikan Gazebo tetap bernama `gamantaray_rov`, tetapi URI modelnya bisa dipilih lewat `rov_variant`.

- `rov_variant:=github_blue`: default baru. Body ROV dan T200 propeller diambil dari GitHub `evan-palmer/blue` karena lisensinya MIT dan layout thruster-nya jelas. Mesh body diskalakan agar body + thruster berada dalam batas 35 x 35 x 35 cm dari PDF; gripper boleh berada di luar dimensi ROV sesuai PDF. Gripper bawah-depan dibuat ulang sebagai assembly SDF custom compact: saddle ke rangka, housing aktuator kecil, cheek plate, pin pivot, dan dua rahang claw animasi. Jadi gripper tidak lagi terlihat seperti mesh mentah yang ditempel di depan ROV.
- `rov_variant:=github_blue_joint`: varian stabil untuk command latihan. Model ROV tetap memakai assembly compact `gamantaray_rov_github_blue_gripper`, rahang tetap digerakkan kinematic oleh `gripper_manager`, dan tampilan gripper dibuat menyatu dengan body supaya tidak lepas saat ROV digerakkan oleh driver kinematic/PWM.
- `rov_variant:=github_blue_joint_experimental`: varian eksperimen yang memakai model `gamantaray_rov_github_blue_joint_gripper`. Rahang kiri/kanan menjadi `link` di dalam satu model ROV, bergerak lewat `revolute joint` dan `gz-sim-joint-position-controller-system`. Pada varian ini, kontak rahang-payload dapat dihitung oleh contact solver Gazebo, tetapi tidak direkomendasikan untuk latihan misi karena body ROV masih digerakkan kinematic sehingga joint dinamis bisa jitter atau terlihat offset di GUI.
- `rov_variant:=beaumont`: alternatif dari `rov_gamantaray_1`. Ini disediakan supaya model Beaumont bisa dibandingkan langsung di arena KKI. Gripper Beaumont masih memakai rahang visual terpisah seperti mode kinematic.
- `rov_variant:=bluerov`: alternatif lama dari `rov_gamantaray_2`. Ini tetap ada untuk pembanding, tetapi bukan default karena gripper aslinya tidak sepaket dengan mesh tersebut.

Model ROV dibuat static agar bisa digerakkan langsung oleh ROS melalui service Gazebo `set_pose`. Payload A/B/C/D dibuat non-static/dinamis, sehingga punya massa, collision, dan bisa bergeser saat tersentuh. Karena ROV dan capit default masih digerakkan kinematic, `gripper_manager` menambahkan respons kontak kecil: saat body ROV atau capit masuk zona collision payload sambil bergerak ke arah payload, pose payload digeser di lantai kolam. `kinematic_driver` juga menahan pose ROV agar tidak diset menembus pipa/peg hook. Ini membuat arti collide terlihat di simulasi tanpa membuat ROV jitter atau tembus lantai.

Model berisi:

- body utama,
- 4 thruster horizontal,
- 2 thruster vertikal,
- kamera depan,
- kamera bawah,
- IMU,
- gripper visual terintegrasi di bawah-depan body.

Di mode kinematic, propeller dibuat sebagai enam model visual terpisah: `gamantaray_rov_thruster1_prop_visual` sampai `gamantaray_rov_thruster6_prop_visual`. Pada `rov_variant:=github_blue`, posisi dan orientasi propeller mengikuti `blue_description/description/bluerov2/urdf.xacro` dari GitHub `evan-palmer/blue`, sehingga pusat propeller berada di duct/ring model BlueROV2. Node `kinematic_driver` menghitung pose setiap propeller dari pose ROV, offset thruster, orientasi referensi, dan spin angle berdasarkan PWM dari `/rov/thruster_pwm` yang sudah dikonversi menjadi estimasi thrust. Jadi ROV tidak bergerak langsung dari `cmd_vel`; alurnya lewat PWM thruster dulu.

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
6. Publish `geometry_msgs/Twist` ke `/rov/manual_cmd_vel` setiap 0.05 s.

Karena command selalu dihitung dari posisi axis terbaru, stick yang kembali tengah menghasilkan command nol. Parameter `joystick_enable_button` bisa dipakai sebagai deadman button tambahan.

### 4. Allocator thruster

Node `cmd_vel_mux` memilih `/rov/manual_cmd_vel` atau `/rov/auto_cmd_vel`, lalu meneruskan hasilnya ke `/rov/cmd_vel`. Node `thruster_allocator` mengubah `/rov/cmd_vel` menjadi 6 command thruster normalized, lalu menjadi PWM ESC.

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

Semua command di-clamp ke `-1..1`, lalu dikonversi ke PWM default `1100..1900 us` dengan netral `1500 us`. Deadband default `25 us`. Estimasi thrust dihitung dari PWM dengan kurva kuadratik sederhana:

```text
thrust_n = max_thrust_n * normalized_pwm * abs(normalized_pwm)
```

Jika tidak ada command baru selama `0.5 s`, watchdog otomatis mengirim PWM netral ke semua thruster.

### 5. Driver kinematic

Node `kinematic_driver` membaca `/rov/thruster_pwm`, mengubah PWM menjadi estimasi thrust, lalu menghitung surge, sway, heave, dan yaw untuk mengintegrasikan posisi ROV setiap 0.05 s.

Urutan logika:

1. Baca PWM ESC dari `/rov/thruster_pwm`.
2. Konversi PWM ke estimasi thrust.
3. Hitung command body-frame dari output thruster.
4. Masukkan command ke model respons lambat:
   `v = v + (target_v - v) * (1 - exp(-dt/tau))`.
5. Tambahkan roll/pitch visual kecil sesuai surge/sway agar ROV tidak terlihat terlalu kaku.
6. Ubah body velocity ke world-frame memakai yaw ROV.
7. Integrasikan `x`, `y`, `z`, dan yaw.
8. Batasi posisi agar tetap di dalam kolam.
9. Publish odometry ke `/model/gamantaray_rov/odometry`.
10. Kirim pose ke Gazebo lewat `/world/kki_rov_pool/set_pose`.

Ini bukan fisika hidrodinamika penuh, tetapi sekarang geraknya diberi lag/damping agar lebih terasa seperti ROV di air. Parameter yang bisa dituning: `linear_response_s`, `vertical_response_s`, `yaw_response_s`, `attitude_response_s`, dan `max_visual_tilt_rad`.

### 5a. Animasi propeller dan efek air

Propeller visual menerima spin angle dari nilai thruster:

```text
prop_speed = clamp(thrust / max_thrust) * prop_spin_gain
prop_angle = prop_angle + prop_speed * dt
```

Nilai default `prop_spin_gain_rad_s = 60.0`. Setiap propeller adalah model visual terpisah yang posenya dihitung ulang dari pose ROV, offset thruster, orientasi referensi BlueROV2, dan `prop_angle`. Kalau command nol, `prop_speed` nol sehingga propeller berhenti di sudut terakhir.

`water_effects_driver` menggerakkan dua model visual:

- `rov_surface_wake`: muncul saat ROV bergerak dekat permukaan air.
- `rov_thruster_wash`: muncul saat ROV bergerak di bawah air untuk menggambarkan pusaran/bubble akibat thruster.

Efek ini visual-only. Ia tidak mengubah gaya fisika ROV, tetapi membantu tampilan bawah air lebih realistis saat diuji di Gazebo.

### 6. Gripper dan payload

Node `gripper_manager` menerima `/rov/gripper_cmd`:

- `0.0`: buka,
- `1.0`: tutup.

Pada default `rov_variant:=github_blue` dan varian stabil `rov_variant:=github_blue_joint`, rahang gripper dianimasikan oleh `gripper_manager` sebagai dua model visual terpisah: `gamantaray_rov_left_gripper_jaw` dan `gamantaray_rov_right_gripper_jaw`. Pada varian eksperimen `rov_variant:=github_blue_joint_experimental`, rahang menjadi link fisika di dalam model ROV dan command gripper dipublish ke joint controller Gazebo. Pivot rahang ditempatkan di pin gripper bawah-depan ROV. Versi sekarang memakai desain compact: mount lebih sempit, housing/kotak gripper lebih kecil, cheek plate di sekitar `±0.050 m`, pin lebih pendek, finger lebih ramping, dan bukaan default sekitar `0.125 m`. Tujuannya agar capit proporsional dengan body BlueROV, tetap cukup lebar untuk menerima payload 5 cm, tetapi tidak terlihat seperti lengan besar yang menempel di bawah ROV.

Default kontak gripper sekarang memakai `payload_contact_model:=strict`. Pada mode ini tidak ada funnel otomatis dan tidak ada attach hanya karena payload dekat. Payload baru dianggap terjepit jika semua syarat ini terpenuhi:

- gripper sedang menutup sampai gap rahang mendekati lebar payload,
- inner pad rahang kiri dan kanan sama-sama menekan payload (`pinched=true`),
- pusat payload berada di depan gripper, bukan sekadar dekat dengan ROV,
- error depan-belakang, kiri-kanan, dan tinggi masih dalam toleransi capture,
- kontak dua sisi bertahan sebentar, default `grip_min_bilateral_contact_s:=0.20`, sehingga payload tidak attached dari satu frame yang kebetulan overlap.

Payload A/B/C/D punya bentuk sesuai PDF: plate vertikal 5 cm x 10 cm, QR 4 cm x 4 cm di sisi depan, base 3 cm, dan lubang gantung di atas QR. Collision plate dipecah menjadi beberapa bagian supaya area lubang benar-benar kosong untuk pasak hook. Bukaan lubang di simulasi dibuat sedikit lebih besar, sekitar 3.4 cm, agar peg hook diameter 2 cm punya clearance visual dan collision. Visual hitam penutup lubang sudah dihapus, jadi bagian itu benar-benar kosong, bukan hanya lingkaran hitam. Payload dibuat `static=false` supaya bisa bergeser di lantai kolam. Rahang gripper juga punya collision pada pivot hub, finger, inner pad, hook tip, dan rear link. Pada mode default kinematic, `gripper_manager` membaca pose payload dari `/world/kki_rov_pool/pose/info`, lalu memberi respons kontak berbasis overlap geometri. Jika payload terkena body, frame gripper, front tip, left pad, atau right pad, payload hanya digeser ke arah resolusi tabrakan tersebut. Kontak pad dihitung dari posisi box `inner_pad_collision` aktual yang ikut rotasi rahang visual efektif, bukan dari gap global rahang. Jika kedua pad menekan bersamaan, status menjadi `contact=bilateral_clamp` dan `pinched=true`, tetapi payload tidak di-snap ke tengah. Visual rahang juga dibatasi oleh `jaw_visual_stop_clearance_m` dan binary-search overlap pad, sehingga capit kinematic tidak terus digambar menembus payload saat command tutup tetap ditekan. Koreksi kontak dibatasi oleh `payload_contact_max_step_m` default `0.016 m` dan arahnya selalu keluar dari bagian capit yang benar-benar overlap. Itulah satu-satunya kondisi yang boleh berubah menjadi `attached`.

Saat payload sedang attached, payload tidak lagi ditempel kaku secara instan. Node memakai constraint lunak yang cukup rapat: pose payload mengikuti titik tengah rahang dengan response default `held_payload_response_s:=0.06` dan batas sway `held_payload_max_sway_m:=0.020`. Field `grip_stress` di status menunjukkan seberapa besar payload tertarik dari titik jepit. Model `held_payload_collision_proxy` ikut dipindahkan ke posisi payload. Proxy ini punya collision dan visual hijau transparan supaya collision benda yang sedang diangkat terlihat jelas di Gazebo.

Collision ROV dan capit berada di file berikut:

- ROV/body: `src/rov_gamantaray_description/models/gamantaray_rov_github_blue_gripper/model.sdf`
  - `body_collision`
  - `gripper_saddle_collision`
  - `gripper_neck_collision`
  - `gripper_left_cheek_collision`
  - `gripper_right_cheek_collision`
  - `gripper_cross_pin_collision`
- Capit kiri/kanan: `src/rov_gamantaray_description/models/gamantaray_gripper_ricketts_left_jaw_visual/model.sdf` dan `gamantaray_gripper_ricketts_right_jaw_visual/model.sdf`
  - `pivot_hub_collision`
  - `outer_finger_collision`
  - `inner_pad_collision`
  - `front_hook_tip_collision`
  - `rear_link_collision`

Collision ROV dan capit tetap ada di SDF, tetapi visual debug hijau untuk collision disembunyikan pada model default agar tampilan ROV lebih bersih. Jika ingin mengecek collision, aktifkan tampilan collision dari menu Gazebo, bukan dari visual dekoratif model.

Pada mode default `physics_mode:=kinematic`, pengambilan payload tetap memakai logika alignment dari `gripper_manager`, bukan gaya kontak murni, supaya simulasi stabil dan tidak bergantung pada solver kontak kecil yang mudah jitter. Collision untuk dorong/geser tetap aktif melalui respons kontak kinematic di node yang sama.

Selama attached, pose payload dikendalikan seperti constraint gripper lunak melalui service Gazebo `set_pose`. Payload tidak boleh ditarik lebih rendah dari `payload_floor_z`; jika gripper terlalu rendah, attach ditolak supaya payload tidak tenggelam ke lantai saat pertama kali tercapit. Hook A/B/C/D dibuat seperti gantungan PVC di dinding kolam: clamp di bibir kolam, pipa vertikal, elbow bawah, dan peg horizontal terbuka tanpa ujung lancip atau stopper. Semua bagian utama punya collision SDF, sehingga peg, pipa, dan backing bukan sekadar visual. Pada mode kinematic, collision SDF dilengkapi `hook_collision_guard` di driver ROV dan guard kecil di payload supaya objek yang dikendalikan `set_pose` tidak bebas menembus hook.

Saat gripper dibuka, payload baru masuk mode `hung` jika lubang payload benar-benar masuk ke sumbu pasak hook dan kedalaman sepanjang pasak masih valid. QR menentukan target misi, tetapi fisika gantung tetap boleh terjadi di hook fisik yang sedang dimasuki walaupun QR belum terbaca. Jarak/yaw ROV tetap dilaporkan di `hook_err`, tetapi tidak lagi menjadi syarat utama release, karena yang menentukan secara fisik adalah lubang payload terhadap peg. Toleransi presisi default: `hook_snap_hole_tolerance_m:=0.024` dan `hook_snap_axial_tolerance_m:=0.026`. Saat payload masih dibawa gripper, guard peg memakai jendela masuk `hook_peg_pass_window_m:=0.028` supaya pipa tidak bebas menembus bagian solid payload. Lubang payload harus pernah berada di luar ujung bebas pipa lalu masuk ke dalam sepanjang sumbu pipa (`hook_entry_tip_min_t:=0.78`); jika payload ditabrakkan dari kanan/kiri langsung ke tengah pipa atau area dekat ujung, status menjadi `side_entry_blocked` dan payload didorong keluar secara lateral, bukan sepanjang batang. Setelah entry valid, tarikan kiri/kanan dikunci sebagai kontak lubang-pipa: posisi lubang diproyeksikan balik ke sumbu peg, sehingga payload tidak bisa keluar dari samping selama memori entry masih aktif. Visual side plate lubang payload juga ditebalkan sehingga bukaan terlihat sekitar 3 cm dan tidak tampak seperti celah samping terbuka. Jika lubang sudah sempat masuk dari ujung pipa lalu operator bergeser sedikit, `hook_latch_memory_s:=3.00` menyimpan titik latch sementara; release masih bisa berhasil sebagai `ready_latched` sampai `hook_latch_release_tolerance_m:=0.036` dan axial tolerance `0.075`. Jika dibuka jauh dari hook atau lubangnya belum masuk area pasak, payload masuk state `dropping`: node memberi kecepatan awal dari gerak ROV, gaya berat efektif yang dikurangi buoyancy, drag linear/kuadratik air, batas terminal speed, dan tilt kecil selama tenggelam. Jadi payload tidak langsung snap ke hook atau dasar. Status alignment, tabrakan, drop, dan hook gantung bisa dilihat lewat:

```bash
ros2 topic echo /rov/gripper_status
```

Parameter gantung yang bisa dituning dari launch:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true hanging_damping:=2.8 hanging_release_velocity_gain:=0.35 hanging_max_angle_rad:=0.35 hook_snap_hole_tolerance_m:=0.024 hook_snap_axial_tolerance_m:=0.026 hook_peg_pass_window_m:=0.028 hook_latch_release_tolerance_m:=0.036 hook_latch_memory_s:=3.00 held_payload_response_s:=0.06 held_payload_max_sway_m:=0.020
```

### 7. Deteksi QR

Node `qr_detector` memakai OpenCV `QRCodeDetector` pada `/rov/camera/wall/image`, karena QR pada payload KKI menghadap samping, bukan ke atas.

Jalankan simulasi dengan vision aktif:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true use_vision:=true
```

Lihat kamera depan untuk QR:

```bash
ros2 run rqt_image_view rqt_image_view /rov/camera/wall/image
```

Lihat kamera bawah:

```bash
ros2 run rqt_image_view rqt_image_view /rov/camera/bottom/image
```

Lihat gambar debug hasil deteksi QR:

```bash
ros2 run rqt_image_view rqt_image_view /rov/qr_debug/image
```

Lihat hasil huruf QR:

```bash
ros2 topic echo /rov/qr_code
```

Catatan penting untuk autonomous: `payload_code:=C` pada launch hanya memilih payload yang dispawn di world untuk skenario uji. ROV tidak memakai argumen itu sebagai pengetahuan target saat mode lomba. Target hook baru dianggap valid setelah node QR mempublish huruf A/B/C/D ke `/rov/qr_code`.

Kalau `rqt_image_view` belum ada:

```bash
sudo apt install ros-jazzy-rqt-image-view
```

## GUI Lomba KKI

GUI khusus KKI tersedia sebagai node `kki_dashboard`. Tampilannya dibuat sebagai console operator modern untuk tim Gamantara, Universitas Gadjah Mada: top information bar, dua panel kamera, panel QR/status, altitude, trajectory map, desain ROV, dan footer status. GUI ini menampilkan:

- kamera wall/front untuk QR,
- kamera bottom,
- hasil QR A/B/C/D dan valid/invalid,
- altitude titik tengah ROV dari dasar kolam,
- hari/tanggal/waktu, nama tim, dan universitas,
- trajectory map dari titik awal sampai posisi sekarang,
- desain/schematic ROV dan status tether.

Jalankan bersama simulasi:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true use_vision:=true kki_gui:=true tether:=true
```

Jika hanya ingin mengecek launch tanpa membuka window GUI, tambahkan `kki_gui_window:=false`.

Atau jalankan GUI saja setelah simulasi sudah hidup:

```bash
ros2 run rov_gamantaray_control kki_dashboard
```

Alurnya:

1. Konversi ROS image ke OpenCV image dengan `cv_bridge`.
2. Jalankan `detectAndDecodeMulti`.
3. Jika gagal, coba `detectAndDecode` single QR.
4. Filter hasil agar hanya payload A/B/C/D yang dianggap valid.
5. Jika lebih dari satu QR terlihat, pilih QR dengan skor terbaik dari area terbesar dan posisi paling dekat tengah kamera.
6. Publish hasil huruf QR ke `/rov/qr_code`.
7. Publish debug image ke `/rov/qr_debug/image` jika QR terdeteksi.

### 8. Mission supervisor

Node `mission_supervisor` adalah finite-state machine sederhana:

```text
scan_payload -> pick_payload -> go_to_hook -> release_payload -> surface
```

Ada tiga profil:

- `mission_profile:=full`: mode eksperimen end-to-end untuk scan QR, ambil payload, bawa ke hook, lepas, lalu naik.
- `mission_profile:=release_surface`: profil utama untuk misi nomor 5. Operator manual sudah scan QR, mengambil payload, dan membawa ROV ke dekat hook. Autonomous hanya membuka gripper, memastikan payload benar-benar `hung`, lalu membawa ROV ke permukaan.
- `mission_profile:=carry_release_surface`: mode eksperimen latihan. ROV diasumsikan sudah membawa payload, lalu autonomous menuju hook, melepas payload, dan naik. Ini bukan alur utama misi nomor 5 jika aturan mengharuskan hanya pelepasan payload yang dinilai autonomous.

Kontrol geraknya memakai proportional controller:

```text
cmd = gain * error_posisi
```

Autonomous menuju hook tidak lagi langsung menarik ROV ke koordinat hook. Jalurnya dibuat bertahap:

```text
rise_to_transit -> go_to_standoff -> approach_hook -> release_payload -> surface
```

Pada setiap tahap, target posisi diubah ke body-frame ROV dan yaw ROV diarahkan menghadap dinding hook. Tahap `approach_hook` memakai speed lebih kecil supaya payload tidak datang terlalu cepat ke gantungan.

Target hook:

- A: sisi kiri kolam,
- B: sisi kanan kolam,
- C: sisi atas kolam,
- D: sisi bawah kolam.

Aktifkan dengan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py mission_autonomy:=true use_vision:=true
```

Untuk misi nomor 5 yang sesuai alur PDF, jalankan manual dulu tetapi siapkan autonomous release:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true mission_autonomy:=true mission_profile:=release_surface payload_code:=C command_source:=manual require_qr_for_target:=true use_vision:=true
```

Alurnya:

1. Operator menggerakkan ROV manual.
2. Arahkan kamera depan ke QR sampai `/rov/qr_code` terbaca.
3. Ambil payload secara manual dengan gripper.
4. Bawa payload ke hook yang sesuai QR secara manual.
5. Pastikan `/rov/gripper_status` menunjukkan `hook_aligned=true` atau `release_block=ready`.
6. Pindahkan sumber command ke autonomous:

```bash
ros2 topic pub --once /rov/command_source std_msgs/msg/String "{data: auto}"
```

Setelah itu autonomous membuka gripper hanya kalau `hook_aligned=true`. Jika payload masuk `hung`, ROV naik ke permukaan. Jika belum `hung`, ROV tidak langsung naik karena default `release_surface_on_timeout:=false`. Jika `auto` diaktifkan terlalu cepat, gripper tetap ditahan tertutup sampai alignment hook valid.

Jika menjalankan mode potongan misi dari tengah dan kamera belum sempat membaca QR, publish hasil QR sekali dari terminal lain untuk debug:

```bash
ros2 topic pub --once /rov/qr_code std_msgs/msg/String "{data: C}"
```

Untuk cek alasan payload belum bisa digantung:

```bash
ros2 topic echo /rov/gripper_status
```

Perhatikan field:

- `hook_aligned=true`: lubang payload sudah cukup sejajar dengan pasak.
- `hook_err=(radial,axial,rov,yaw)`: error lubang ke sumbu pasak, error kedalaman sepanjang pasak, jarak ROV ke pose referensi, dan error yaw. Release fisik hanya diblokir oleh radial/axial, sedangkan `rov/yaw` dipakai sebagai panduan operator.
- `grip_stress=0..1`: seberapa besar payload tertarik dari titik jepit selama dibawa.
- `release_block=...`: alasan release belum dianggap valid, misalnya `hole_not_on_peg`, `peg_depth_bad`, atau `unknown_qr`.

Mode eksperimen jika ingin ROV autonomous bergerak dari posisi membawa payload menuju hook:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py mission_autonomy:=true mission_profile:=carry_release_surface payload_code:=C require_qr_for_target:=true joystick:=false
```

## Topic Penting

- `/rov/manual_cmd_vel`: command dari stik/keyboard.
- `/rov/auto_cmd_vel`: command dari mission supervisor.
- `/rov/command_source`: pilih `manual` atau `auto`.
- `/rov/active_command_source`: sumber command yang sedang aktif.
- `/rov/cmd_vel`: hasil mux yang masuk ke allocator.
- `/rov/thruster_pwm`: PWM ESC 6 thruster, default netral `1500 us`.
- `/rov/thruster_status`: estimasi thrust Newton dari PWM.
- `/rov/thruster1/pwm` sampai `/rov/thruster6/pwm`: PWM tiap thruster.
- `/rov/thruster1/cmd` sampai `/rov/thruster6/cmd`: command thruster Gazebo.
- `/rov/gripper_cmd`: command buka/tutup gripper.
- `/rov/gripper_status`: status payload.
- `/rov/camera/wall/image`: kamera depan.
- `/rov/camera/bottom/image`: kamera bawah.
- `/rov/qr_code`: hasil deteksi QR.
- `/rov/tether_status`: status panjang/tension tether visual.
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
- payload QR samping dan lubang gantung ada di `src/rov_gamantaray_gazebo/models/kki_payload_A` sampai `D`.

Untuk mengubah model ROV:

- GitHub BlueROV2 + gripper custom terintegrasi default ada di `src/rov_gamantaray_description/models/gamantaray_rov_github_blue_gripper`,
- propeller T200 GitHub ada di `gamantaray_blue_t200_prop_cw_visual` dan `gamantaray_blue_t200_prop_ccw_visual`,
- rahang gripper animasi ada di `gamantaray_gripper_ricketts_left_jaw_visual` dan `gamantaray_gripper_ricketts_right_jaw_visual`; nama folder masih membawa nama referensi lama, tetapi geometri aktifnya sudah dibuat ulang sebagai claw SDF custom,
- BlueROV2 lokal lama ada di `src/rov_gamantaray_description/models/gamantaray_rov`,
- pilih model saat launch dengan `rov_variant:=github_blue`, `rov_variant:=github_blue_joint`, `rov_variant:=github_blue_joint_experimental`, `rov_variant:=beaumont`, atau `rov_variant:=bluerov`.
- panduan membuat model sendiri ada di `docs/custom_rov_modeling_guide.md`.

Untuk mengembangkan autonomous:

- mulai dari `mission_supervisor.py`,
- ganti proportional controller dengan PID, pure pursuit, atau behavior tree,
- pakai `/rov/qr_code` untuk keputusan target,
- pakai `/model/gamantaray_rov/odometry` untuk feedback posisi.

Untuk mengembangkan vision:

- mulai dari `qr_detector.py`,
- tambahkan filtering hasil QR, estimasi posisi QR dari kamera depan/wall, atau tracking payload,
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
3. Cek command source:

```bash
ros2 topic echo /rov/active_command_source
```

4. Cek publisher `/rov/cmd_vel`:

```bash
ros2 topic info /rov/cmd_vel
```

5. Uji nilai command:

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
