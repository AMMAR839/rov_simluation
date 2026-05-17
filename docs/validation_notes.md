# Catatan Pertanggungjawaban Simulasi

Dokumen ini memisahkan klaim yang aman dari klaim yang harus menunggu data uji asli.

## Thruster

Di mode default `physics_mode:=kinematic`, input operator tetap masuk ke `thruster_allocator`, lalu dikonversi menjadi PWM ESC `1100..1900 us` dengan netral `1500 us`. Driver gerak membaca `/rov/thruster_pwm`, mengubah PWM menjadi estimasi thrust, lalu menggerakkan pose ROV secara stabil.

Propeller yang terlihat berputar adalah visualisasi PWM/thrust command. Visual ini berguna untuk debugging dan presentasi, tetapi tidak boleh dipakai sebagai bukti bahwa gaya motor asli sudah sama sebelum ada kalibrasi motor, propeller, dan ESC.

Klaim aman:

- mapping 6 thruster sudah merepresentasikan surge, sway, heave, dan yaw,
- command joystick masuk ke `/rov/cmd_vel`,
- allocator mengubah command menjadi enam PWM thruster,
- `/rov/thruster_status` adalah estimasi thrust dari PWM,
- propeller visual berputar saat PWM keluar dari deadband netral.

Klaim yang perlu data uji:

- nilai Newton thrust motor asli,
- efisiensi propeller di air,
- arus balik dari propeller,
- konsumsi daya motor.

## Air

Air di Gazebo dibuat sebagai representasi visual kolam bawah air. Elemen seperti tint, transparansi, ripple, wavefield ringan, thruster wash, gelembung, dan caustic dipakai agar kamera dan operator melihat arena seperti lingkungan bawah air.

World air di `/home/ammar/Documents/rov_gamantaray_1` dan `/home/ammar/Documents/rov_gamantaray_2` tidak disalin langsung karena visual airnya hanya berupa plane biru transparan (`water_plane`). Bagian yang lebih penting dari referensi tersebut adalah ide buoyancy, hydrodynamics, ukuran density air, layout thruster, dan pola world Gazebo. Workspace ini memakai air visual yang lebih lengkap untuk kolam KKI: permukaan, volume transparan, fog, haze kedalaman, ripple, wavefield ringan, bubble, caustic, wake visual saat dekat permukaan, dan thruster wash saat ROV bergerak di bawah air.

Referensi eksternal dipakai dengan batas yang jelas:

- `gazebosim/gz-sim` dipakai sebagai acuan utama untuk jalur fisika Gazebo Harmonic: `Buoyancy`, `Hydrodynamics`, dan `Thruster`.
- `osrf/vrx` dipakai sebagai acuan visual wavefield/wake, bukan sebagai full dependency.
- `rock-gazebo/simulation-gazebo_underwater` tidak dipakai langsung karena plugin tersebut untuk Gazebo Classic lama; konsep buoyancy/damping-nya tetap relevan.

Klaim aman:

- geometri kolam dan kedalaman air dibuat sesuai asumsi arena,
- visual bawah air membantu simulasi misi dan pembacaan kamera,
- riak permukaan adalah efek visual berbasis posisi/kecepatan ROV,
- thruster wash adalah efek visual berbasis odometry ROV, bukan solver turbulensi,
- mode default cocok untuk pengujian alur misi dan ergonomi kontrol.

Klaim yang perlu data uji:

- drag dan turbulensi sama dengan kolam asli,
- added mass dan damping sama dengan ROV asli,
- waktu tempuh sama dengan hardware asli,
- gelombang permukaan sama dengan fluid solver dunia nyata,
- kualitas visual kamera sama dengan kamera underwater asli.

## Kalimat Laporan Yang Aman

```text
Simulasi ini digunakan untuk menguji alur misi ROV, integrasi ROS 2-Gazebo, mapping kontrol, kamera, QR Code, gripper, dan perpindahan payload. Visual air dan kolam dibuat agar merepresentasikan lingkungan bawah air KKI. Parameter fisika fluida dan thrust belum diklaim identik dengan sistem asli sebelum dilakukan kalibrasi terhadap data uji kolam.
```

## Data Uji Yang Dibutuhkan Agar Lebih Kuat

- ukuran kolam dan kedalaman aktual,
- massa, dimensi, dan buoyancy ROV,
- kurva command motor vs thrust di air,
- kecepatan maksimum maju/geser/naik/yaw dari ROV asli,
- rekaman kamera underwater untuk membandingkan pencahayaan dan visibilitas.

## Validasi 2026-05-17 - `rov_variant:=github_blue_joint`

Masalah sebelumnya: varian `github_blue_joint` memakai rahang sebagai link dinamis dengan revolute joint, sementara body ROV digerakkan kinematic lewat `/world/kki_rov_pool/set_pose`. Kombinasi ini dapat membuat rahang terlihat offset atau terlepas di GUI.

Perbaikan: `rov_variant:=github_blue_joint` sekarang diarahkan ke model compact stabil `gamantaray_rov_github_blue_gripper` dengan rahang visual kinematic. Varian joint fisika penuh dipindah ke `rov_variant:=github_blue_joint_experimental`.

Validasi yang dijalankan:

```bash
python3 -m py_compile src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
colcon build --symlink-install
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=true joystick_device:=/dev/input/js0 rov_variant:=github_blue_joint use_vision:=false
```

Hasil: launch headless berjalan, joystick `/dev/input/js0` terdeteksi, dan world sementara `/tmp/kki_rov_pool_A_kinematic.sdf` serta `/tmp/kki_rov_pool_C_kinematic.sdf` memakai URI `model://gamantaray_rov_github_blue_gripper` plus model `gamantaray_rov_left_gripper_jaw`.

## Validasi 2026-05-17 - Strict Gripper Contact

Masalah sebelumnya: mode gripper masih memiliki bantuan `claw_guided` / funnel, sehingga payload bisa dituntun ke tengah rahang dan berubah `attached` hanya karena alignment, bukan karena dua sisi rahang benar-benar menjepit.

Perbaikan: default launch memakai `payload_contact_model:=strict`. Pada mode ini payload hanya bergerak dari resolusi tabrakan body/frame/front tip/pad. Status `attached` hanya boleh terjadi setelah `pinched=true`, `contact=bilateral_clamp`, gap rahang cukup kecil, dan kontak dua sisi bertahan minimal `grip_min_bilateral_contact_s`.

Validasi yang dijalankan:

```bash
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
colcon build --symlink-install
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false rov_variant:=github_blue_joint payload_contact_model:=strict
ros2 param get /gripper_manager payload_contact_model
ros2 topic pub --once /rov/gripper_cmd std_msgs/msg/Float64 '{data: 1.0}'
ros2 topic echo --once /rov/gripper_status
```

Hasil: parameter terbaca `strict`. Saat gripper ditutup tanpa payload berada di antara dua pad, status tetap `closed`, `attached` tidak aktif, `pinched=false`, `collision=false`, dan `contact=none`.

## Validasi 2026-05-17 - Anti-Clipping Gripper Frame

Masalah lanjutan: payload masih bisa tampak menembus bagian kotak/frame gripper ketika masuk terlalu dalam ke pangkal capit.

Perbaikan:

- titik jepit default dimajukan dari `capture_forward_offset_m:=0.335` ke `0.360`,
- inner pad rahang kiri/kanan digeser ke depan agar area kontak berada di ujung claw,
- ditambahkan `gripper_frame_backstop_x_m:=0.330`; jika payload masuk lebih belakang dari batas ini, node memberi resolusi kontak `frame_backstop` dan mendorong payload keluar dari frame,
- visual rahang di mode strict sekarang berhenti pada lebar payload ketika payload berada di volume sapuan claw, meskipun belum `attached`.

Validasi yang dijalankan:

```bash
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
python3 - <<'PY'
import xml.etree.ElementTree as ET
for path in [
    'src/rov_gamantaray_description/models/gamantaray_gripper_ricketts_left_jaw_visual/model.sdf',
    'src/rov_gamantaray_description/models/gamantaray_gripper_ricketts_right_jaw_visual/model.sdf',
    'src/rov_gamantaray_description/models/gamantaray_rov_github_blue_gripper/model.sdf',
    'src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf',
]:
    ET.parse(path)
PY
colcon build --symlink-install
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false rov_variant:=github_blue_joint payload_contact_model:=strict
```

Hasil: Python compile OK, XML/SDF parse OK, build OK, dan launch headless berjalan tanpa crash.

## Validasi 2026-05-17 - No Snap To Center

Masalah lanjutan: ketika payload/rahang datang dari kanan atau kiri, respons `bilateral_clamp` masih menggeser payload langsung ke tengah rahang.

Perbaikan: koreksi bilateral sekarang tidak lagi memakai `dy=-local_y`. Koreksi dihitung dari selisih penetrasi pad kanan dan kiri, lalu dibatasi oleh `payload_contact_max_step_m`.

Validasi yang dijalankan:

```bash
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
colcon build --symlink-install
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false rov_variant:=github_blue_joint payload_contact_model:=strict payload_contact_max_step_m:=0.006
ros2 param get /gripper_manager payload_contact_model
ros2 param get /gripper_manager payload_contact_max_step_m
ros2 topic echo --once /rov/gripper_status
```

Hasil: parameter terbaca `strict` dan `0.006`. Status gripper terpublish normal.

## Validasi 2026-05-17 - Kontak Pad Aktual Tanpa Bantuan

Masalah lanjutan: payload masih bisa terlihat masuk ke tengah walaupun capit kanan/kiri belum benar-benar menyentuh sisi payload. Penyebabnya adalah kontak pad masih dihitung dari gap global rahang, bukan dari box pad aktual.

Perbaikan:

- kontak kiri/kanan sekarang dihitung dari posisi box `inner_pad_collision` aktual pada SDF,
- rotasi rahang visual efektif ikut dihitung saat menentukan overlap payload dengan pad,
- koreksi lateral diarahkan keluar dari pad yang overlap, bukan menuju `local_y=0`,
- pada validasi saat itu, default `payload_contact_max_step_m` diturunkan menjadi `0.003` supaya resolusi kontak tidak terlihat loncat. Nilai aktif terbaru dicatat pada bagian validasi capit lebih kecil di bawah.

Validasi yang dijalankan:

```bash
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
colcon build --symlink-install
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false rov_variant:=github_blue_joint payload_contact_model:=strict
```

Hasil saat validasi tersebut: Python compile OK, XML/SDF parse OK, build OK, launch headless menjalankan proses `gripper_manager`, default `payload_contact_max_step_m` terbaca `0.003`, dan status awal gripper tetap `contact=none`, `pinched=false`, `pad=(false,false)` saat payload belum menyentuh pad. Nilai aktif terbaru sudah dinaikkan untuk mengurangi overlap pada kecepatan lebih tinggi.

## Validasi 2026-05-17 - Hook PVC Dan Payload Tidak Masuk Lantai

Masalah lanjutan: saat payload pertama kali tercapit, payload dapat tertarik turun jika ROV terlalu rendah. Bentuk hook juga masih terlalu sederhana dan toleransi release masih terasa seperti bantuan otomatis.

Perbaikan:

- attach ditolak jika titik jepit gripper berada lebih rendah dari lantai payload,
- pose payload yang sedang dibawa di-clamp minimal pada `payload_floor_z`,
- hook A/B/C/D diganti menjadi model PVC dinding: backing, clamp bibir kolam, pipa vertikal, elbow, peg horizontal, dan ujung naik,
- semua bagian utama hook memiliki collision SDF,
- toleransi hook diperketat menjadi `hook_snap_hole_tolerance_m=0.008`, `hook_snap_axial_tolerance_m=0.008`, `hook_snap_rov_tolerance_m=0.30`, dan `hook_snap_yaw_tolerance_rad=0.35`.

Validasi yang dijalankan:

```bash
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
python3 - <<'PY'
import xml.etree.ElementTree as ET
for path in [
    'src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf',
    'src/rov_gamantaray_gazebo/models/kki_payload_A/model.sdf',
    'src/rov_gamantaray_gazebo/models/kki_payload_B/model.sdf',
    'src/rov_gamantaray_gazebo/models/kki_payload_C/model.sdf',
    'src/rov_gamantaray_gazebo/models/kki_payload_D/model.sdf',
]:
    ET.parse(path)
PY
```

Hasil: Python compile OK, XML/SDF parse OK, `colcon build --symlink-install` OK, launch headless berjalan tanpa crash, `/rov/gripper_status` terpublish, dan generated world `/tmp/kki_rov_pool_A_kinematic.sdf` memuat hook PVC baru beserta collision `peg_collision` dan `tip_up_collision`.

## Validasi 2026-05-17 - Hook Lancip Pendek Dan Guard Tabrakan Kinematic

Masalah lanjutan: ujung hook yang menghadap ke atas masih terasa terlalu tinggi dan ROV/payload masih bisa terlihat menembus hook karena mode default menggerakkan body ROV dengan `set_pose`.

Perbaikan:

- ujung hook PVC diganti menjadi guide lancip pendek: visual tetap cone supaya terlihat lancip, tetapi collision ujung memakai cylinder kecil agar DART/Gazebo tidak memberi warning collision cone,
- tinggi peg disamakan dengan logika release di `gripper_manager` pada `z=-0.43`,
- panjang peg logika release disamakan dengan model world menjadi `hook_peg_length_m=0.26`,
- `kinematic_driver` menambahkan `hook_collision_guard` untuk pipa vertikal, peg horizontal, dan ujung hook,
- launch mengekspos parameter `hook_collision_guard_enabled`, `hook_collision_body_radius_m`, `hook_collision_margin_m`, `hook_collision_body_half_height_m`, dan `hook_collision_velocity_damping`,
- `gripper_manager` menambahkan guard payload terhadap pipa vertikal dan ujung hook, tetapi area peg tetap dibiarkan supaya lubang payload masih bisa masuk ke gantungan.

Validasi yang dijalankan:

```bash
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/kinematic_driver.py src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
colcon build --symlink-install
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false rov_variant:=github_blue_joint payload_contact_model:=strict
ros2 param get /kinematic_driver hook_collision_guard_enabled
ros2 param get /kinematic_driver hook_collision_body_radius_m
ros2 param get /gripper_manager hook_peg_length_m
ros2 topic echo --once /rov/gripper_status
```

Hasil: Python compile OK, XML/SDF parse OK, build OK, launch headless berjalan tanpa crash, `hook_collision_guard_enabled=True`, `hook_collision_body_radius_m=0.17`, `hook_peg_length_m=0.26`, status `/rov/gripper_status` terpublish, generated world memuat `tip_point` visual cone, `tip_point_collision` cylinder, dan `peg_collision`. Log Gazebo tidak lagi memunculkan warning collision cone.

### Validasi 2026-05-17 - Ujung Lancip Hook Dihapus

Masalah lanjutan: ujung cone/lancip pada hook terlihat seperti tombak dan tidak sesuai permintaan visual.

Perbaikan:
- visual `tip_point` berbentuk cone di hook A/B/C/D dihapus,
- collision `tip_point_collision` dihapus,
- stopper pendek `tip_up` tetap dipertahankan supaya ujung peg masih punya batas fisik.

Validasi:
- XML template world OK,
- `colcon build --symlink-install` OK,
- launch headless singkat OK,
- generated world `/tmp/kki_rov_pool_A_kinematic.sdf` tidak lagi memuat `tip_point` atau `<cone>`.

### Validasi 2026-05-17 - Stopper Ujung Peg Hook Dihapus

Masalah lanjutan: stopper silinder `tip_up` di ujung peg masih menghalangi jalur masuk lubang payload.

Perbaikan:
- visual dan collision `tip_up` di hook A/B/C/D dihapus,
- guard kinematic untuk ujung hook dihapus dari `kinematic_driver`,
- guard payload terhadap ujung hook dihapus dari `gripper_manager`,
- peg horizontal tetap memiliki collision dan tetap menjadi target masuk lubang payload.

Validasi:
- XML template world OK,
- Python compile untuk `kinematic_driver.py` dan `gripper_manager.py` OK,
- `colcon build --symlink-install` OK,
- launch headless singkat OK,
- generated world tidak lagi memuat `tip_up`, `tip_point`, atau `<cone>`,
- `peg_collision` masih ada sehingga lubang payload tetap punya pasak horizontal untuk masuk.

### Validasi 2026-05-17 - Kontak Luar Capit Dan Peg Hook

Masalah lanjutan: payload masih bisa terlihat menembus sisi luar capit. Payload juga bisa terlihat masuk ke area hook tetapi belum berubah menjadi `hung`, karena release masih diblokir oleh pose/yaw ROV walaupun lubang sudah berada di peg.

Perbaikan:
- kontak capit tidak hanya menghitung `inner_pad_collision`, tetapi juga `outer_finger_collision`, `front_hook_tip_collision`, dan `rear_link_collision`,
- batas visual jaw juga memakai semua komponen rahang tersebut supaya sisi luar capit tidak terus digambar menembus payload,
- `gripper_manager` menambahkan guard peg horizontal: peg boleh masuk hanya jika pusat lubang payload sejajar dengan sumbu peg; jika bagian solid payload menyentuh peg, payload dikoreksi keluar,
- release ke mode `hung` sekarang diblokir oleh error lubang-ke-peg dan kedalaman peg saja. Jarak/yaw ROV tetap dilaporkan di `hook_err`, tetapi tidak lagi menghalangi release jika lubang memang sudah masuk peg.

Validasi:
- Python compile OK,
- `colcon build --symlink-install` OK,
- launch headless singkat OK tanpa traceback/error,
- generated world tetap tidak memuat `tip_up`, `tip_point`, atau `<cone>`, dan `peg_collision` masih ada,
- probe hook menunjukkan lubang tepat di peg menghasilkan `aligned_exact_hole=True` dan `release_block=ready` walaupun ROV jauh,
- probe peg guard menunjukkan pose lubang sejajar tidak dikoreksi, sedangkan pose solid payload yang mengenai peg dikoreksi keluar,
- probe outer finger menunjukkan kontak sisi luar capit terdeteksi.

### Validasi 2026-05-17 - Payload Tidak Menghindari Peg Saat Lubang Sudah Masuk

Masalah lanjutan: guard peg terlalu agresif. Saat payload dibawa gripper dan lubang sudah hampir masuk ke peg, payload masih bisa terdorong menjauh karena guard menganggap peg menabrak badan payload.

Perbaikan:
- ditambahkan parameter `hook_peg_pass_window_m`, default `0.024`,
- default `hook_snap_hole_tolerance_m` dinaikkan ke `0.020`,
- default `hook_snap_axial_tolerance_m` dinaikkan ke `0.022`,
- guard peg tidak lagi mendorong payload ketika peg berada dalam jendela masuk lubang,
- payload tetap didorong keluar jika posisi lubang sudah di luar jendela masuk dan bagian solid payload menyentuh peg.

Validasi:
- probe offset radial `0.000`, `0.017`, dan `0.019 m` menghasilkan `aligned=True`, `release_block=ready`, dan `guard_corrected=False`,
- probe offset radial `0.025 m` menghasilkan `aligned=False` dan `guard_corrected=True`,
- Python compile OK,
- `colcon build --symlink-install` OK,
- launch headless singkat OK tanpa traceback/error.

## Validasi 2026-05-17 - Lubang Payload Terbuka Dan Stop Visual Capit

Masalah lanjutan: lubang payload terlihat seperti lingkaran hitam, bukan bukaan nyata. Rahang kanan/kiri juga masih bisa terlihat menembus payload ketika command tutup ditahan.

Perbaikan:

- visual `hanging_hole_shadow` dihapus dari payload A/B/C/D,
- bukaan lubang payload diperbesar menjadi sekitar 3.4 cm dengan lower/top/side plate yang tetap punya collision,
- `gripper_manager` menambahkan parameter `jaw_visual_stop_clearance_m`,
- visual rahang diposisikan setelah update kontak payload, bukan sebelum update kontak,
- `effective_gripper_position` sekarang memakai stop zone lebih luas dan binary search overlap `inner_pad_collision`, sehingga rahang kinematic berhenti sebelum terus digambar menembus payload.

Validasi yang dijalankan:

```bash
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
python3 - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path
for path in sorted(Path('src/rov_gamantaray_gazebo/models').glob('kki_payload_*/model.sdf')):
    ET.parse(path)
    assert 'hanging_hole_shadow' not in path.read_text()
PY
colcon build --symlink-install
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false rov_variant:=github_blue_joint payload_contact_model:=strict
ros2 param get /gripper_manager jaw_visual_stop_clearance_m
ros2 topic echo --once /rov/gripper_status
```

Hasil: Python compile OK, semua payload SDF parse OK, tidak ada lagi `hanging_hole_shadow`, build OK, launch headless berjalan tanpa crash, parameter `jaw_visual_stop_clearance_m=0.006` terbaca, dan `/rov/gripper_status` terpublish.

## Validasi 2026-05-17 - Capit Lebih Kecil Dan Koridor Peg Tidak Menolak Payload

Masalah lanjutan: payload masih terlihat seperti terdorong menjauhi gantungan saat lubang sudah diarahkan ke peg. Pada kecepatan operator yang agak tinggi, sisi luar capit juga masih bisa terlihat overlap dengan payload.

Perbaikan:

- geometri rahang Ricketts kiri/kanan diperkecil lagi di visual dan collision: pivot hub, outer finger, inner pad, front tip, rear link, dan web lebih ramping,
- default geometri aktif di launch diubah menjadi `jaw_pivot_y_m=0.050`, `jaw_open_angle_rad=0.38`, `capture_forward_offset_m=0.335`, dan `open_gap_m=0.125`,
- toleransi capture dibuat lebih ketat: `capture_forward_tolerance_m=0.040` dan `capture_lateral_tolerance_m=0.035`,
- resolusi kontak kinematic diperkuat: `payload_contact_max_step_m=0.016` dan `jaw_visual_stop_clearance_m=0.014`,
- `hook_peg_pass_window_m` dinaikkan menjadi `0.028` agar payload yang lubangnya sudah berada di koridor peg tidak didorong keluar oleh guard,
- syarat release tetap lebih ketat dari koridor lewat: `hook_snap_hole_tolerance_m=0.024` dan `hook_snap_axial_tolerance_m=0.026`.

Validasi yang dijalankan:

```bash
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/gripper_manager.py src/rov_gamantaray_bringup/launch/kki_rov_sim.launch.py
python3 - <<'PY'
import xml.etree.ElementTree as ET
for p in [
 'src/rov_gamantaray_description/models/gamantaray_gripper_ricketts_left_jaw_visual/model.sdf',
 'src/rov_gamantaray_description/models/gamantaray_gripper_ricketts_right_jaw_visual/model.sdf',
 'src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf',
]:
    ET.parse(p)
PY
colcon build --symlink-install
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false rov_variant:=github_blue_joint
```

Hasil:

- XML/SDF parse OK,
- Python compile OK,
- probe parameter menunjukkan default baru aktif: `jaw_pivot_y_m=0.05`, `jaw_open_angle_rad=0.38`, `open_gap_m=0.125`, `payload_contact_max_step_m=0.016`, `jaw_visual_stop_clearance_m=0.014`, `hook_peg_pass_window_m=0.028`,
- probe hook menunjukkan offset radial `0.000`, `0.019`, `0.023`, dan `0.024 m` menghasilkan `corridor=True`, `hook_guard_corrected=False`, dan `release_block=ready`,
- offset `0.027 m` masih boleh lewat koridor tanpa didorong guard, tetapi belum `ready` untuk release,
- offset `0.029 m` dan `0.034 m` dikoreksi keluar oleh hook guard,
- `colcon build --symlink-install` selesai untuk 5 package,
- launch headless 10 detik tidak menampilkan traceback/error.

## Validasi 2026-05-17 - Side Entry Hook Diblok Secara Visual

Masalah lanjutan: payload sudah diblok dari samping secara status, tetapi koreksi collision masih bisa terlihat seperti payload masuk ke arah batang karena fallback guard memakai arah sepanjang hook.

Perbaikan:

- guard peg sekarang memakai proyeksi `XY` lubang payload terhadap sumbu peg untuk mendeteksi side entry, sehingga beda tinggi kecil tidak jatuh ke guard umum,
- jika payload datang dari kanan/kiri ke tengah peg tanpa pernah masuk dari ujung bebas, koreksi pose diarahkan lateral menjauh dari peg,
- side plate lubang payload A/B/C/D ditebalkan dari `0.008 m` ke `0.010 m`,
- posisi side plate diatur ke `y=+/-0.020 m`, sehingga bukaan visual lubang menjadi sekitar `0.030 m` dan lebih jelas menyerupai bukaan fisik, bukan celah lebar yang bisa dimasuki dari samping.

Validasi:

- Python compile OK untuk `gripper_manager.py` dan launch file,
- XML/SDF parse OK untuk payload A/B/C/D,
- probe side approach langsung ke tengah peg A: `corridor=False`, `side_corrected=True`, `pose_after=(-4.72, 0.047, -0.4535)`, `reason=side_entry_blocked`,
- probe masuk dari ujung peg A tetap valid: `tip_corridor=True`, lalu `slide_corridor_after_tip=True`,
- `colcon build --symlink-install` selesai untuk 5 package,
- launch headless 10 detik tidak menampilkan traceback/error.

## Validasi 2026-05-17 - Side Entry Dekat Ujung Hook Diblok

Masalah lanjutan: setelah side entry di tengah pipa diblok, payload masih bisa masuk dari kanan/kiri di area dekat ujung bebas pipa karena syarat lama menganggap posisi dekat ujung sebagai entry valid.

Perbaikan:

- entry valid sekarang harus melewati dua tahap: lubang payload pernah berada di luar ujung bebas pipa, lalu masuk ke koridor peg sepanjang sumbu pipa,
- posisi dekat ujung pipa tanpa tahap luar-ujung tidak lagi dianggap `entered_from_tip`,
- memori pre-entry dibersihkan jika payload menjauh dari hook.

Validasi probe:

- side approach di tengah peg A -> `corridor=False`, `side_entry_blocked`, pose dikoreksi ke `y=0.047`,
- side approach dekat ujung peg A (`x=-4.61`) -> `corridor=False`, `side_entry_blocked`, pose dikoreksi ke `y=0.047`,
- masuk benar dari luar ujung peg A (`x=-4.585`) merekam pre-entry, lalu geser masuk ke `x=-4.61` -> `corridor=True`,
- setelah pre-entry valid, guard peg tidak mengoreksi payload yang masuk dari ujung.

## Validasi 2026-05-17 - Payload Tidak Keluar Samping Setelah Masuk Peg

Masalah lanjutan: payload yang sudah masuk dari ujung peg masih bisa digeser kiri/kanan dan terlihat keluar dari samping.

Perbaikan:

- pre-entry di ujung peg sekarang harus memiliki gerak masuk sepanjang sumbu pipa, bukan sekadar berada di dekat ujung,
- setelah entry valid, `gripper_manager` mengaktifkan constraint lubang-pipa sementara: lubang payload diproyeksikan kembali ke sumbu peg jika operator menarik kiri/kanan,
- constraint ini hanya aktif setelah entry valid; pose lateral tanpa entry valid tidak ditarik otomatis ke hook.

Validasi probe:

- side approach dekat ujung peg A tanpa gerak axial -> `corridor=False`, `side_entry_blocked`, pose dikoreksi ke `y=0.047`,
- front approach dari luar ujung `x=-4.575 -> -4.585 -> -4.61` -> `front_inside_corridor=True`,
- setelah front entry valid, payload digeser ke `y=0.060` -> guard menghasilkan `peg_lateral_constraint` dan posisi kembali ke `y=0.000`,
- pose lateral yang sama tanpa entry valid -> `corridor=False`, `guard=False`, sehingga tidak ada bantuan otomatis masuk hook.

## Validasi 2026-05-17 - Release Hook Lebih Mudah Saat Peg Sudah Masuk Lubang

Masalah lanjutan: secara visual peg sudah masuk lubang payload, tetapi status masih bisa `hole_not_on_peg` karena syarat `ready` memakai pusat lubang yang sangat presisi. Payload yang sedang attached juga masih punya constraint lunak sehingga pose aktual bisa tertinggal sedikit dari titik gripper.

Perbaikan:

- `hook_peg_pass_window_m` dinaikkan dari `0.040` ke `0.045`,
- `hook_latch_release_tolerance_m` dinaikkan dari `0.045` ke `0.050`,
- `hook_latch_axial_release_tolerance_m` tetap longgar di `0.075`,
- `hook_latch_memory_s` dinaikkan ke `2.50`,
- saat payload attached dan berada dalam toleransi latch, `update_hook_alignment_status()` langsung memberi `release_block=ready_latched`,
- release juga mengecek pose target payload di gripper jika pose aktual masih sedikit tertinggal, tetapi hanya jika pose aktual sudah dekat koridor hook,
- constraint payload saat dicapit dibuat lebih rapat: `held_payload_response_s=0.06` dan `held_payload_max_sway_m=0.020`.

Validasi probe:

- offset `0.000 m` -> `ready`, `snapped=A`,
- offset `0.024 m` -> `ready`, `snapped=A`,
- offset `0.032 m` -> `ready_latched`, `snapped=A`,
- offset `0.040 m` -> `ready_latched`, `snapped=A`,
- offset `0.048 m` -> `ready_latched`, `snapped=A`,
- offset `0.050 m` -> `ready_latched`, `snapped=A`,
- offset `0.052 m` -> `hole_not_on_peg`, tidak snap,
- offset `0.060 m` -> `hole_not_on_peg`, tidak snap.

Build:

- `python3 -m py_compile ...` OK,
- `colcon build --symlink-install` selesai untuk 5 package.

## Validasi 2026-05-17 - Hook Fisik Tetap Bisa Menggantung Tanpa QR

Masalah lanjutan: payload bisa terlihat sudah masuk ke gantungan, tetapi jatuh lagi karena target `payload_code` belum tersedia dari QR. Selain itu, jendela collision peg yang terlalu besar membuat pipa bisa terlihat menembus bagian solid payload.

Perbaikan:

- release sekarang memilih hook fisik yang benar-benar sedang dimasuki jika `payload_code`/QR belum tersedia,
- QR tetap dipakai untuk target misi, tetapi fisika gantung tidak diblokir oleh `unknown_qr`,
- `hook_peg_pass_window_m` diturunkan menjadi `0.028` agar collision guard kembali menolak bagian solid payload,
- `hook_latch_release_tolerance_m` disetel ke `0.036`,
- `hook_latch_memory_s` disetel ke `3.00`,
- fallback pose target gripper hanya dipakai jika pose aktual payload sudah dekat koridor hook, supaya tidak menjadi snap dari jauh.

Validasi probe:

- target `None`, offset `0.000`, `0.024`, `0.028`, `0.032`, `0.036 m` -> `snapped=A`,
- target `None`, offset `0.038` dan `0.045 m` -> `hole_not_on_peg`, tidak snap,
- target `A` memberikan hasil sama,
- `python3 -m py_compile ...` OK,
- `colcon build --symlink-install` selesai untuk 5 package,
- launch headless 10 detik tidak menampilkan traceback/error.

## Validasi 2026-05-17 - Payload Tidak Bisa Masuk Hook Dari Samping

Masalah lanjutan: payload bisa ditabrakkan dari kanan/kiri ke bagian tengah pipa dan dianggap masuk gantungan. Ini tidak realistis karena lubang seharusnya masuk dari ujung bebas pipa, bukan dari samping.

Perbaikan:

- ditambahkan entry gate `hook_entry_tip_min_t=0.78`,
- ditambahkan memori entry `hook_entry_memory_s=4.00`,
- bypass guard peg hanya aktif jika lubang payload pernah melewati ujung bebas pipa,
- side approach langsung ke tengah pipa menghasilkan `release_block=side_entry_blocked`,
- jika payload sudah masuk dari ujung bebas pipa lalu meluncur ke dalam, status tetap bisa `ready`/`ready_latched`.

Validasi probe:

- payload langsung ditempatkan di tengah pipa dari samping -> `corridor=False`, `status=False`, `reason=side_entry_blocked`, `snapped=None`,
- guard pada side approach mengoreksi pose payload keluar dari pipa,
- payload masuk dari ujung bebas pipa -> `corridor=True`, `entry=A`,
- setelah payload digeser masuk ke bagian tengah pipa dengan entry valid -> `status=True`, `reason=ready`, `snapped=A`,
- `python3 -m py_compile ...` OK,
- `colcon build --symlink-install` selesai untuk 5 package,
- launch headless 10 detik tidak menampilkan traceback/error.

## Validasi 2026-05-17 - Housing Capit Lebih Kecil Dan Latch Hook Lebih Mudah

Masalah lanjutan: kotak/housing gripper masih terlihat terlalu besar. Payload juga terasa sulit dilepas ke gantungan karena ketika lubang sudah masuk peg lalu operator bergeser sedikit, release bisa berubah gagal.

Perbaikan:

- ukuran `gripper_saddle`, `gripper_neck`, `actuator_housing`, `upper_bridge`, `side_rail`, `front_yoke`, `cheek_plate`, `lower_brace`, `cross_pin`, dan `pivot_cap` pada model `gamantaray_rov_github_blue_gripper` diperkecil,
- collision mount gripper disamakan dengan ukuran visual yang lebih kecil,
- ditambahkan parameter `hook_latch_release_tolerance_m=0.032`, `hook_latch_axial_release_tolerance_m=0.034`, dan `hook_latch_memory_s=1.20`,
- jika payload masih attached dan lubangnya sudah sempat masuk koridor peg, node menyimpan titik latch sebentar sehingga release setelah geser kecil masih menjadi `hung`,
- offset yang terlalu jauh tetap gagal, sehingga payload tidak snap dari luar gantungan.

Validasi:

- XML/SDF parse OK untuk model ROV compact dan dua model rahang,
- Python compile OK untuk `gripper_manager.py` dan launch file,
- probe release hook:
  - offset `0.024 m` -> `snapped=A`, `reason=ready`,
  - offset `0.028 m` -> `snapped=A`, `reason=ready_latched`,
  - offset `0.030 m` -> `snapped=A`, `reason=ready_latched`,
  - offset `0.033 m` -> `snapped=None`, `reason=hole_not_on_peg`,
- `colcon build --symlink-install` selesai untuk 5 package,
- launch headless 10 detik tidak menampilkan traceback/error.
