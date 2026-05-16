# Cara Menjalankan Workspace ROV KKI

Dokumen ini berisi langkah menjalankan simulasi `WS_ROV` dari awal sampai fitur-fitur pentingnya. Gunakan dokumen ini saat ingin menjalankan ROV dengan stik, keyboard, kamera QR, payload A/B/C/D, mode autonomous, atau mode hydro eksperimen.

## 1. Prasyarat

Workspace ini diasumsikan dijalankan di device yang sudah memiliki:

- ROS 2 Jazzy,
- Gazebo Harmonic,
- `colcon`,
- package ROS-Gazebo bridge,
- Python OpenCV dan `cv_bridge` untuk QR.

Lokasi workspace:

```bash
cd /home/ammar/Documents/WS_ROV
```

Setiap terminal baru harus melakukan source ROS:

```bash
source /opt/ros/jazzy/setup.bash
```

Setelah workspace pernah dibuild, source juga install workspace:

```bash
source install/setup.bash
```

## 2. Build Workspace

Jalankan dari root workspace:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Kapan harus build ulang:

- setelah mengubah file Python di `src/rov_gamantaray_control`,
- setelah mengubah launch file,
- setelah menambah model/package baru,
- setelah mengubah install rule di `CMakeLists.txt` atau `setup.py`.

Kalau hanya mengubah file SDF/model yang sudah terinstall dengan symlink, biasanya cukup restart launch. Tetapi aman saja kalau tetap menjalankan build ulang.

## 3. Cara Utama Menjalankan Dengan Stik Xbox

Ini cara yang disarankan:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/js0
```

Jika device stik berubah, cek daftar device:

```bash
ls -l /dev/input/js*
ls -l /dev/input/by-id/
```

Jika ada path by-id, lebih stabil memakai path itu. Contoh:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/by-id/usb-SHANWAN_Android_Gamepad-joystick
```

Perilaku stik:

- ROV bergerak hanya selama stick analog digeser.
- Saat stick kembali ke tengah, command menjadi nol dan ROV berhenti.
- Tombol `A` menutup gripper.
- Tombol `B` membuka gripper.
- Tidak perlu membuka terminal teleop keyboard.
- Tidak masalah jika fokus window sedang di Gazebo, karena node membaca langsung `/dev/input/js*`.

Mapping default:

```text
left stick atas/bawah  -> maju/mundur
left stick kiri/kanan  -> geser kiri/kanan
right stick atas/bawah -> naik/turun
right stick kiri/kanan -> yaw kiri/kanan
A                      -> tutup gripper
B                      -> buka gripper
```

## 4. Jika Mapping Stik Terbalik atau Salah Axis

Launch argument default:

```text
joystick_axis_surge=1
joystick_axis_sway=0
joystick_axis_heave=3
joystick_axis_yaw=2
joystick_invert_surge=true
joystick_invert_sway=true
joystick_invert_heave=true
joystick_invert_yaw=false
joystick_deadzone=0.08
joystick_linear_scale=0.75
joystick_vertical_scale=0.55
joystick_yaw_scale=0.65
```

Contoh mengganti axis:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true \
  joystick_axis_surge:=1 \
  joystick_axis_sway:=0 \
  joystick_axis_heave:=3 \
  joystick_axis_yaw:=2
```

Contoh kalau maju/mundur kebalik:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_invert_surge:=false
```

Contoh mengurangi sensitivitas:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true \
  joystick_linear_scale:=0.45 \
  joystick_vertical_scale:=0.35 \
  joystick_yaw_scale:=0.40
```

Contoh menambah deadzone jika ROV bergerak sendiri karena stick drift:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_deadzone:=0.15
```

Contoh memakai deadman button supaya ROV hanya bergerak saat tombol tertentu ditahan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_enable_button:=4
```

Nomor tombol tergantung stik. Jika tidak yakin, cek event joystick dengan tool Linux seperti `jstest` jika tersedia:

```bash
jstest /dev/input/js0
```

## 5. Jalankan Tanpa GUI untuk Tes Cepat

Untuk validasi cepat atau laptop berat:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false
```

Dengan stik tetapi tanpa GUI:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=true joystick_device:=/dev/input/js0
```

Mode ini cocok untuk mengecek apakah node hidup, topik muncul, dan launch tidak crash.

## 6. Jalankan Dengan Kamera QR

Vision tidak aktif default agar simulasi lebih ringan. Aktifkan dengan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true use_vision:=true
```

Default QR dibaca dari kamera depan/wall karena QR payload menghadap samping. Kalau perlu mengganti sumber kamera, pakai:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true use_vision:=true qr_image_topic:=/rov/camera/wall/image
```

Lihat hasil QR:

```bash
ros2 topic echo /rov/qr_code
```

Lihat kamera depan/wall untuk QR:

```bash
ros2 run rqt_image_view rqt_image_view /rov/camera/wall/image
```

Lihat kamera bawah:

```bash
ros2 run rqt_image_view rqt_image_view /rov/camera/bottom/image
```

Lihat debug QR:

```bash
ros2 run rqt_image_view rqt_image_view /rov/qr_debug/image
```

Jika `rqt_image_view` belum terinstall:

```bash
sudo apt install ros-jazzy-rqt-image-view
```

Cara menggunakan QR:

1. Jalankan launch dengan `use_vision:=true`.
2. Turunkan ROV mendekati payload.
3. Arahkan kamera depan/wall ke QR di sisi payload.
4. Cek `/rov/qr_code`.
5. Jika QR terbaca, huruf `A/B/C/D` akan dipublish.

## 7. Memilih Payload A/B/C/D

Default payload adalah A.

Pilih payload B:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true payload_code:=B
```

Pilih payload C:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true payload_code:=C
```

Pilih payload D:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true payload_code:=D
```

Payload yang dipilih akan dimasukkan ke world sebagai:

```text
kki_payload_A
kki_payload_B
kki_payload_C
kki_payload_D
```

## 8. Memilih Model ROV

Default:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true rov_variant:=github_blue
```

Model lama pembanding:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true rov_variant:=bluerov
```

Penjelasan:

- `github_blue` adalah default karena visual lebih rapi, propeller T200 jelas, dan gripper custom sudah menyatu dengan ROV.
- `bluerov` adalah alternatif lama dari referensi lokal dan tetap bisa dipakai untuk pembanding.

## 9. Kontrol Keyboard Opsional

Keyboard hanya disarankan untuk backup. Masalah utamanya: terminal keyboard harus fokus. Kalau fokus ada di Gazebo, tombol tidak masuk ke node keyboard.

Terminal 1:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py
```

Terminal 2:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rov_gamantaray_control rov_teleop_keyboard
```

Mapping keyboard:

```text
w/s   -> maju/mundur
a/d   -> geser kiri/kanan
r/f   -> naik/turun
q/e   -> yaw kiri/kanan
o     -> buka gripper
p     -> tutup gripper
space -> stop
```

Catatan penting:

- Jangan menjalankan joystick dan keyboard bersamaan jika tidak perlu.
- Joystick dan keyboard sama-sama masuk jalur `/rov/manual_cmd_vel`; kalau keduanya aktif, input manualnya bisa saling menimpa.

## 10. Cara Mengambil Payload Dengan Gripper

Jalankan simulasi:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/js0
```

Pantau status gripper:

```bash
ros2 topic echo /rov/gripper_status
```

Langkah manual:

1. Gerakkan ROV mendekati payload.
2. Turunkan ROV sampai gripper sejajar dengan payload.
3. Arahkan capit ke payload.
4. Cek status:

```text
aligned=true
```

5. Tekan tombol `A` untuk menutup gripper.
6. Payload baru attached jika posisi benar dan jaw sudah menjepit.
7. Bawa payload ke hook tujuan.
8. Tekan tombol `B` untuk membuka gripper dan melepas payload.

Jika payload dilepas cukup dekat dengan hook yang sesuai QR, status akan berubah menjadi `hung`. Pada kondisi ini lubang payload dikunci ke pasak hook dan payload akan berayun kecil lalu mereda, seperti benda yang baru digantung.

Makna status:

```text
aligned=true/false     -> payload sejajar dengan titik capture gripper atau belum
collision=true/false   -> payload sedang kontak dengan body/capit atau tidak
contact=claw           -> kontak dari capit
contact=rov            -> kontak dari body ROV
err=(x,y,z)            -> error posisi payload terhadap titik capture
```

Payload juga bisa bergeser saat ditabrak ROV/capit. Jika ingin melihat efek ini, turunkan ROV ke dekat dasar lalu dorong payload perlahan.

Tuning efek gantung:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true hanging_damping:=2.8 hanging_release_velocity_gain:=0.35 hanging_max_angle_rad:=0.35
```

- `hanging_damping`: makin besar, ayunan makin cepat berhenti.
- `hanging_release_velocity_gain`: makin besar, gerakan ROV saat release lebih kuat membuat payload berayun.
- `hanging_max_angle_rad`: batas maksimum sudut ayunan.

## 11. Mission Autonomous Sederhana

Aktifkan mission supervisor:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py mission_autonomy:=true mission_profile:=full use_vision:=true
```

Profil autonomous yang tersedia:

```text
full                   -> scan QR, ambil, bawa ke hook, release, surface
carry_release_surface  -> sudah membawa payload, lalu ke hook, release, surface
release_surface        -> sudah dekat hook, release payload, lalu surface
```

Untuk misi nomor 5 dari PDF, gunakan `release_surface` jika ROV sudah dekat hook:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py mission_autonomy:=true mission_profile:=release_surface payload_code:=C joystick:=false
```

Untuk skenario manual lalu autonomous dalam satu launch:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true mission_autonomy:=true mission_profile:=release_surface payload_code:=C command_source:=manual
```

Pada command di atas, `mission_supervisor` sudah hidup tetapi menunggu sumber command menjadi `auto`. Bawa ROV dan payload ke dekat hook secara manual, lalu aktifkan autonomous:

```bash
ros2 topic pub --once /rov/command_source std_msgs/msg/String "{data: auto}"
```

Jika ROV sudah menjepit payload tetapi belum berada di hook, gunakan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py mission_autonomy:=true mission_profile:=carry_release_surface payload_code:=C joystick:=false
```

Jika ingin setelah payload terjepit langsung pindah autonomous dan bergerak ke gantungan, jalankan dari awal dengan:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true mission_autonomy:=true mission_profile:=carry_release_surface payload_code:=C command_source:=manual auto_start_on_attached:=true use_vision:=true
```

Alurnya:

1. Operator menggerakkan ROV manual.
2. Operator scan QR dan menjepit payload.
3. Saat `/rov/gripper_status` berubah menjadi `attached`, mission supervisor otomatis mengirim `/rov/command_source = auto`.
4. ROV bergerak menuju hook sesuai `payload_code`.
5. ROV membuka gripper, payload masuk mode `hung`, lalu ROV naik ke permukaan.

Untuk pengujian autonomous murni, pakai `joystick:=false`. Jika joystick tetap aktif, workspace memakai `cmd_vel_mux`: manual masuk ke `/rov/manual_cmd_vel`, autonomous masuk ke `/rov/auto_cmd_vel`, lalu mux meneruskan sumber aktif ke `/rov/cmd_vel`.

Pindah ke autonomous:

```bash
ros2 topic pub --once /rov/command_source std_msgs/msg/String "{data: auto}"
```

Kembali ke manual:

```bash
ros2 topic pub --once /rov/command_source std_msgs/msg/String "{data: manual}"
```

Pantau state:

```bash
ros2 topic echo /rov/mission_state
```

Urutan state:

```text
scan_payload -> pick_payload -> go_to_hook -> release_payload -> surface
```

Pada profil `release_surface`, state langsung dimulai dari:

```text
release_payload -> surface -> complete
```

Catatan:

- Autonomous ini baseline sederhana.
- Belum ada obstacle avoidance.
- Belum ada visual servoing penuh.
- State `release_payload` membaca `/rov/gripper_status`. Kalau gripper melaporkan `hung`, ROV lanjut ke `surface`.
- Cocok untuk kerangka awal pengembangan.

## 12. Mode Hydro Eksperimental

Mode default adalah kinematic. Mode hydro tersedia untuk visual bawah air dengan kontrol tetap responsif:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true physics_mode:=hydro
```

Secara default, command di atas memakai:

```text
hydro_control_mode:=kinematic
```

Artinya world memakai suasana bawah air dan plugin air, tetapi gerak ROV tetap memakai `kinematic_driver`. Ini sengaja dipakai supaya analog stik langsung menggerakkan ROV dan real-time tidak jatuh.

Gunakan mode ini jika ingin eksperimen fisika. Untuk latihan misi dan real-time yang stabil, tetap gunakan mode default:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true physics_mode:=kinematic
```

Mode wrench hydro masih tersedia:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true physics_mode:=hydro \
  hydro_control_mode:=wrench \
  hydro_horizontal_force_gain:=2.00 \
  hydro_vertical_force_gain:=0.80 \
  hydro_yaw_torque_gain:=0.20
```

Nilai yang bisa dituning:

```text
hydro_horizontal_force_gain -> kuat maju/mundur dan geser
hydro_vertical_force_gain   -> kuat naik/turun
hydro_yaw_torque_gain       -> kuat yaw
```

Jika analog maju normal di `hydro_control_mode:=kinematic`, tetapi tidak maju di `hydro_control_mode:=wrench`, berarti masalahnya ada di tuning gaya wrench, buoyancy, damping, dan hydrodynamics, bukan di stik.

Catatan implementasi: mode `wrench` sekarang memakai gaya Gazebo dan `pose_assist_enabled=true` di `hydro_wrench_driver`. Pose assist ini membuat ROV tetap bergerak terlihat saat parameter hidrodinamika belum dikalibrasi penuh.

Batasan hydro:

- perlu tuning massa dan buoyancy,
- real-time bisa lebih berat,
- belum dikalibrasi dengan ROV asli,
- belum menjadi jalur utama lomba.

## 13. Perintah Cek Topik

Lihat semua topic:

```bash
ros2 topic list
```

Cek command dari stik/keyboard:

```bash
ros2 topic echo /rov/manual_cmd_vel
```

Cek command final yang masuk allocator:

```bash
ros2 topic echo /rov/cmd_vel
```

Cek output allocator:

```bash
ros2 topic echo /rov/thruster_status
```

Cek odometry:

```bash
ros2 topic echo /model/gamantaray_rov/odometry
```

Cek gripper:

```bash
ros2 topic echo /rov/gripper_status
```

Cek QR:

```bash
ros2 topic echo /rov/qr_code
```

Cek mission:

```bash
ros2 topic echo /rov/mission_state
```

## 14. Perintah Manual Publish untuk Tes

Tes maju lewat jalur manual:

```bash
ros2 topic pub --rate 10 /rov/manual_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

Tes mundur:

```bash
ros2 topic pub --rate 10 /rov/manual_cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}}"
```

Tes geser kanan:

```bash
ros2 topic pub --rate 10 /rov/manual_cmd_vel geometry_msgs/msg/Twist "{linear: {y: -0.5}}"
```

Tes naik:

```bash
ros2 topic pub --rate 10 /rov/manual_cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.4}}"
```

Tes yaw:

```bash
ros2 topic pub --rate 10 /rov/manual_cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.4}}"
```

Stop manual:

```bash
ros2 topic pub --once /rov/manual_cmd_vel geometry_msgs/msg/Twist "{}"
```

Tutup gripper:

```bash
ros2 topic pub --once /rov/gripper_cmd std_msgs/msg/Float64 "{data: 1.0}"
```

Buka gripper:

```bash
ros2 topic pub --once /rov/gripper_cmd std_msgs/msg/Float64 "{data: 0.0}"
```

## 15. Troubleshooting

### Stik tidak terbaca

Cek device:

```bash
ls -l /dev/input/js*
ls -l /dev/input/by-id/
```

Coba jalankan dengan path yang benar:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/js0
```

Jika permission ditolak, cek group user:

```bash
groups
```

Biasanya device input butuh akses group seperti `input`. Jika perlu, logout/login setelah menambah group.

### ROV tidak bergerak

Cek apakah `/rov/manual_cmd_vel` berubah:

```bash
ros2 topic echo /rov/manual_cmd_vel
```

Cek sumber command aktif:

```bash
ros2 topic echo /rov/active_command_source
```

Cek apakah allocator menghasilkan thruster:

```bash
ros2 topic echo /rov/thruster_status
```

Cek apakah odometry berubah:

```bash
ros2 topic echo /model/gamantaray_rov/odometry
```

Jika `/rov/manual_cmd_vel` nol terus:

- stik belum terbaca,
- axis salah,
- deadzone terlalu besar,
- ada deadman button yang belum ditekan.

Jika `/rov/cmd_vel` berubah tetapi ROV tidak bergerak:

- cek `kinematic_driver` hidup dengan `ros2 node list`,
- cek `/rov/thruster_status`,
- restart launch.

### Keyboard tidak bergerak saat Gazebo difokuskan

Itu normal. Keyboard teleop membaca input dari terminal. Fokus harus berada di terminal `rov_teleop_keyboard`. Untuk menghindari masalah ini, pakai stik.

### Payload tidak terambil

Cek:

```bash
ros2 topic echo /rov/gripper_status
```

Payload hanya attached jika:

- `aligned=true`,
- gripper ditutup,
- jaw gap cukup kecil,
- payload berada di depan capit, bukan sekadar dekat body.

Jika `aligned=false`, gerakkan ROV sampai `err=(x,y,z)` kecil.

### Payload tidak bergeser saat ditabrak

Pastikan ROV sudah dekat dasar. Payload berada di dasar kolam, sedangkan ROV awalnya lebih tinggi. Turunkan ROV dulu memakai heave down.

Cek status:

```bash
ros2 topic echo /rov/gripper_status
```

Jika kontak benar, akan muncul:

```text
collision=true contact=claw
```

atau:

```text
collision=true contact=rov
```

### QR tidak terbaca

Jalankan dengan vision:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true use_vision:=true
```

Cek kamera QR:

```bash
ros2 run rqt_image_view rqt_image_view /rov/camera/wall/image
```

Cek hasil:

```bash
ros2 topic echo /rov/qr_code
```

Penyebab umum:

- kamera depan/wall belum menghadap QR samping payload,
- ROV terlalu jauh,
- sudut terlalu miring,
- `use_vision` belum true.

### Gazebo terasa berat

Pakai mode tanpa vision:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true use_vision:=false
```

Atau headless:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false
```

### Simulasi terasa aneh setelah beberapa kali launch

Matikan proses lama:

```bash
pkill -f gz
pkill -f ros2
```

Lalu source ulang dan jalankan lagi:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true
```

## 16. Urutan Run yang Disarankan Untuk Latihan

Urutan paling aman:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/js0 use_vision:=true payload_code:=A
```

Terminal tambahan untuk monitor:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ammar/Documents/WS_ROV/install/setup.bash
ros2 topic echo /rov/gripper_status
```

Terminal kamera:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ammar/Documents/WS_ROV/install/setup.bash
ros2 run rqt_image_view rqt_image_view /rov/camera/bottom/image
```

Untuk QR payload KKI, pakai kamera depan/wall:

```bash
ros2 run rqt_image_view rqt_image_view /rov/camera/wall/image
```

Dengan urutan ini, operator bisa:

1. menggerakkan ROV dengan stik,
2. melihat QR dari kamera depan/wall,
3. mengecek alignment gripper,
4. mengambil payload,
5. memindahkan payload ke hook,
6. melepas payload.

## 17. Perintah Validasi Setelah Edit Kode

Validasi Python:

```bash
python3 -m py_compile src/rov_gamantaray_control/rov_gamantaray_control/*.py
python3 -m py_compile src/rov_gamantaray_vision/rov_gamantaray_vision/*.py
```

Validasi SDF/XML:

```bash
xmllint --noout src/rov_gamantaray_gazebo/worlds/kki_rov_pool.template.sdf
xmllint --noout src/rov_gamantaray_gazebo/models/kki_payload_A/model.sdf
```

Build:

```bash
colcon build --symlink-install
```

Smoke test headless:

```bash
timeout 12s ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false
```

Exit code `124` dari command `timeout` normal jika proses dihentikan oleh timeout. Yang penting tidak ada error Python atau crash sebelum timeout.

## 18. Ringkasan Command Utama

Paling sering dipakai:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/js0
```

Dengan vision:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/js0 use_vision:=true
```

Payload C:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py joystick:=true joystick_device:=/dev/input/js0 payload_code:=C
```

Autonomous:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py mission_autonomy:=true use_vision:=true
```

Headless test:

```bash
ros2 launch rov_gamantaray_bringup kki_rov_sim.launch.py gui:=false joystick:=false use_vision:=false
```
