# README Setup ROV Asli

Dokumen ini menjelaskan setup ROV asli untuk workspace `WS_ROV`. Fokusnya adalah cara membawa kontrol dari ROS 2 ke hardware nyata: stik, GUI, QR, PWM thruster, gripper, kamera, sensor, dan prosedur uji aman.

Workspace simulasi tetap berguna untuk latihan misi. Untuk ROV asli, bagian Gazebo tidak dipakai. Yang dipakai adalah jalur ROS sampai output PWM, lalu PWM dikirim ke mikrokontroler untuk mengendalikan ESC dan thruster.

Kode siap pakai untuk ROV asli sudah dipisahkan di:

```text
setup_asli_ROV/
```

Isi folder tersebut:

- `setup_asli_ROV/gcs`: kode untuk laptop ground station/operator.
- `setup_asli_ROV/onboard`: kode untuk komputer onboard di ROV.
- `setup_asli_ROV/hardware`: node ROS serial PWM ke mikrokontroler.
- `setup_asli_ROV/firmware`: firmware Arduino dan contoh STM32.
- `setup_asli_ROV/common`: konfigurasi environment dan cek topic.

## 1. Arsitektur Sistem

Alur kontrol ROV asli:

```text
Stik / autonomous
        |
        v
/rov/manual_cmd_vel atau /rov/auto_cmd_vel
        |
        v
cmd_vel_mux
        |
        v
/rov/cmd_vel
        |
        v
thruster_allocator
        |
        v
/rov/thruster_pwm
        |
        v
serial_pwm_driver
        |
        v
STM32 / Arduino / ESP32 / PCA9685 / Pixhawk
        |
        v
ESC
        |
        v
Thruster asli
```

Stik tidak langsung menggerakkan ESC. Stik masuk ke ROS 2, ROS membuat nilai PWM, lalu mikrokontroler mengubah data serial dari ROS menjadi sinyal PWM ke ESC.

## 1.1. Stik Di Ground Station Menggerakkan ROV Bagaimana?

Kalau stik ada di ground station/laptop operator, ROV tetap bisa bergerak karena command stik dikirim lewat jaringan ROS 2 melalui tether Ethernet.

Arsitektur yang disarankan:

```text
GROUND STATION / LAPTOP OPERATOR
  - stik Xbox/gamepad
  - rov_joystick
  - kki_dashboard / GUI
  - rqt_image_view jika perlu
        |
        |  ROS 2 DDS lewat tether Ethernet
        v
ONBOARD COMPUTER DI ROV
  - cmd_vel_mux
  - thruster_allocator
  - serial_pwm_driver
  - camera driver
  - sensor driver
        |
        |  USB serial
        v
STM32 / Arduino / ESP32
        |
        |  PWM signal
        v
ESC + thruster
```

Jadi stik tidak perlu dicolok ke ROV. Stik dicolok ke laptop ground station. Node `rov_joystick` di laptop publish `/rov/manual_cmd_vel`. Karena laptop dan komputer onboard berada dalam satu jaringan ROS 2, komputer onboard menerima topic itu, lalu menjalankan `cmd_vel_mux`, `thruster_allocator`, dan `serial_pwm_driver`.

### Setup Jaringan Ground Station Dan Onboard

Gunakan tether Ethernet. Contoh IP statis:

```text
Ground station laptop : 192.168.10.1
Onboard computer ROV  : 192.168.10.2
Netmask               : 255.255.255.0
```

Cek koneksi:

```bash
ping 192.168.10.2
```

Di kedua komputer, pakai domain ROS yang sama:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Kalau memakai terminal baru, export ini harus diulang atau dimasukkan ke `~/.bashrc`.

### Jalankan Di Onboard Computer ROV

Di komputer yang ada di ROV, jalankan node hardware. Stik dan GUI dimatikan di sini karena ada di ground station.

Mode aman dry-run:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=false kki_gui:=false hardware_dry_run:=true
```

Mode kirim ke STM32/ESC:

```bash
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=false kki_gui:=false \
  hardware_dry_run:=false serial_port:=/dev/ttyACM0
```

### Jalankan Di Ground Station

Di laptop operator, jalankan joystick:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 run rov_gamantaray_control rov_joystick --ros-args \
  -p device_path:=/dev/input/js0 \
  -p enable_button:=4 \
  -p linear_scale:=0.30 \
  -p vertical_scale:=0.25 \
  -p yaw_scale:=0.25
```

Jalankan GUI di ground station:

```bash
ros2 run rov_gamantaray_control kki_dashboard
```

Jika kamera onboard publish ke `/rov/camera/wall/image` dan `/rov/camera/bottom/image`, GUI di ground station akan menerima gambar lewat ROS 2 network.

### Cek Topic Antar Komputer

Di ground station:

```bash
ros2 topic echo /rov/manual_cmd_vel
```

Di onboard:

```bash
ros2 topic echo /rov/manual_cmd_vel
ros2 topic echo /rov/thruster_pwm
ros2 topic echo /rov/hardware_pwm_status
```

Jika `/rov/manual_cmd_vel` muncul di ground station tetapi tidak muncul di onboard, masalahnya ada di jaringan ROS 2, bukan stik.

### Ringkasan Jalur Command

```text
Stik di laptop
  -> /rov/manual_cmd_vel
  -> lewat tether Ethernet
  -> onboard computer
  -> /rov/cmd_vel
  -> /rov/thruster_pwm
  -> serial ke STM32
  -> PWM ke ESC
  -> thruster bergerak
```

## 2. Hardware Yang Diperlukan

Minimal hardware:

- Komputer onboard: mini PC, Jetson, Raspberry Pi 5, atau laptop kecil.
- Mikrokontroler / PWM controller: STM32, Arduino, ESP32, PCA9685, atau Pixhawk.
- 6 ESC untuk 6 thruster.
- 6 thruster.
- Gripper/capit.
- Servo atau aktuator gripper.
- 2 kamera:
  - kamera depan untuk QR,
  - kamera bawah untuk observasi payload/dasar kolam.
- IMU.
- Pressure/depth sensor.
- Leak sensor.
- Sensor tegangan dan arus.
- Tether komunikasi, biasanya Ethernet.
- Baterai atau power dari permukaan.
- Regulator 5 V untuk komputer/mikrokontroler.
- Regulator atau BEC untuk servo.
- Fuse.
- Emergency stop.
- Enclosure kedap air.
- Penetrator kabel.
- O-ring/gasket.
- Strain relief untuk tether.

Untuk uji awal, cukup mulai dari:

- komputer ROS 2,
- mikrokontroler,
- 1 ESC,
- 1 thruster,
- power supply aman,
- emergency stop.

Jangan langsung uji 6 thruster sekaligus.

## 3. Wiring Dasar Thruster

Setiap thruster dikendalikan oleh ESC.

```text
Baterai +  -> ESC power +
Baterai -  -> ESC power -
ESC output 3 phase -> thruster
STM32 PWM pin -> ESC signal
STM32 GND -> ESC GND
```

Catatan penting:

- STM32 tidak memberi daya ke thruster.
- STM32 hanya memberi sinyal PWM.
- ESC yang memberi daya ke thruster.
- GND STM32 dan GND ESC harus tersambung.
- Jalur power thruster harus lewat fuse atau proteksi.

## 4. Sinyal PWM ESC

Umumnya ESC ROV memakai sinyal servo PWM:

```text
1500 us = stop / netral
>1500 us = maju
<1500 us = mundur
1100 us = full reverse
1900 us = full forward
```

Untuk uji awal jangan pakai full range. Pakai batas kecil:

```text
1450 us = pelan mundur
1500 us = stop
1550 us = pelan maju
```

Launch `real_rov.launch.py` default memakai batas aman:

```text
pwm_min_us = 1400
pwm_max_us = 1600
pwm_neutral_us = 1500
```

Setelah arah dan fail-safe benar, batas PWM bisa dinaikkan perlahan.

## 5. Package Hardware Di Workspace

Package hardware:

```text
src/rov_gamantaray_hardware/
```

Node utama:

```text
serial_pwm_driver
```

Input node:

```text
/rov/thruster_pwm
/rov/gripper_cmd
```

Output serial ke mikrokontroler:

```text
PWM,t1,t2,t3,t4,t5,t6,gripper
```

Contoh:

```text
PWM,1500,1500,1500,1500,1500,1500,1100
```

Arti data:

```text
t1 sampai t6 = PWM thruster 1 sampai 6
gripper      = PWM servo gripper
```

Firmware contoh Arduino:

```text
src/rov_gamantaray_hardware/firmware/arduino_serial_pwm_bridge/arduino_serial_pwm_bridge.ino
```

Untuk STM32, logikanya sama: baca serial, parse format `PWM,...`, lalu tulis PWM microsecond ke timer PWM.

## 6. Install Dan Build Workspace

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Jika `python3-serial` belum ada:

```bash
sudo apt install python3-serial
```

## 7. Upload Firmware Mikrokontroler

Jika memakai Arduino:

1. Buka Arduino IDE.
2. Buka file:

```text
src/rov_gamantaray_hardware/firmware/arduino_serial_pwm_bridge/arduino_serial_pwm_bridge.ino
```

3. Pilih board dan port.
4. Upload.

Jika memakai STM32:

1. Buat project STM32CubeIDE.
2. Aktifkan UART USB/serial.
3. Aktifkan 7 output PWM:
   - 6 untuk thruster,
   - 1 untuk gripper.
4. Buat parser serial untuk format:

```text
PWM,t1,t2,t3,t4,t5,t6,gripper
```

5. Set timer PWM dengan periode 20 ms atau 50 Hz.
6. Ubah duty supaya pulse width sesuai microsecond 1100 sampai 1900 us.
7. Tambahkan watchdog: jika serial tidak masuk selama 500 ms, semua thruster kembali `1500 us`.

## 8. Cek Port Serial

Hubungkan mikrokontroler ke komputer, lalu cek:

```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

Contoh port:

```text
/dev/ttyACM0
/dev/ttyUSB0
```

Jika tidak punya permission:

```bash
sudo usermod -a -G dialout $USER
```

Setelah itu logout/login ulang.

## 9. Test Aman Tanpa Mengirim Ke ESC

Jalankan dry-run dulu. Mode ini tidak membuka serial dan tidak mengirim ke ESC.

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=true kki_gui:=true hardware_dry_run:=true
```

Cek output PWM:

```bash
ros2 topic echo /rov/thruster_pwm
```

Cek status hardware driver:

```bash
ros2 topic echo /rov/hardware_pwm_status
```

Saat stik dilepas, PWM harus kembali ke sekitar:

```text
1500,1500,1500,1500,1500,1500
```

## 10. Test Serial Ke Mikrokontroler

Jika mikrokontroler sudah tersambung dan firmware sudah siap, jalankan:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=true kki_gui:=true \
  hardware_dry_run:=false serial_port:=/dev/ttyACM0
```

Ganti `/dev/ttyACM0` sesuai port yang terbaca.

Untuk tahap ini, sebaiknya ESC belum dipasang ke thruster, atau propeller belum dipasang. Tujuannya hanya memastikan mikrokontroler menerima data serial.

## 11. Jalankan Dengan Deadman Button

Default `real_rov.launch.py` memakai:

```text
joystick_enable_button = 4
```

Artinya ROV hanya bergerak kalau tombol itu ditahan. Jika tombol di stik berbeda:

```bash
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=true \
  joystick_enable_button:=5 hardware_dry_run:=true
```

Cari nomor tombol dengan:

```bash
jstest /dev/input/js0
```

Jika `jstest` belum ada:

```bash
sudo apt install joystick
```

## 12. Test ESC Dan Thruster Satu Per Satu

Urutan aman:

1. Lepas propeller jika memungkinkan.
2. Sambungkan satu ESC dan satu thruster.
3. Pastikan ESC menerima `1500 us` saat idle.
4. Gerakkan stik sedikit.
5. Cek arah dorong.
6. Catat thruster nomor berapa yang bergerak.
7. Ulangi untuk semua thruster.

Jangan langsung uji semua thruster sekaligus.

Jika arah thruster salah:

- balik konfigurasi ESC,
- ubah arah motor jika ESC mendukung,
- atau ubah mapping output thruster di software/mikrokontroler.

Jangan membalik joystick sebagai solusi utama, karena itu bisa membuat mapping misi menjadi membingungkan.

## 13. Mapping Thruster

Output `/rov/thruster_pwm` berisi 6 nilai:

```text
[thruster1, thruster2, thruster3, thruster4, thruster5, thruster6]
```

Allocator saat ini memakai logika:

```text
thruster 1-4 = gerak horizontal dan yaw
thruster 5-6 = naik/turun
```

Saat uji hardware, pastikan:

- perintah maju membuat ROV maju,
- perintah geser membuat ROV geser,
- perintah yaw membuat ROV berputar,
- perintah naik membuat ROV naik,
- perintah turun membuat ROV turun.

Jika ROV berputar saat harus maju, berarti mapping atau arah salah.

## 14. Gripper Nyata

Topic gripper:

```text
/rov/gripper_cmd
```

Makna:

```text
0.0 = buka
1.0 = tutup
```

`serial_pwm_driver` mengubah nilai itu menjadi PWM gripper:

```text
0.0 -> gripper_open_pwm_us
1.0 -> gripper_closed_pwm_us
```

Default:

```text
open   = 1100 us
closed = 1900 us
```

Tes manual:

```bash
ros2 topic pub --once /rov/gripper_cmd std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic pub --once /rov/gripper_cmd std_msgs/msg/Float64 "{data: 1.0}"
```

Atur batas servo agar tidak stall. Servo yang stall bisa panas dan rusak.

## 15. Kamera Nyata

Target topic agar GUI dan QR langsung bekerja:

```text
/rov/camera/wall/image
/rov/camera/bottom/image
```

Jika memakai kamera USB, salah satu opsi adalah `v4l2_camera`.

Install:

```bash
sudo apt install ros-jazzy-v4l2-camera
```

Contoh kamera depan:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -r /image_raw:=/rov/camera/wall/image
```

Contoh kamera bawah:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video1 \
  -r /image_raw:=/rov/camera/bottom/image
```

Cek kamera:

```bash
ros2 run rqt_image_view rqt_image_view
```

## 16. Deteksi QR Di Hardware

Jalankan QR detector:

```bash
ros2 run rov_gamantaray_vision qr_detector --ros-args \
  -p image_topic:=/rov/camera/wall/image
```

Cek hasil:

```bash
ros2 topic echo /rov/qr_code
```

Hal yang perlu diperhatikan:

- kamera harus fokus pada jarak payload,
- lampu jangan terlalu memantul ke QR,
- exposure kamera sebaiknya stabil,
- QR 4 cm x 4 cm harus cukup besar di gambar kamera,
- uji di air, bukan hanya di udara.

## 17. Depth Sensor Dan Altitude

GUI butuh tinggi ROV dari dasar kolam. Di hardware nyata, gunakan pressure sensor.

Rumus:

```text
altitude = kedalaman_kolam - depth_sensor
```

Contoh:

```text
kedalaman kolam = 0.85 m
depth sensor    = 0.20 m
altitude        = 0.65 m dari dasar
```

Untuk tahap awal, driver depth bisa publish ke topic yang sudah dibaca GUI:

```text
/model/gamantaray_rov/odometry
```

Untuk jangka panjang, lebih rapi memakai:

```text
/rov/odometry
/rov/depth
```

Lalu GUI diremap atau param `odom_topic` diganti.

## 18. Launch ROV Asli

Launch yang sudah disiapkan:

```text
src/rov_gamantaray_bringup/launch/real_rov.launch.py
```

Mode aman:

```bash
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=true kki_gui:=true hardware_dry_run:=true
```

Mode kirim serial ke mikrokontroler:

```bash
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=true kki_gui:=true \
  hardware_dry_run:=false serial_port:=/dev/ttyACM0
```

Dengan kamera dan QR:

```bash
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=true kki_gui:=true \
  use_vision:=true hardware_dry_run:=false serial_port:=/dev/ttyACM0
```

## 19. Checklist Sebelum Masuk Air

- Enclosure sudah leak test.
- Tidak ada embun/air di enclosure.
- Fuse terpasang.
- Emergency stop aktif.
- ESC idle di `1500 us`.
- Saat ROS mati, ESC kembali netral.
- Deadman button aktif.
- Kamera depan muncul di GUI.
- Kamera bawah muncul di GUI.
- QR terbaca.
- Gripper buka/tutup tanpa stall.
- Leak sensor terbaca.
- Tegangan baterai terbaca.
- Tether punya strain relief.
- Tether tidak bisa masuk propeller.
- Operator tahu cara stop.

## 20. Urutan Uji Kolam

1. Uji elektronik tanpa masuk air.
2. Leak test enclosure tanpa elektronik.
3. Uji kamera dan sensor di udara.
4. Uji satu thruster dengan PWM kecil.
5. Uji semua thruster dengan PWM kecil.
6. Masukkan ROV ke air tanpa misi.
7. Cek buoyancy.
8. Cek maju/mundur.
9. Cek geser kiri/kanan.
10. Cek naik/turun.
11. Cek yaw.
12. Cek gripper di air.
13. Cek QR di air.
14. Baru uji misi payload.

## 21. Troubleshooting

### Serial port tidak muncul

Cek:

```bash
lsusb
dmesg | tail -50
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

Pastikan kabel USB data, bukan kabel charge-only.

### Permission denied saat buka serial

```bash
sudo usermod -a -G dialout $USER
```

Logout/login ulang.

### PWM tidak berubah

Cek:

```bash
ros2 topic echo /rov/manual_cmd_vel
ros2 topic echo /rov/cmd_vel
ros2 topic echo /rov/thruster_pwm
```

Jika `/rov/manual_cmd_vel` nol terus, stik belum terbaca atau deadman button belum ditekan.

### Thruster bergerak saat harus diam

Kemungkinan:

- ESC belum kalibrasi neutral,
- sinyal PWM bukan 1500 us,
- ground tidak common,
- noise power,
- firmware tidak menjalankan watchdog.

### ROV maju tapi belok

Kemungkinan:

- arah thruster ada yang terbalik,
- mapping channel salah,
- posisi thruster tidak simetris,
- satu ESC tidak merespon sama.

### QR tidak terbaca

Kemungkinan:

- kamera blur,
- QR terlalu kecil di frame,
- lampu memantul,
- exposure berubah-ubah,
- topic kamera salah.

## 22. Batas Klaim

Simulasi boleh diklaim untuk:

- validasi alur misi,
- validasi integrasi ROS 2,
- validasi GUI,
- validasi QR,
- validasi mapping command ke PWM,
- latihan operator.

Jangan klaim simulasi sudah sama persis dengan hardware nyata untuk:

- gaya thruster,
- drag air,
- buoyancy,
- efek tether,
- kemampuan gripper menjepit payload asli.

Kalimat aman untuk laporan:

> Simulasi digunakan untuk memvalidasi alur misi, integrasi ROS 2, pembacaan QR, kontrol joystick, pemetaan command ke PWM thruster, tampilan GUI operator, dan urutan manipulasi payload sebelum implementasi ke ROV nyata.

Kalimat tambahan:

> Nilai PWM dan respon gerak pada hardware harus dikalibrasi ulang melalui uji kolam karena massa, buoyancy, drag, dan efek tether pada ROV nyata berbeda dari model simulasi.
