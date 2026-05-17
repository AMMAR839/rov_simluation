# Migrasi Workspace Simulasi Ke ROV Nyata

Dokumen ini menjelaskan cara membawa workspace `WS_ROV` dari simulasi Gazebo ke sistem ROV nyata. Intinya: bagian operator, joystick, QR, GUI, command mux, dan allocator PWM bisa dipertahankan. Bagian Gazebo harus diganti dengan driver hardware yang benar-benar mengirim PWM ke ESC, membaca sensor asli, dan mengambil gambar dari kamera asli.

## 1. Prinsip Sim-To-Real

Jalur kontrol di simulasi sekarang:

```text
Joystick / autonomous
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
/rov/thruster1..6/pwm dan /rov/thruster_pwm
        |
        v
Gazebo driver / visual propeller
```

Pada ROV nyata, jalurnya menjadi:

```text
Joystick / autonomous
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
MCU / PCA9685 / flight controller
        |
        v
ESC + thruster asli
```

Jadi yang diganti bukan logika joystick atau GUI, tetapi output akhir yang saat ini masuk ke Gazebo.

## 2. Bagian Workspace Yang Bisa Dipakai Langsung

Bagian berikut masih relevan untuk sistem nyata:

- `rov_joystick`: membaca stik Xbox/gamepad dan publish `/rov/manual_cmd_vel`.
- `cmd_vel_mux`: memilih command manual atau autonomous.
- `thruster_allocator`: mengubah `/rov/cmd_vel` menjadi PWM 6 thruster di `/rov/thruster_pwm`.
- `qr_detector`: membaca QR dari topic kamera dan publish `/rov/qr_code`.
- `kki_dashboard`: GUI operator untuk 2 kamera, QR, altitude, trajectory, status, tim, dan universitas.
- `mission_supervisor`: bisa dipakai sebagai awal autonomous, tetapi harus diuji ulang dengan sensor nyata.

Bagian yang khusus simulasi dan tidak dipakai langsung di ROV nyata:

- `kinematic_driver`
- `hydro_wrench_driver`
- `gripper_manager` versi Gazebo
- `water_effects_driver`
- `tether_driver` visual Gazebo
- `ros_gz_bridge`
- model/world SDF Gazebo

## 3. Hardware Yang Perlu Disiapkan

Minimal hardware untuk ROV nyata:

1. Komputer onboard
   - Raspberry Pi 5, Jetson, mini PC, atau SBC lain yang kuat menjalankan ROS 2.
   - Disarankan menjalankan ROS 2 Jazzy agar sama dengan workspace ini.

2. Mikrokontroler / PWM controller
   - Arduino, STM32, ESP32, Pixhawk, atau PCA9685.
   - Tugasnya menerima command dari ROS lalu mengeluarkan PWM stabil ke ESC dan servo gripper.

3. Thruster dan ESC
   - 6 thruster sesuai konfigurasi workspace.
   - Setiap ESC harus punya supply, ground common dengan controller, dan sinyal PWM.
   - Nilai awal workspace: `1500 us` netral, `1100 us` minimum, `1900 us` maksimum. Sesuaikan dengan datasheet ESC yang dipakai.

4. Gripper nyata
   - Servo atau aktuator linear.
   - Driver harus subscribe `/rov/gripper_cmd`.
   - Nilai `0.0` berarti buka, `1.0` berarti tutup.

5. Kamera
   - Minimal 2 kamera: depan/wall untuk QR, bawah untuk observasi dasar.
   - Publish ke:
     - `/rov/camera/wall/image`
     - `/rov/camera/bottom/image`
   - Jika memakai kamera USB, bisa pakai node seperti `v4l2_camera`.

6. Sensor
   - IMU untuk orientasi.
   - Pressure/depth sensor untuk kedalaman.
   - Leak sensor untuk keamanan.
   - Sensor arus/tegangan baterai untuk monitoring.

7. Tether/kabel
   - Kabel komunikasi, biasanya Ethernet.
   - Kabel power jika power dari permukaan, atau tether komunikasi saja jika baterai onboard.
   - Strain relief wajib agar kabel tidak menarik konektor elektronik.

8. Power dan proteksi
   - Fuse utama.
   - Fuse per jalur thruster jika memungkinkan.
   - Emergency stop.
   - Regulator terpisah untuk komputer, servo, dan ESC.
   - Grounding harus dirancang rapi untuk mengurangi noise.

9. Mekanik kedap air
   - Enclosure elektronik.
   - Penetrator kabel.
   - O-ring/gasket.
   - Uji kebocoran sebelum elektronik aktif di air.

## 4. Node Hardware Yang Sudah Disiapkan

Workspace ini sudah memiliki starter package hardware:

```text
src/rov_gamantaray_hardware/
```

Node yang sudah tersedia:

```text
serial_pwm_driver
  subscribe: /rov/thruster_pwm
  subscribe: /rov/gripper_cmd
  output   : serial CSV ke mikrokontroler
  protocol : PWM,t1,t2,t3,t4,t5,t6,gripper
```

Firmware contoh Arduino tersedia di:

```text
src/rov_gamantaray_hardware/firmware/arduino_serial_pwm_bridge/arduino_serial_pwm_bridge.ino
```

Node yang masih perlu dibuat sesuai sensor nyata:

```text

depth_sensor_driver
  publish  : /rov/depth
  publish  : /model/gamantaray_rov/odometry atau topic odom nyata

camera_driver_wall
  publish  : /rov/camera/wall/image

camera_driver_bottom
  publish  : /rov/camera/bottom/image

health_monitor
  publish  : status baterai, leak sensor, suhu, dan arus
```

Untuk tahap awal, yang paling penting adalah `serial_pwm_driver`, firmware mikrokontroler, dan kamera.

## 5. Mapping Topic Simulasi Ke Hardware

| Fungsi | Topic Saat Ini | Di Sistem Nyata |
|---|---|---|
| Command manual joystick | `/rov/manual_cmd_vel` | Tetap dipakai |
| Command autonomous | `/rov/auto_cmd_vel` | Tetap dipakai |
| Command aktif | `/rov/cmd_vel` | Tetap dipakai |
| PWM semua thruster | `/rov/thruster_pwm` | Dibaca driver hardware |
| PWM per thruster | `/rov/thruster1/pwm` sampai `/rov/thruster6/pwm` | Bisa untuk debug |
| Gripper | `/rov/gripper_cmd` | Dibaca driver servo |
| Kamera depan | `/rov/camera/wall/image` | Publish dari kamera nyata |
| Kamera bawah | `/rov/camera/bottom/image` | Publish dari kamera nyata |
| QR hasil deteksi | `/rov/qr_code` | Tetap dari `qr_detector` |
| GUI | `kki_dashboard` | Tetap dipakai |
| Odometry/altitude | `/model/gamantaray_rov/odometry` | Ganti dari estimator/sensor nyata atau remap |

Catatan: nama `/model/gamantaray_rov/odometry` berasal dari Gazebo. Untuk ROV nyata lebih rapi memakai topic baru seperti `/rov/odometry`, lalu GUI dan node lain diremap atau param `odom_topic` diganti. Untuk kompatibilitas cepat, driver nyata boleh publish sementara ke `/model/gamantaray_rov/odometry`.

## 6. Launch Yang Disarankan Untuk Sistem Nyata

Jangan menjalankan Gazebo saat hardware nyata aktif. Launch awal untuk sistem nyata sudah disiapkan:

```text
src/rov_gamantaray_bringup/launch/real_rov.launch.py
```

Isi launch nyata sebaiknya menjalankan:

- `rov_joystick`
- `cmd_vel_mux`
- `thruster_allocator`
- `serial_pwm_driver`
- driver kamera depan
- driver kamera bawah
- `qr_detector`
- `kki_dashboard`
- sensor depth/IMU
- health monitor

Tidak menjalankan:

- `gz sim`
- `ros_gz_bridge`
- `kinematic_driver`
- `gripper_manager` Gazebo
- `water_effects_driver`
- `tether_driver` visual

Jalankan mode aman dry-run dulu:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=true kki_gui:=true hardware_dry_run:=true
```

Jika mikrokontroler sudah tersambung dan ESC aman untuk diuji:

```bash
ros2 launch rov_gamantaray_bringup real_rov.launch.py joystick:=true kki_gui:=true \
  hardware_dry_run:=false serial_port:=/dev/ttyACM0
```

Mode real serial mengirim baris seperti ini ke mikrokontroler:

```text
PWM,1500,1500,1500,1500,1500,1500,1100
```

## 7. Kalibrasi Thruster Dan PWM

Urutan kalibrasi yang disarankan:

1. Lepas propeller atau pastikan thruster berada dalam kondisi uji yang aman sesuai datasheet.
2. Cek setiap ESC menerima sinyal netral `1500 us`.
3. Cek deadband. Di workspace default `pwm_deadband_us:=25.0`.
4. Cek arah tiap thruster satu per satu.
5. Jika arah terbalik, jangan langsung ubah joystick. Perbaiki mapping thruster atau arah motor/ESC.
6. Batasi output awal, misalnya jangan langsung pakai `1100-1900 us`; mulai dari rentang kecil sekitar `1450-1550 us`.
7. Setelah stabil, naikkan perlahan.

Parameter penting dari launch/simulasi yang relevan untuk hardware:

```bash
pwm_neutral_us:=1500.0
pwm_min_us:=1100.0
pwm_max_us:=1900.0
pwm_deadband_us:=25.0
pwm_response_s:=0.04
```

Untuk uji awal di hardware, disarankan memakai batas lebih aman:

```bash
pwm_min_us:=1400.0 pwm_max_us:=1600.0
```

Setelah arah dan respon benar, baru lebarkan rentang.

## 8. Kalibrasi Joystick

Joystick tetap bisa memakai node yang sama:

```bash
ros2 run rov_gamantaray_control rov_joystick --ros-args \
  -p device_path:=/dev/input/js0
```

Parameter penting:

```bash
joystick_deadzone:=0.08
joystick_linear_scale:=0.75
joystick_vertical_scale:=0.55
joystick_yaw_scale:=0.65
joystick_enable_button:=4
```

Untuk sistem nyata, sangat disarankan memakai deadman button:

```bash
joystick_enable_button:=4
```

Artinya ROV hanya bergerak saat tombol tertentu ditahan. Ini lebih aman daripada joystick selalu aktif.

## 9. Kamera Dan QR Di ROV Nyata

Target topic supaya `qr_detector` dan GUI langsung bekerja:

```text
/rov/camera/wall/image
/rov/camera/bottom/image
```

Jika kamera nyata publish ke topic lain, ada dua opsi:

1. Remap topic kamera ke nama di atas.
2. Ubah parameter `image_topic` pada `qr_detector`.

Contoh:

```bash
ros2 run rov_gamantaray_vision qr_detector --ros-args \
  -p image_topic:=/camera_front/image_raw
```

Hal yang harus disiapkan agar QR terbaca di air:

- kamera fokus pada jarak kerja payload,
- lampu depan yang tidak membuat pantulan berlebihan,
- exposure kamera dikunci jika auto exposure membuat QR berkedip,
- resolusi cukup untuk QR 4 cm x 4 cm,
- uji di air, bukan hanya di udara.

## 10. Gripper Nyata

Di simulasi, `/rov/gripper_cmd` memakai:

```text
0.0 = buka
1.0 = tutup
```

Di hardware, node gripper harus mengubah nilai itu menjadi PWM servo atau command aktuator. Contoh mapping:

```text
0.0 -> 1100 us atau posisi buka
1.0 -> 1900 us atau posisi tutup
```

Nilai servo tidak boleh langsung diasumsikan. Harus diuji:

- posisi buka tidak menabrak frame,
- posisi tutup tidak memaksa servo stall,
- arus servo aman,
- gripper bisa menjepit payload tanpa merusak payload,
- payload bisa dilepas saat di hook.

## 11. Depth, Altitude, Dan GUI

GUI sekarang menghitung altitude dari odometry:

```text
altitude = z_rov - z_dasar_kolam
```

Di hardware nyata, lebih aman memakai pressure sensor untuk kedalaman. Untuk menampilkan altitude dari dasar kolam, perlu:

```text
altitude = kedalaman_kolam - depth_sensor
```

Jika kolam 0.85 m dan ROV berada 0.20 m di bawah permukaan:

```text
altitude = 0.85 - 0.20 = 0.65 m dari dasar
```

Jadi untuk hardware, disarankan membuat node `depth_to_odometry` atau mengubah GUI agar membaca `/rov/depth` langsung.

## 12. Tether/Kabel Nyata

Tether nyata minimal membawa:

- komunikasi Ethernet,
- power jika power dari permukaan,
- strain relief mekanik,
- pelindung agar tidak masuk propeller.

Model tether di simulasi sekarang hanya visual dan estimasi tension. Untuk ROV nyata, yang penting bukan visualnya, tetapi:

- kabel tidak tertarik langsung ke konektor,
- kabel tidak masuk propeller,
- panjang kabel cukup,
- ada operator khusus tether jika memungkinkan,
- ada prosedur emergency stop jika kabel tersangkut.

## 13. Urutan Uji Bertahap

Jangan langsung uji semua sistem di kolam. Gunakan tahap berikut.

### Tahap 1 - Uji ROS Tanpa Hardware Aktuator

Tujuan: memastikan joystick, GUI, QR, dan topic jalan.

```bash
ros2 topic echo /rov/manual_cmd_vel
ros2 topic echo /rov/thruster_pwm
ros2 topic echo /rov/gripper_cmd
```

Pastikan saat joystick dilepas, nilai command kembali nol/netral.

### Tahap 2 - Uji PWM Tanpa Thruster Terpasang

Tujuan: memastikan MCU/PWM controller menerima data dari ROS.

Cek dengan oscilloscope/servo tester jika ada. Pastikan:

- netral `1500 us`,
- channel sesuai thruster 1 sampai 6,
- tidak ada lonjakan saat node start/stop,
- saat ROS mati, output kembali netral.

### Tahap 3 - Uji Thruster Satu Per Satu

Tujuan: memastikan arah dan mapping benar.

Uji:

```text
thruster 1
thruster 2
thruster 3
thruster 4
thruster 5
thruster 6
```

Catat arah putar dan gaya dorong. Sesuaikan mapping sebelum uji gerak penuh.

### Tahap 4 - Uji Gripper

Tujuan:

- buka/tutup sesuai command,
- tidak stall,
- tidak menjepit terlalu kuat,
- payload bisa masuk,
- payload bisa dilepas.

### Tahap 5 - Uji Kamera Dan QR

Tujuan:

- kamera depan tampil di GUI,
- kamera bawah tampil di GUI,
- QR A/B/C/D terbaca di `/rov/qr_code`,
- QR tetap terbaca di air dengan lampu aktif.

### Tahap 6 - Uji Kolam Dangkal

Tujuan:

- ROV tidak bocor,
- buoyancy mendekati netral,
- thruster tidak membuat ROV roll/pitch liar,
- joystick bisa mengendalikan maju, geser, naik/turun, yaw.

Gunakan limit PWM kecil dulu.

### Tahap 7 - Uji Misi KKI

Urutan:

1. Turun ke dasar secara manual.
2. Scan QR.
3. Ambil payload dengan gripper.
4. Bergerak ke sisi A/B/C/D sesuai QR.
5. Masukkan lubang payload ke hook.
6. Lepas gripper.
7. ROV naik ke permukaan dan bersandar di sisi payload.
8. Untuk misi autonomous, setelah payload terkait, jalankan mode autonomous surface/release sesuai aturan lomba.

## 14. Parameter Yang Perlu Diubah Saat Pindah Ke Hardware

Parameter awal yang sebaiknya lebih konservatif:

```bash
joystick_linear_scale:=0.30
joystick_vertical_scale:=0.25
joystick_yaw_scale:=0.25
pwm_min_us:=1400.0
pwm_max_us:=1600.0
pwm_response_s:=0.15
```

Setelah aman, naikkan sedikit demi sedikit.

Jangan langsung memakai tuning simulasi sebagai tuning final hardware. Simulasi berguna untuk alur misi dan software, tetapi gaya air, massa, buoyancy, drag, dan kabel nyata harus dikalibrasi di kolam.

## 15. Checklist Sebelum Masuk Air

- Enclosure sudah leak test tanpa elektronik.
- Semua konektor diberi strain relief.
- Fuse utama terpasang.
- Emergency stop tersedia.
- Stik punya deadman button.
- ESC netral saat ROS start.
- ESC kembali netral saat ROS mati.
- Kamera muncul di GUI.
- QR terbaca.
- Gripper buka/tutup tanpa stall.
- Leak sensor terbaca.
- Tegangan baterai terbaca.
- Tether tidak bisa masuk propeller.
- Operator tahu perintah stop.

## 16. Checklist Setelah Uji Air

- Cek ada air/embun di enclosure.
- Cek suhu ESC, regulator, dan motor.
- Cek log tegangan dan arus.
- Cek apakah ada propeller longgar.
- Cek apakah gripper berubah posisi akibat benturan.
- Catat parameter yang dipakai saat uji.
- Catat masalah mapping thruster atau joystick.

## 17. Batas Klaim Yang Aman Untuk Laporan

Kalimat yang aman:

> Simulasi digunakan untuk memvalidasi alur misi, integrasi ROS 2, pembacaan QR, kontrol joystick, pemetaan command ke PWM thruster, tampilan GUI operator, dan urutan manipulasi payload sebelum implementasi ke ROV nyata.

Jangan mengklaim:

> Gaya thruster, drag air, dan performa payload di simulasi sudah sama persis dengan ROV nyata.

Kalimat yang lebih defensible:

> Nilai PWM dan respon gerak pada hardware harus dikalibrasi ulang melalui uji kolam karena massa, buoyancy, drag, dan efek tether pada ROV nyata berbeda dari model simulasi.

## 18. Target Implementasi Berikutnya

Prioritas agar workspace ini benar-benar siap hardware:

1. Uji `serial_pwm_driver` dengan `hardware_dry_run:=true`.
2. Upload firmware contoh ke mikrokontroler.
3. Uji `serial_pwm_driver` dengan `hardware_dry_run:=false` tanpa propeller.
4. Integrasikan kamera nyata ke `/rov/camera/wall/image` dan `/rov/camera/bottom/image`.
5. Tambahkan driver depth sensor.
6. Tambahkan health monitor untuk leak, tegangan, arus, dan suhu.
7. Tambahkan logging uji kolam.
