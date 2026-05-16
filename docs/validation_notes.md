# Catatan Pertanggungjawaban Simulasi

Dokumen ini memisahkan klaim yang aman dari klaim yang harus menunggu data uji asli.

## Thruster

Di mode default `physics_mode:=kinematic`, nilai thruster tetap dihitung oleh `thruster_allocator`, tetapi gaya fisika tidak langsung berasal dari propeller Gazebo. Nilai thruster dipakai oleh `kinematic_driver` untuk menggerakkan pose ROV secara stabil.

Propeller yang terlihat berputar adalah visualisasi command thruster. Visual ini berguna untuk debugging dan presentasi, tetapi tidak boleh dipakai sebagai bukti bahwa gaya motor asli sudah sama.

Klaim aman:

- mapping 6 thruster sudah merepresentasikan surge, sway, heave, dan yaw,
- command joystick masuk ke `/rov/cmd_vel`,
- allocator mengubah command menjadi enam output thruster,
- propeller visual berputar saat output thruster tidak nol.

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
