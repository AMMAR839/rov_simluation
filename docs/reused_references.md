# Bagian Referensi yang Dipakai

Sumber tugas utama adalah `/home/ammar/Downloads/Sosialisasi KKI 2026 ROV.pdf`.

Hal yang diambil dari PDF:

- kolam 10 m x 10 m, kedalaman representatif 0.85 m dari rentang 0.7 sampai 0.9 m,
- layout sisi kolam A/B/C/D dengan hook/gantungan,
- payload dengan QR Code A/B/C/D,
- kebutuhan ROV lengkap dengan gripper dan dua kamera,
- alur misi scan QR, ambil payload, pindahkan ke hook sesuai QR, lalu naik ke permukaan.

Hal yang diambil dari `/home/ammar/Documents/rov_gamantaray_1`:

- pola simulasi Gazebo Harmonic untuk kendaraan bawah air,
- konsep model Beaumont yang sudah punya gripper/claw, multi-kamera, thruster, buoyancy, dan hydrodynamics,
- pemisahan topik kamera bawah/depan dan perintah claw/gripper.
- konsep gripper/claw dipakai untuk mekanisme attach/release payload. Mesh Beaumont tidak lagi dijadikan opsi launch karena visualnya tidak rapi saat dipakai di arena KKI.

Hal yang diambil dari `/home/ammar/Documents/rov_gamantaray_2`:

- struktur ROS 2 package yang lebih rapi,
- konfigurasi 6 thruster seperti BlueROV2,
- penggunaan plugin bawaan Gazebo untuk thruster dan hydrodynamics,
- pola bridge ROS-Gazebo untuk command thruster, kamera, dan odometry.
- mesh `bluerov2_noprop.dae` tetap tersedia di `rov_variant:=bluerov` sebagai pembanding lama.
- default baru memakai `rov_variant:=github_blue`, yaitu mesh BlueROV2 dan T200 propeller dari GitHub `evan-palmer/blue` ditambah gripper bawah-depan custom yang dibuat sebagai geometri SDF agar menyatu dengan body ROV. Asset Ricketts tetap disimpan sebagai referensi lokal, tetapi visual aktif claw sudah dibuat ulang.
- posisi dan orientasi propeller default mengikuti `blue_description/description/bluerov2/urdf.xacro` dari repo GitHub tersebut, lalu diskalakan agar body + thruster berada di batas 35 x 35 x 35 cm sesuai PDF.
- visual air sekarang mengambil konsep wavefield/wake dari `osrf/vrx`, tetapi tidak memasukkan full dependency VRX karena targetnya USV laut terbuka.
- fisika air mengacu ke dokumentasi `gazebosim/gz-sim` untuk buoyancy, hydrodynamics, dan thruster. Plugin `rock-gazebo/simulation-gazebo_underwater` tidak dipakai langsung karena dibuat untuk Gazebo Classic lama.

Keputusan implementasi baru:

- plugin kustom lama dari `rov_gamantaray_1` tidak dijadikan dependency wajib karena workspace baru harus langsung cocok dengan ROS 2 Jazzy + Gazebo Harmonic di device ini,
- model dibuat SDF native supaya Gazebo bisa memuat sensor, visual ROV, payload, dan kolam tanpa proses konversi URDF yang rapuh,
- pergerakan default dibuat kinematic dari output allocator thruster karena kombinasi rigid-body kecil + DART/ODE pada device ini membuat model dinamik bawah air crash saat smoke test,
- gripper memakai node ROS `gripper_manager` untuk animasi rahang, pengecekan alignment payload terhadap area jepit, dan attach/release payload melalui service `/world/kki_rov_pool/set_pose`.
- mode hidrodinamika tetap disimpan sebagai opsi eksperimen, tetapi launch default memakai mode kinematic yang lebih ringan dan stabil untuk latihan misi.
- world air dari kedua folder referensi tidak disalin langsung karena keduanya hanya memakai `water_plane` biru sederhana; fisika airnya berada di plugin buoyancy/hydrodynamics, bukan di model visual air.
