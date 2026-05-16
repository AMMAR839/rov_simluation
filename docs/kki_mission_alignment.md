# Kesesuaian Workspace Dengan Sosialisasi KKI 2026 ROV

Sumber pembanding: `/home/ammar/Downloads/Sosialisasi KKI 2026 ROV.pdf`.

## Sudah Sesuai Untuk Simulasi Misi

- Kolam lomba 10 m x 10 m dengan kedalaman representatif 0.85 m dari rentang PDF 0.7-0.9 m.
- ROV default `rov_variant:=github_blue` memakai visual BlueROV2-style dari GitHub yang diskalakan agar body + thruster berada dalam batas 35 x 35 x 35 cm. Gripper menempel ke ROV dan boleh berada di luar dimensi sesuai PDF.
- Payload QR A/B/C/D tersedia di dasar kolam.
- Hook/gantungan A/B/C/D tersedia di sisi kolam.
- Alur misi tersedia: scan QR, ambil payload, pindah ke sisi sesuai QR, lepas payload, lalu naik ke permukaan.
- Kontrol manual tersedia lewat joystick dan keyboard.
- Mode autonomous baseline tersedia lewat `mission_autonomy:=true`.

## Belum Lengkap Untuk Paket Lomba Penuh

- GUI lomba belum dibuat penuh. PDF meminta display dua kamera, hasil QR, ketinggian ROV, waktu, identitas tim, desain ROV, dan trajectory.
- Fitur advanced seperti screenshot otomatis, logging, replay camera/trajectory, alarm kedalaman, dan toggle manual/autonomous belum dibuat sebagai GUI.
- Posisi A/B/C/D di PDF disebut dapat diacak. Workspace ini masih memakai layout default A kiri, B kanan, C atas, D bawah.
- Tether baru ada sebagai kebutuhan desain di PDF, belum dimodelkan sebagai kabel fisik.

## Kesimpulan Teknis

Workspace ini sudah cocok untuk pengembangan kontrol, persepsi QR, gripper, dan alur misi ROV KKI. Untuk disebut sebagai simulasi lomba lengkap, tahap berikutnya adalah membuat GUI GCS dan opsi randomisasi posisi A/B/C/D.
