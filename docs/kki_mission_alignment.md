# Kesesuaian Workspace Dengan Sosialisasi KKI 2026 ROV

Sumber pembanding: `/home/ammar/Downloads/Sosialisasi KKI 2026 ROV.pdf`.

## Sudah Sesuai Untuk Simulasi Misi

- Kolam lomba 10 m x 10 m dengan kedalaman representatif 0.85 m dari rentang PDF 0.7-0.9 m.
- ROV default `rov_variant:=github_blue` memakai visual BlueROV2-style dari GitHub yang diskalakan agar body + thruster berada dalam batas 35 x 35 x 35 cm. Gripper menempel ke ROV dan boleh berada di luar dimensi sesuai PDF.
- Payload QR A/B/C/D tersedia di dasar kolam dengan geometri mengikuti gambar PDF: plate 5 cm x 10 cm, QR 4 cm x 4 cm menghadap samping, base 3 cm, dan lubang gantung di atas QR. Bukaan simulasi dibuat sekitar 3.4 cm agar peg hook diameter 2 cm punya clearance dan benar-benar bisa masuk.
- Hook/gantungan A/B/C/D tersedia di sisi kolam sebagai model PVC dinding: backing, clamp bibir kolam, pipa vertikal, elbow, dan peg horizontal terbuka diameter 2 cm. Bagian utama punya collision SDF.
- Pelepasan payload di dekat hook memakai constraint gantung kinematic hanya setelah lubang payload benar-benar sejajar dengan peg dalam toleransi ketat. Jika tidak sejajar, payload dilepas/drop dan tidak dihitung tergantung.
- Misi autonomous nomor 5 tersedia lewat `mission_profile:=release_surface`: operator manual membawa payload ke area hook, lalu autonomous melakukan pelepasan payload dan naik ke permukaan. `carry_release_surface` hanya mode eksperimen tambahan, bukan alur utama misi nomor 5.
- Alur misi tersedia: scan QR, ambil payload, pindah ke sisi sesuai QR, lepas payload, lalu naik ke permukaan. Untuk misi nomor 5, bagian yang dijalankan autonomous adalah pelepasan payload dan naik ke permukaan setelah operator membawa ROV ke area hook.
- Kontrol manual tersedia lewat joystick dan keyboard.
- Mode autonomous baseline tersedia lewat `mission_autonomy:=true`.
- GUI lomba tersedia lewat `kki_gui:=true`: dua kamera, status QR, altitude ROV dari dasar kolam, waktu, identitas Gamantara/Universitas Gadjah Mada, desain ROV, trajectory map, status gripper, mission, dan tether.
- Tether tersedia sebagai kabel visual dinamis dari anchor permukaan ke ROV dengan status panjang/tension di `/rov/tether_status`.

## Belum Lengkap Untuk Paket Lomba Penuh

- Fitur advanced seperti screenshot otomatis, logging, replay camera/trajectory, alarm kedalaman, dan toggle manual/autonomous belum dibuat sebagai GUI.
- Posisi A/B/C/D di PDF disebut dapat diacak. Workspace ini masih memakai layout default A kiri, B kanan, C atas, D bawah.
- Tether yang dibuat adalah visual/tension model, belum full flexible-body tether dengan collision dan solver gaya kabel nyata.

## Kesimpulan Teknis

Workspace ini sudah cocok untuk pengembangan kontrol, persepsi QR, gripper, GUI operator minimal, tether visual, dan alur misi ROV KKI. Untuk disebut sebagai paket lomba penuh, tahap berikutnya adalah logging/replay/alarm, toggle manual-autonomous di GUI, randomisasi posisi A/B/C/D, dan tether fleksibel berbasis solver.
