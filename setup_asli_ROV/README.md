# Setup Asli ROV

Folder ini berisi kode dan script yang dipakai saat ROV dijalankan sebagai hardware asli, bukan simulasi Gazebo.

Pembagian komputer:

```text
Ground station / laptop operator
  - stik Xbox/gamepad
  - GUI KKI
  - opsional QR detector
        |
        | tether Ethernet / jaringan ROS 2
        v
Onboard computer di ROV
  - cmd_vel_mux
  - thruster_allocator
  - serial_pwm_driver
  - camera driver
  - sensor driver
        |
        | USB serial
        v
STM32 / Arduino / ESP32
        |
        | PWM
        v
ESC + thruster
```

## Isi Folder

```text
setup_asli_ROV/
  common/
    env_ground_station.sh
    env_onboard.sh
    check_network_topics.sh
  gcs/
    gcs_station.launch.py
    run_gcs.sh
    README.md
  onboard/
    onboard_rov.launch.py
    run_onboard_dryrun.sh
    run_onboard_real.sh
    README.md
  hardware/
    serial_pwm_driver.py
    README.md
  firmware/
    arduino_serial_pwm_bridge.ino
    stm32_serial_pwm_bridge_hal_example.c
    README.md
```

## Jalur Command

```text
stik
  -> /rov/manual_cmd_vel
  -> cmd_vel_mux
  -> /rov/cmd_vel
  -> thruster_allocator
  -> /rov/thruster_pwm
  -> serial_pwm_driver
  -> STM32/Arduino
  -> PWM ESC
  -> thruster
```

## Setup Jaringan

Contoh IP:

```text
Ground station : 192.168.10.1
Onboard ROV    : 192.168.10.2
ROS_DOMAIN_ID  : 42
```

Di kedua komputer:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Cek koneksi:

```bash
ping 192.168.10.2
```

## Build Workspace

Di kedua komputer, workspace harus sudah dibuild:

```bash
cd /home/ammar/Documents/WS_ROV
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Cara Jalan Aman

### 1. Onboard ROV dry-run

Di komputer onboard:

```bash
cd /home/ammar/Documents/WS_ROV
bash setup_asli_ROV/onboard/run_onboard_dryrun.sh
```

Mode ini belum mengirim serial ke STM32/ESC.

### 2. Ground station

Di laptop operator:

```bash
cd /home/ammar/Documents/WS_ROV
bash setup_asli_ROV/gcs/run_gcs.sh
```

Stik publish `/rov/manual_cmd_vel` dari ground station. Topic ini diterima onboard lewat tether Ethernet.

### 3. Cek topic

Di onboard:

```bash
bash setup_asli_ROV/common/check_network_topics.sh
```

Pastikan `/rov/manual_cmd_vel`, `/rov/thruster_pwm`, dan `/rov/hardware_pwm_status` muncul.

### 4. Onboard real serial

Jika STM32/Arduino sudah siap dan uji aman sudah dilakukan:

```bash
cd /home/ammar/Documents/WS_ROV
bash setup_asli_ROV/onboard/run_onboard_real.sh /dev/ttyACM0
```

Jangan menjalankan mode real serial dengan propeller terpasang sebelum arah thruster dan emergency stop benar.

## Safety

- Uji dry-run dulu.
- Uji serial tanpa propeller.
- Pastikan ESC netral di `1500 us`.
- Pastikan watchdog mikrokontroler mengembalikan thruster ke netral jika serial timeout.
- Gunakan deadman button pada stik.
- Gunakan fuse dan emergency stop.

