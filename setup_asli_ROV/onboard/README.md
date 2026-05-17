# Onboard ROV Code

Folder ini untuk komputer yang berada di ROV.

Yang dijalankan onboard:

- `cmd_vel_mux`
- `thruster_allocator`
- `serial_pwm_driver`
- opsional `qr_detector`
- driver kamera/sensor nyata jika sudah dibuat

Yang tidak dijalankan onboard:

- `rov_joystick` jika stik berada di ground station
- `kki_dashboard` jika GUI berada di ground station
- Gazebo

## Dry-run

```bash
cd /home/ammar/Documents/WS_ROV
bash setup_asli_ROV/onboard/run_onboard_dryrun.sh
```

## Real serial

```bash
cd /home/ammar/Documents/WS_ROV
bash setup_asli_ROV/onboard/run_onboard_real.sh /dev/ttyACM0
```

