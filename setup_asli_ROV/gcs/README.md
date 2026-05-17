# Ground Station Code

Folder ini untuk laptop operator.

Yang dijalankan di ground station:

- `rov_joystick`: membaca stik Xbox/gamepad.
- `kki_dashboard`: GUI operator.
- Opsional `qr_detector` jika pemrosesan QR ingin dilakukan di laptop operator.

Ground station mengirim command ke onboard lewat ROS 2 DDS di jaringan tether Ethernet.

## Jalankan

```bash
cd /home/ammar/Documents/WS_ROV
bash setup_asli_ROV/gcs/run_gcs.sh
```

Default:

- joystick device: `/dev/input/js0`
- deadman button: `4`
- linear scale: `0.30`
- vertical scale: `0.25`
- yaw scale: `0.25`

Ubah device:

```bash
JOYSTICK_DEVICE=/dev/input/by-id/usb-SHANWAN_Android_Gamepad-joystick \
bash setup_asli_ROV/gcs/run_gcs.sh
```

