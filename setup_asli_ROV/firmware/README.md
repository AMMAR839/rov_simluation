# Firmware Mikrokontroler

Firmware menerima serial dari ROS:

```text
PWM,t1,t2,t3,t4,t5,t6,gripper
```

Lalu mengeluarkan sinyal PWM servo-style ke ESC:

```text
1500 us = netral
1100 us = full reverse
1900 us = full forward
```

File:

- `arduino_serial_pwm_bridge.ino`: contoh siap upload untuk Arduino.
- `stm32_serial_pwm_bridge_hal_example.c`: contoh logika untuk STM32 HAL/CubeIDE.

Firmware wajib punya watchdog. Jika serial timeout, semua thruster harus kembali `1500 us`.

