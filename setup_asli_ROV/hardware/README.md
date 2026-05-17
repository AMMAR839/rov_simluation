# Hardware ROS Node

`serial_pwm_driver.py` adalah salinan kode node hardware yang membaca:

```text
/rov/thruster_pwm
/rov/gripper_cmd
```

Lalu mengirim serial:

```text
PWM,t1,t2,t3,t4,t5,t6,gripper
```

Versi yang dipakai oleh `ros2 run` berasal dari package:

```text
src/rov_gamantaray_hardware/rov_gamantaray_hardware/serial_pwm_driver.py
```

File di folder ini disediakan agar kode hardware mudah ditemukan bersama setup ROV asli.

