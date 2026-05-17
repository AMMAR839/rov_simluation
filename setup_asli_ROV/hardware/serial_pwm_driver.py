import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class SerialPwmDriver(Node):
    """Send ROV thruster PWM commands to a real microcontroller over serial."""

    def __init__(self) -> None:
        super().__init__("serial_pwm_driver")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("thruster_pwm_topic", "/rov/thruster_pwm")
        self.declare_parameter("gripper_cmd_topic", "/rov/gripper_cmd")
        self.declare_parameter("pwm_neutral_us", 1500.0)
        self.declare_parameter("pwm_min_us", 1100.0)
        self.declare_parameter("pwm_max_us", 1900.0)
        self.declare_parameter("startup_neutral_s", 2.0)
        self.declare_parameter("include_gripper", True)
        self.declare_parameter("gripper_open_pwm_us", 1100.0)
        self.declare_parameter("gripper_closed_pwm_us", 1900.0)
        self.declare_parameter("status_topic", "/rov/hardware_pwm_status")

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.send_rate_hz = max(1.0, float(self.get_parameter("send_rate_hz").value))
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.pwm_neutral = float(self.get_parameter("pwm_neutral_us").value)
        self.pwm_min = float(self.get_parameter("pwm_min_us").value)
        self.pwm_max = float(self.get_parameter("pwm_max_us").value)
        self.startup_neutral_s = max(0.0, float(self.get_parameter("startup_neutral_s").value))
        self.include_gripper = bool(self.get_parameter("include_gripper").value)
        self.gripper_open_pwm = float(self.get_parameter("gripper_open_pwm_us").value)
        self.gripper_closed_pwm = float(self.get_parameter("gripper_closed_pwm_us").value)

        self.serial_handle = None
        self.last_pwm = [self.pwm_neutral] * 6
        self.gripper_pwm = self.gripper_open_pwm
        self.last_command_time = 0.0
        self.last_log_time = 0.0
        self.start_time = time.monotonic()

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("thruster_pwm_topic").value),
            self.thruster_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("gripper_cmd_topic").value),
            self.gripper_callback,
            10,
        )
        self.create_timer(1.0 / self.send_rate_hz, self.send_update)

        mode = "DRY RUN" if self.dry_run else "REAL SERIAL"
        self.get_logger().warning(
            f"Serial PWM driver started in {mode} mode. "
            f"port={self.serial_port} baud={self.baud_rate}"
        )

    def thruster_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            self.get_logger().warning("Ignoring /rov/thruster_pwm with fewer than 6 values")
            return
        self.last_pwm = [
            clamp(float(value), self.pwm_min, self.pwm_max)
            for value in list(msg.data[:6])
        ]
        self.last_command_time = time.monotonic()

    def gripper_callback(self, msg: Float64) -> None:
        position = clamp(float(msg.data), 0.0, 1.0)
        self.gripper_pwm = self.gripper_open_pwm + position * (
            self.gripper_closed_pwm - self.gripper_open_pwm
        )

    def send_update(self) -> None:
        now = time.monotonic()
        pwm = list(self.last_pwm)
        state = "active"

        if now - self.start_time < self.startup_neutral_s:
            pwm = [self.pwm_neutral] * 6
            state = "startup_neutral"
        elif now - self.last_command_time > self.command_timeout_s:
            pwm = [self.pwm_neutral] * 6
            state = "watchdog_neutral"

        line = self.make_line(pwm)
        if self.dry_run:
            if now - self.last_log_time > 1.0:
                self.get_logger().info(f"dry_run serial line: {line.strip()}")
                self.last_log_time = now
        else:
            self.write_serial(line)

        self.status_pub.publish(
            String(
                data=(
                    f"state={state} dry_run={str(self.dry_run).lower()} "
                    f"port={self.serial_port} pwm={','.join(str(int(v)) for v in pwm)} "
                    f"gripper={int(self.gripper_pwm)}"
                )
            )
        )

    def make_line(self, pwm: list[float]) -> str:
        values = [int(round(clamp(value, self.pwm_min, self.pwm_max))) for value in pwm]
        if self.include_gripper:
            values.append(int(round(self.gripper_pwm)))
        return "PWM," + ",".join(str(value) for value in values) + "\n"

    def write_serial(self, line: str) -> None:
        handle = self.open_serial()
        if handle is None:
            return
        try:
            handle.write(line.encode("ascii"))
            handle.flush()
        except Exception as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
            try:
                handle.close()
            except Exception:
                pass
            self.serial_handle = None

    def open_serial(self):
        if self.serial_handle is not None:
            return self.serial_handle
        try:
            import serial
        except ImportError:
            self.get_logger().error(
                "python3-serial is not installed. Install with: sudo apt install python3-serial"
            )
            return None
        try:
            self.serial_handle = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.02,
                write_timeout=0.05,
            )
            self.get_logger().info(f"Opened serial port {self.serial_port}")
        except Exception as exc:
            self.get_logger().error(f"Cannot open serial port {self.serial_port}: {exc}")
            self.serial_handle = None
        return self.serial_handle

    def send_neutral_once(self) -> None:
        line = self.make_line([self.pwm_neutral] * 6)
        if self.dry_run:
            return
        self.write_serial(line)


def main() -> None:
    rclpy.init()
    node = SerialPwmDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send_neutral_once()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
