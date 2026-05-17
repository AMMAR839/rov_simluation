import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from rov_gamantaray_control.pwm_model import clamp, normalized_to_pwm, pwm_to_thrust


class ThrusterAllocator(Node):
    """Map cmd_vel into ESC-style PWM and a thrust estimate for six thrusters."""

    def __init__(self) -> None:
        super().__init__("thruster_allocator")
        self.declare_parameter("max_horizontal_thrust_n", 22.0)
        self.declare_parameter("max_vertical_thrust_n", 18.0)
        self.declare_parameter("yaw_scale", 0.75)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("pwm_neutral_us", 1500.0)
        self.declare_parameter("pwm_min_us", 1100.0)
        self.declare_parameter("pwm_max_us", 1900.0)
        self.declare_parameter("pwm_deadband_us", 25.0)
        self.declare_parameter("pwm_response_s", 0.04)

        self.max_horizontal = float(self.get_parameter("max_horizontal_thrust_n").value)
        self.max_vertical = float(self.get_parameter("max_vertical_thrust_n").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.pwm_neutral = float(self.get_parameter("pwm_neutral_us").value)
        self.pwm_min = float(self.get_parameter("pwm_min_us").value)
        self.pwm_max = float(self.get_parameter("pwm_max_us").value)
        self.pwm_deadband = float(self.get_parameter("pwm_deadband_us").value)
        self.pwm_response_s = float(self.get_parameter("pwm_response_s").value)

        self.thrust_pubs = [
            self.create_publisher(Float64, f"/rov/thruster{i}/cmd", 10)
            for i in range(1, 7)
        ]
        self.pwm_pubs = [
            self.create_publisher(Float64, f"/rov/thruster{i}/pwm", 10)
            for i in range(1, 7)
        ]
        self.pwm_status_pub = self.create_publisher(Float64MultiArray, "/rov/thruster_pwm", 10)
        self.status_pub = self.create_publisher(Float64MultiArray, "/rov/thruster_status", 10)
        self.create_subscription(Twist, "/rov/cmd_vel", self.cmd_vel_callback, 10)
        self.create_timer(0.05, self.watchdog)

        self.last_command_time = 0.0
        self.last_output = [0.0] * 6
        self.last_pwm = [self.pwm_neutral] * 6
        self.last_publish_time = time.monotonic()
        self.zero_sent = True

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.last_command_time = time.monotonic()
        self.zero_sent = False

        surge = clamp(msg.linear.x, -1.0, 1.0)
        sway = clamp(msg.linear.y, -1.0, 1.0)
        heave = clamp(msg.linear.z, -1.0, 1.0)
        yaw = clamp(msg.angular.z, -1.0, 1.0) * self.yaw_scale

        normalized = [
            surge - sway - yaw,
            surge + sway + yaw,
            surge + sway - yaw,
            surge - sway + yaw,
            heave,
            heave,
        ]

        max_abs = max(abs(v) for v in normalized[:4])
        if max_abs > 1.0:
            normalized[:4] = [v / max_abs for v in normalized[:4]]

        normalized[4] = clamp(normalized[4], -1.0, 1.0)
        normalized[5] = clamp(normalized[5], -1.0, 1.0)
        self.publish_normalized(normalized)

    def watchdog(self) -> None:
        if self.zero_sent:
            return
        if time.monotonic() - self.last_command_time > self.command_timeout_s:
            self.publish_normalized([0.0] * 6)
            self.zero_sent = True

    def publish_normalized(self, normalized: list[float]) -> None:
        target_pwm = [
            normalized_to_pwm(value, self.pwm_neutral, self.pwm_min, self.pwm_max)
            for value in normalized
        ]
        pwm = self.apply_pwm_response(target_pwm)
        thrust = [
            pwm_to_thrust(
                value,
                self.max_horizontal if index < 4 else self.max_vertical,
                self.pwm_neutral,
                self.pwm_min,
                self.pwm_max,
                self.pwm_deadband,
            )
            for index, value in enumerate(pwm)
        ]

        self.last_pwm = pwm
        self.last_output = thrust
        for pub, value in zip(self.pwm_pubs, pwm):
            pub.publish(Float64(data=float(value)))
        for pub, value in zip(self.thrust_pubs, thrust):
            pub.publish(Float64(data=float(value)))
        self.pwm_status_pub.publish(Float64MultiArray(data=[float(v) for v in pwm]))
        self.status_pub.publish(Float64MultiArray(data=[float(v) for v in thrust]))

    def apply_pwm_response(self, target_pwm: list[float]) -> list[float]:
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_publish_time))
        self.last_publish_time = now
        if self.pwm_response_s <= 1e-6:
            return target_pwm
        alpha = 1.0 - math.exp(-dt / self.pwm_response_s)
        return [
            current + (target - current) * alpha
            for current, target in zip(self.last_pwm, target_pwm)
        ]

    def publish(self, commands: list[float]) -> None:
        normalized = []
        for index, value in enumerate(commands):
            limit = self.max_horizontal if index < 4 else self.max_vertical
            normalized.append(clamp(value / max(1e-6, limit), -1.0, 1.0))
        self.publish_normalized(normalized)


def main() -> None:
    rclpy.init()
    node = ThrusterAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if "Unable to convert call argument" not in str(exc):
            raise
    finally:
        if rclpy.ok():
            try:
                node.publish_normalized([0.0] * 6)
            except Exception:
                pass
        try:
            node.destroy_node()
        except (KeyboardInterrupt, Exception):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
