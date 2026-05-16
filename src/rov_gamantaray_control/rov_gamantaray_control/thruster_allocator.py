import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ThrusterAllocator(Node):
    """Map normalized cmd_vel input into six Gazebo thruster commands."""

    def __init__(self) -> None:
        super().__init__("thruster_allocator")
        self.declare_parameter("max_horizontal_thrust_n", 22.0)
        self.declare_parameter("max_vertical_thrust_n", 18.0)
        self.declare_parameter("yaw_scale", 0.75)
        self.declare_parameter("command_timeout_s", 0.5)

        self.max_horizontal = float(self.get_parameter("max_horizontal_thrust_n").value)
        self.max_vertical = float(self.get_parameter("max_vertical_thrust_n").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)

        self.thruster_pubs = [
            self.create_publisher(Float64, f"/rov/thruster{i}/cmd", 10)
            for i in range(1, 7)
        ]
        self.status_pub = self.create_publisher(Float64MultiArray, "/rov/thruster_status", 10)
        self.create_subscription(Twist, "/rov/cmd_vel", self.cmd_vel_callback, 10)
        self.create_timer(0.05, self.watchdog)

        self.last_command_time = 0.0
        self.last_output = [0.0] * 6
        self.zero_sent = True

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.last_command_time = time.monotonic()
        self.zero_sent = False

        surge = clamp(msg.linear.x, -1.0, 1.0) * self.max_horizontal
        sway = clamp(msg.linear.y, -1.0, 1.0) * self.max_horizontal
        heave = clamp(msg.linear.z, -1.0, 1.0) * self.max_vertical
        yaw = clamp(msg.angular.z, -1.0, 1.0) * self.max_horizontal * self.yaw_scale

        commands = [
            surge - sway - yaw,
            surge + sway + yaw,
            surge + sway - yaw,
            surge - sway + yaw,
            heave,
            heave,
        ]

        max_abs = max(abs(v) for v in commands[:4])
        if max_abs > self.max_horizontal:
            scale = self.max_horizontal / max_abs
            commands[:4] = [v * scale for v in commands[:4]]

        commands[4] = clamp(commands[4], -self.max_vertical, self.max_vertical)
        commands[5] = clamp(commands[5], -self.max_vertical, self.max_vertical)
        self.publish(commands)

    def watchdog(self) -> None:
        if self.zero_sent:
            return
        if time.monotonic() - self.last_command_time > self.command_timeout_s:
            self.publish([0.0] * 6)
            self.zero_sent = True

    def publish(self, commands: list[float]) -> None:
        self.last_output = commands
        for pub, value in zip(self.thruster_pubs, commands):
            pub.publish(Float64(data=float(value)))
        self.status_pub.publish(Float64MultiArray(data=[float(v) for v in commands]))


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
                node.publish([0.0] * 6)
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
