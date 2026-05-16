import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CmdVelMux(Node):
    """Select manual or autonomous velocity command for the ROV."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")
        self.declare_parameter("default_source", "manual")
        self.declare_parameter("command_timeout_s", 0.5)

        self.source = str(self.get_parameter("default_source").value).lower()
        if self.source not in {"manual", "auto"}:
            self.source = "manual"
        self.command_timeout = float(self.get_parameter("command_timeout_s").value)

        self.manual_cmd = Twist()
        self.auto_cmd = Twist()
        self.manual_time = 0.0
        self.auto_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/rov/cmd_vel", 10)
        self.source_pub = self.create_publisher(String, "/rov/active_command_source", 10)
        self.create_subscription(Twist, "/rov/manual_cmd_vel", self.manual_callback, 10)
        self.create_subscription(Twist, "/rov/auto_cmd_vel", self.auto_callback, 10)
        self.create_subscription(String, "/rov/command_source", self.source_callback, 10)
        self.create_timer(0.05, self.update)
        self.get_logger().info(f"Command source: {self.source}")

    def manual_callback(self, msg: Twist) -> None:
        self.manual_cmd = msg
        self.manual_time = time.monotonic()

    def auto_callback(self, msg: Twist) -> None:
        self.auto_cmd = msg
        self.auto_time = time.monotonic()

    def source_callback(self, msg: String) -> None:
        source = msg.data.strip().lower()
        if source not in {"manual", "auto"}:
            self.get_logger().warning("Command source must be 'manual' or 'auto'")
            return
        if source != self.source:
            self.source = source
            self.get_logger().info(f"Command source: {self.source}")

    def update(self) -> None:
        now = time.monotonic()
        if self.source == "auto":
            cmd = self.auto_cmd if now - self.auto_time <= self.command_timeout else Twist()
        else:
            cmd = self.manual_cmd if now - self.manual_time <= self.command_timeout else Twist()
        self.cmd_pub.publish(cmd)
        self.source_pub.publish(String(data=self.source))


def main() -> None:
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
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
