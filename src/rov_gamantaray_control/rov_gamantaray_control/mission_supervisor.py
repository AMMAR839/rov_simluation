import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String


HOOK_TARGETS = {
    "A": (-4.45, 0.0, -0.45),
    "B": (4.45, 0.0, -0.45),
    "C": (0.0, 4.45, -0.45),
    "D": (0.0, -4.45, -0.45),
}


class MissionSupervisor(Node):
    """Simple autonomous mission baseline for scan, pick, carry, release, surface."""

    def __init__(self) -> None:
        super().__init__("mission_supervisor")
        self.declare_parameter("linear_gain", 0.45)
        self.declare_parameter("position_tolerance_m", 0.35)
        self.declare_parameter("default_payload_code", "A")

        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.tolerance = float(self.get_parameter("position_tolerance_m").value)
        self.payload_code = str(self.get_parameter("default_payload_code").value).upper()
        self.state = "scan_payload"
        self.latest_odom: Optional[Odometry] = None
        self.state_started = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, "/rov/cmd_vel", 10)
        self.gripper_pub = self.create_publisher(Float64, "/rov/gripper_cmd", 10)
        self.state_pub = self.create_publisher(String, "/rov/mission_state", 10)
        self.create_subscription(Odometry, "/model/gamantaray_rov/odometry", self.odom_callback, 10)
        self.create_subscription(String, "/rov/qr_code", self.qr_callback, 10)
        self.create_timer(0.1, self.update)

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def qr_callback(self, msg: String) -> None:
        if msg.data in HOOK_TARGETS:
            self.payload_code = msg.data
            if self.state == "scan_payload":
                self.set_state("pick_payload")

    def set_state(self, state: str) -> None:
        if self.state != state:
            self.state = state
            self.state_started = self.get_clock().now()
            self.get_logger().info(f"Mission state: {state}")

    def update(self) -> None:
        self.state_pub.publish(String(data=f"{self.state} payload={self.payload_code}"))
        if self.latest_odom is None:
            return

        if self.state == "scan_payload":
            self.gripper_pub.publish(Float64(data=0.0))
            self.publish_to_target((-0.35, 0.0, -0.42))
            return

        if self.state == "pick_payload":
            reached = self.publish_to_target((-0.35, 0.0, -0.65))
            if reached:
                self.gripper_pub.publish(Float64(data=1.0))
                if self.elapsed_s() > 1.5:
                    self.set_state("go_to_hook")
            return

        if self.state == "go_to_hook":
            target = HOOK_TARGETS.get(self.payload_code, HOOK_TARGETS["A"])
            reached = self.publish_to_target(target)
            self.gripper_pub.publish(Float64(data=1.0))
            if reached:
                self.set_state("release_payload")
            return

        if self.state == "release_payload":
            self.publish_stop()
            self.gripper_pub.publish(Float64(data=0.0))
            if self.elapsed_s() > 1.0:
                self.set_state("surface")
            return

        if self.state == "surface":
            target = HOOK_TARGETS.get(self.payload_code, HOOK_TARGETS["A"])
            self.publish_to_target((target[0], target[1], -0.08))

    def elapsed_s(self) -> float:
        return (self.get_clock().now() - self.state_started).nanoseconds * 1e-9

    def publish_to_target(self, target: tuple[float, float, float]) -> bool:
        p = self.latest_odom.pose.pose.position
        ex = target[0] - p.x
        ey = target[1] - p.y
        ez = target[2] - p.z
        distance = math.sqrt(ex * ex + ey * ey + ez * ez)

        cmd = Twist()
        cmd.linear.x = max(-1.0, min(1.0, self.linear_gain * ex))
        cmd.linear.y = max(-1.0, min(1.0, self.linear_gain * ey))
        cmd.linear.z = max(-0.7, min(0.7, self.linear_gain * ez))
        self.cmd_pub.publish(cmd)
        return distance < self.tolerance

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.publish_stop()
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
