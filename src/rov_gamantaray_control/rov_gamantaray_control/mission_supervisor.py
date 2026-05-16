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

VALID_PROFILES = {"full", "carry_release_surface", "release_surface"}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class MissionSupervisor(Node):
    """Finite-state autonomous baseline for the KKI ROV mission."""

    def __init__(self) -> None:
        super().__init__("mission_supervisor")
        self.declare_parameter("linear_gain", 0.45)
        self.declare_parameter("position_tolerance_m", 0.25)
        self.declare_parameter("default_payload_code", "A")
        self.declare_parameter("mission_profile", "full")
        self.declare_parameter("scan_target_x", -0.55)
        self.declare_parameter("scan_target_y", 0.0)
        self.declare_parameter("scan_target_z", -0.62)
        self.declare_parameter("pickup_target_x", -0.35)
        self.declare_parameter("pickup_target_y", 0.0)
        self.declare_parameter("pickup_target_z", -0.65)
        self.declare_parameter("surface_z", -0.08)
        self.declare_parameter("scan_timeout_s", 8.0)
        self.declare_parameter("use_default_payload_after_scan_timeout", True)
        self.declare_parameter("pickup_timeout_s", 4.0)
        self.declare_parameter("release_timeout_s", 3.0)
        self.declare_parameter("max_xy_cmd", 0.75)
        self.declare_parameter("max_z_cmd", 0.55)
        self.declare_parameter("cmd_vel_topic", "/rov/auto_cmd_vel")
        self.declare_parameter("require_auto_command_source", True)
        self.declare_parameter("auto_start_on_attached", False)

        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.tolerance = float(self.get_parameter("position_tolerance_m").value)
        self.payload_code = str(self.get_parameter("default_payload_code").value).upper()
        if self.payload_code not in HOOK_TARGETS:
            self.payload_code = "A"

        self.profile = str(self.get_parameter("mission_profile").value).lower()
        if self.profile not in VALID_PROFILES:
            self.get_logger().warning(
                f"Unknown mission_profile '{self.profile}', falling back to 'full'"
            )
            self.profile = "full"

        self.scan_target = (
            float(self.get_parameter("scan_target_x").value),
            float(self.get_parameter("scan_target_y").value),
            float(self.get_parameter("scan_target_z").value),
        )
        self.pickup_target = (
            float(self.get_parameter("pickup_target_x").value),
            float(self.get_parameter("pickup_target_y").value),
            float(self.get_parameter("pickup_target_z").value),
        )
        self.surface_z = float(self.get_parameter("surface_z").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_s").value)
        self.use_default_payload_after_scan_timeout = bool(
            self.get_parameter("use_default_payload_after_scan_timeout").value
        )
        self.pickup_timeout = float(self.get_parameter("pickup_timeout_s").value)
        self.release_timeout = float(self.get_parameter("release_timeout_s").value)
        self.max_xy_cmd = float(self.get_parameter("max_xy_cmd").value)
        self.max_z_cmd = float(self.get_parameter("max_z_cmd").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.require_auto_source = bool(
            self.get_parameter("require_auto_command_source").value
        )
        self.auto_start_on_attached = bool(
            self.get_parameter("auto_start_on_attached").value
        )

        self.state = self.initial_state_for_profile()
        self.latest_odom: Optional[Odometry] = None
        self.detected_payload: Optional[str] = None
        self.gripper_state = "unknown"
        self.gripper_hook = "none"
        self.gripper_aligned = False
        self.active_command_source = "unknown"
        self.waiting_for_auto_source = False
        self.state_started = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.gripper_pub = self.create_publisher(Float64, "/rov/gripper_cmd", 10)
        self.state_pub = self.create_publisher(String, "/rov/mission_state", 10)
        self.command_source_pub = self.create_publisher(String, "/rov/command_source", 10)
        self.create_subscription(Odometry, "/model/gamantaray_rov/odometry", self.odom_callback, 10)
        self.create_subscription(String, "/rov/qr_code", self.qr_callback, 10)
        self.create_subscription(String, "/rov/gripper_status", self.gripper_status_callback, 10)
        self.create_subscription(
            String, "/rov/active_command_source", self.command_source_callback, 10
        )
        self.create_timer(0.1, self.update)
        self.get_logger().info(
            f"Mission profile={self.profile} state={self.state} payload={self.payload_code}"
        )

    def initial_state_for_profile(self) -> str:
        if self.profile == "release_surface":
            return "release_payload"
        if self.profile == "carry_release_surface":
            return "go_to_hook"
        return "scan_payload"

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def command_source_callback(self, msg: String) -> None:
        source = msg.data.strip().lower()
        if source in {"manual", "auto"}:
            self.active_command_source = source

    def qr_callback(self, msg: String) -> None:
        code = msg.data.strip().upper()
        if code in HOOK_TARGETS:
            self.detected_payload = code
            self.payload_code = code
            if self.state == "scan_payload":
                self.set_state("pick_payload")

    def gripper_status_callback(self, msg: String) -> None:
        tokens = msg.data.split()
        if tokens:
            self.gripper_state = tokens[0]
        for token in tokens[1:]:
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            if key == "payload" and value in HOOK_TARGETS:
                self.payload_code = value
            elif key == "hook":
                self.gripper_hook = value
            elif key == "aligned":
                self.gripper_aligned = value.lower() == "true"

    def set_state(self, state: str) -> None:
        if self.state != state:
            self.state = state
            self.state_started = self.get_clock().now()
            self.get_logger().info(f"Mission state: {state}")

    def update(self) -> None:
        self.publish_state()
        if self.require_auto_source and self.active_command_source != "auto":
            if self.should_auto_start_from_attached_payload():
                self.command_source_pub.publish(String(data="auto"))
                self.get_logger().info("Payload attached, switching command source to auto")
            self.waiting_for_auto_source = True
            self.publish_stop()
            return
        if self.waiting_for_auto_source:
            self.waiting_for_auto_source = False
            self.state_started = self.get_clock().now()

        if self.latest_odom is None:
            return

        if self.state == "scan_payload":
            self.handle_scan_payload()
        elif self.state == "pick_payload":
            self.handle_pick_payload()
        elif self.state == "go_to_hook":
            self.handle_go_to_hook()
        elif self.state == "release_payload":
            self.handle_release_payload()
        elif self.state == "surface":
            self.handle_surface()
        elif self.state == "complete":
            self.gripper_pub.publish(Float64(data=0.0))
            self.publish_stop()

    def should_auto_start_from_attached_payload(self) -> bool:
        return (
            self.auto_start_on_attached
            and self.profile == "carry_release_surface"
            and self.gripper_state == "attached"
        )

    def handle_scan_payload(self) -> None:
        self.gripper_pub.publish(Float64(data=0.0))
        self.publish_to_target(self.scan_target)
        if (
            self.use_default_payload_after_scan_timeout
            and self.detected_payload is None
            and self.elapsed_s() > self.scan_timeout
        ):
            self.get_logger().warning(
                f"QR not detected after {self.scan_timeout:.1f}s, using payload {self.payload_code}"
            )
            self.set_state("pick_payload")

    def handle_pick_payload(self) -> None:
        reached = self.publish_to_target(self.pickup_target)
        if reached:
            self.gripper_pub.publish(Float64(data=1.0))
        else:
            self.gripper_pub.publish(Float64(data=0.0))

        if self.gripper_state == "attached":
            self.set_state("go_to_hook")
        elif reached and self.elapsed_s() > self.pickup_timeout:
            self.get_logger().warning("Pickup timeout, continuing to hook with current gripper state")
            self.set_state("go_to_hook")

    def handle_go_to_hook(self) -> None:
        self.gripper_pub.publish(Float64(data=1.0))
        target = HOOK_TARGETS.get(self.payload_code, HOOK_TARGETS["A"])
        reached = self.publish_to_target(target)
        if reached:
            self.set_state("release_payload")

    def handle_release_payload(self) -> None:
        self.publish_stop()
        self.gripper_pub.publish(Float64(data=0.0))

        payload_hung = (
            self.gripper_state == "hung"
            and self.gripper_hook in {self.payload_code, "none"}
        )
        if payload_hung:
            self.set_state("surface")
        elif self.elapsed_s() > self.release_timeout:
            self.get_logger().warning("Release timeout, surfacing anyway")
            self.set_state("surface")

    def handle_surface(self) -> None:
        self.gripper_pub.publish(Float64(data=0.0))
        target = HOOK_TARGETS.get(self.payload_code, HOOK_TARGETS["A"])
        reached = self.publish_to_target((target[0], target[1], self.surface_z))
        if reached:
            self.set_state("complete")

    def elapsed_s(self) -> float:
        return (self.get_clock().now() - self.state_started).nanoseconds * 1e-9

    def publish_to_target(self, target: tuple[float, float, float]) -> bool:
        p = self.latest_odom.pose.pose.position
        ex = target[0] - p.x
        ey = target[1] - p.y
        ez = target[2] - p.z
        distance = math.sqrt(ex * ex + ey * ey + ez * ez)

        cmd = Twist()
        cmd.linear.x = clamp(self.linear_gain * ex, -self.max_xy_cmd, self.max_xy_cmd)
        cmd.linear.y = clamp(self.linear_gain * ey, -self.max_xy_cmd, self.max_xy_cmd)
        cmd.linear.z = clamp(self.linear_gain * ez, -self.max_z_cmd, self.max_z_cmd)
        self.cmd_pub.publish(cmd)
        return distance < self.tolerance

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_state(self) -> None:
        self.state_pub.publish(
            String(
                data=(
                    f"{self.state} profile={self.profile} payload={self.payload_code} "
                    f"qr={self.detected_payload or 'none'} gripper={self.gripper_state} "
                    f"hook={self.gripper_hook} aligned={str(self.gripper_aligned).lower()} "
                    f"source={self.active_command_source}"
                )
            )
        )


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
