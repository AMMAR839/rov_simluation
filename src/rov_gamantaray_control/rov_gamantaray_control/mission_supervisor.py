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

HOOK_YAWS = {
    "A": math.pi,
    "B": 0.0,
    "C": math.pi * 0.5,
    "D": -math.pi * 0.5,
}

VALID_PROFILES = {"full", "carry_release_surface", "release_surface"}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def parameter_as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


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
        self.declare_parameter("use_default_payload_after_scan_timeout", False)
        self.declare_parameter("require_qr_for_target", True)
        self.declare_parameter("release_requires_attached", True)
        self.declare_parameter("release_surface_on_timeout", False)
        self.declare_parameter("pickup_timeout_s", 4.0)
        self.declare_parameter("release_timeout_s", 3.0)
        self.declare_parameter("max_xy_cmd", 0.75)
        self.declare_parameter("max_z_cmd", 0.55)
        self.declare_parameter("max_yaw_cmd", 0.55)
        self.declare_parameter("yaw_gain", 0.95)
        self.declare_parameter("yaw_tolerance_rad", 0.16)
        self.declare_parameter("transit_depth_z", -0.45)
        self.declare_parameter("standoff_offset_m", 0.60)
        self.declare_parameter("approach_speed_scale", 0.38)
        self.declare_parameter("stage_tolerance_m", 0.18)
        self.declare_parameter("cmd_vel_topic", "/rov/auto_cmd_vel")
        self.declare_parameter("require_auto_command_source", True)
        self.declare_parameter("auto_start_on_attached", False)

        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.tolerance = float(self.get_parameter("position_tolerance_m").value)
        self.fallback_payload_code = str(self.get_parameter("default_payload_code").value).upper()
        if self.fallback_payload_code not in HOOK_TARGETS:
            self.fallback_payload_code = "A"
        self.payload_code: Optional[str] = None

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
        self.use_default_payload_after_scan_timeout = parameter_as_bool(
            self.get_parameter("use_default_payload_after_scan_timeout").value
        )
        self.require_qr_for_target = parameter_as_bool(
            self.get_parameter("require_qr_for_target").value
        )
        self.release_requires_attached = parameter_as_bool(
            self.get_parameter("release_requires_attached").value
        )
        self.release_surface_on_timeout = parameter_as_bool(
            self.get_parameter("release_surface_on_timeout").value
        )
        self.pickup_timeout = float(self.get_parameter("pickup_timeout_s").value)
        self.release_timeout = float(self.get_parameter("release_timeout_s").value)
        self.max_xy_cmd = float(self.get_parameter("max_xy_cmd").value)
        self.max_z_cmd = float(self.get_parameter("max_z_cmd").value)
        self.max_yaw_cmd = float(self.get_parameter("max_yaw_cmd").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance_rad").value)
        self.transit_depth = float(self.get_parameter("transit_depth_z").value)
        self.standoff_offset = float(self.get_parameter("standoff_offset_m").value)
        self.approach_speed_scale = float(self.get_parameter("approach_speed_scale").value)
        self.stage_tolerance = float(self.get_parameter("stage_tolerance_m").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.require_auto_source = parameter_as_bool(
            self.get_parameter("require_auto_command_source").value
        )
        self.auto_start_on_attached = parameter_as_bool(
            self.get_parameter("auto_start_on_attached").value
        )

        self.state = self.initial_state_for_profile()
        self.latest_odom: Optional[Odometry] = None
        self.detected_payload: Optional[str] = None
        self.gripper_state = "unknown"
        self.gripper_hook = "none"
        self.gripper_aligned = False
        self.gripper_hook_aligned = False
        self.gripper_release_block = "unknown"
        self.active_command_source = "unknown"
        self.hook_phase = "rise_to_transit"
        self.waiting_for_auto_source = False
        self.release_timeout_warned = False
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
            f"Mission profile={self.profile} state={self.state} payload=unknown"
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
            if (
                key == "payload"
                and value in HOOK_TARGETS
                and not self.require_qr_for_target
                and self.payload_code is None
            ):
                self.payload_code = value
            elif key == "hook":
                self.gripper_hook = value
            elif key == "aligned":
                self.gripper_aligned = value.lower() == "true"
            elif key == "hook_aligned":
                self.gripper_hook_aligned = value.lower() == "true"
            elif key == "release_block":
                self.gripper_release_block = value

    def set_state(self, state: str) -> None:
        if self.state != state:
            self.state = state
            self.state_started = self.get_clock().now()
            if state == "go_to_hook":
                self.hook_phase = "rise_to_transit"
            if state == "release_payload":
                self.release_timeout_warned = False
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
            and self.target_payload_code() is not None
        )

    def handle_scan_payload(self) -> None:
        self.gripper_pub.publish(Float64(data=0.0))
        self.publish_to_pose((*self.scan_target, 0.0), position_tolerance=self.tolerance)
        if (
            self.use_default_payload_after_scan_timeout
            and self.detected_payload is None
            and self.elapsed_s() > self.scan_timeout
        ):
            self.get_logger().warning(
                f"QR not detected after {self.scan_timeout:.1f}s, using fallback payload {self.fallback_payload_code}"
            )
            self.payload_code = self.fallback_payload_code
            self.set_state("pick_payload")

    def handle_pick_payload(self) -> None:
        reached = self.publish_to_pose(
            (*self.pickup_target, 0.0),
            position_tolerance=self.tolerance,
        )
        if reached:
            self.gripper_pub.publish(Float64(data=1.0))
        else:
            self.gripper_pub.publish(Float64(data=0.0))

        if self.gripper_state == "attached" and self.target_payload_code() is not None:
            self.set_state("go_to_hook")
        elif reached and self.elapsed_s() > self.pickup_timeout:
            self.get_logger().warning("Pickup timeout, continuing to hook with current gripper state")
            self.set_state("go_to_hook")

    def handle_go_to_hook(self) -> None:
        self.gripper_pub.publish(Float64(data=1.0))
        hook_code = self.target_payload_code()
        if hook_code is None:
            self.publish_stop()
            return
        hook_yaw = HOOK_YAWS[hook_code]
        p = self.latest_odom.pose.pose.position

        if self.hook_phase == "rise_to_transit":
            reached = self.publish_to_pose(
                (p.x, p.y, self.transit_depth, hook_yaw),
                position_tolerance=self.stage_tolerance,
            )
            if reached:
                self.hook_phase = "go_to_standoff"
                self.get_logger().info(f"Hook phase: {self.hook_phase}")
            return

        if self.hook_phase == "go_to_standoff":
            reached = self.publish_to_pose(
                self.hook_standoff_pose(hook_code),
                position_tolerance=self.stage_tolerance,
            )
            if reached:
                self.hook_phase = "approach_hook"
                self.get_logger().info(f"Hook phase: {self.hook_phase}")
            return

        reached = self.publish_to_pose(
            (*HOOK_TARGETS[hook_code], hook_yaw),
            position_tolerance=self.stage_tolerance,
            xy_speed_scale=self.approach_speed_scale,
        )
        if reached:
            self.set_state("release_payload")

    def handle_release_payload(self) -> None:
        self.publish_stop()
        hook_code = self.target_payload_code()
        if hook_code is None:
            if self.gripper_state == "attached":
                self.gripper_pub.publish(Float64(data=1.0))
            self.publish_stop()
            return

        if (
            self.release_requires_attached
            and self.gripper_state not in {"attached", "hung"}
        ):
            self.gripper_pub.publish(Float64(data=0.0))
            self.publish_stop()
            return

        if self.gripper_state == "attached" and not self.gripper_hook_aligned:
            self.gripper_pub.publish(Float64(data=1.0))
            self.publish_stop()
            return

        self.gripper_pub.publish(Float64(data=0.0))

        payload_hung = (
            self.gripper_state == "hung"
            and self.gripper_hook in {hook_code, "none"}
        )
        if payload_hung:
            self.set_state("surface")
        elif self.elapsed_s() > self.release_timeout:
            if self.release_surface_on_timeout:
                self.get_logger().warning("Release timeout, surfacing without hung confirmation")
                self.set_state("surface")
            elif not self.release_timeout_warned:
                self.release_timeout_warned = True
                self.get_logger().warning(
                    "Release is not confirmed hung; holding position and keeping gripper open"
                )

    def handle_surface(self) -> None:
        self.gripper_pub.publish(Float64(data=0.0))
        hook_code = self.target_payload_code()
        if hook_code is None:
            self.publish_stop()
            return
        target = HOOK_TARGETS[hook_code]
        reached = self.publish_to_pose(
            (target[0], target[1], self.surface_z, HOOK_YAWS[hook_code]),
            position_tolerance=self.tolerance,
        )
        if reached:
            self.set_state("complete")

    def elapsed_s(self) -> float:
        return (self.get_clock().now() - self.state_started).nanoseconds * 1e-9

    def target_payload_code(self) -> Optional[str]:
        if self.payload_code in HOOK_TARGETS:
            return self.payload_code
        if self.require_qr_for_target:
            return None
        return self.fallback_payload_code

    def hook_standoff_pose(self, hook_code: str) -> tuple[float, float, float, float]:
        x, y, _, yaw = (*HOOK_TARGETS[hook_code], HOOK_YAWS[hook_code])
        if hook_code == "A":
            x += self.standoff_offset
        elif hook_code == "B":
            x -= self.standoff_offset
        elif hook_code == "C":
            y -= self.standoff_offset
        elif hook_code == "D":
            y += self.standoff_offset
        return x, y, self.transit_depth, yaw

    def publish_to_pose(
        self,
        target: tuple[float, float, float, float],
        position_tolerance: Optional[float] = None,
        yaw_tolerance: Optional[float] = None,
        xy_speed_scale: float = 1.0,
    ) -> bool:
        target_x, target_y, target_z, target_yaw = target
        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        ex = target_x - p.x
        ey = target_y - p.y
        ez = target_z - p.z
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        body_x = ex * cos_yaw + ey * sin_yaw
        body_y = -ex * sin_yaw + ey * cos_yaw
        yaw_error = normalize_angle(target_yaw - current_yaw)
        distance = math.sqrt(ex * ex + ey * ey + ez * ez)

        yaw_alignment_scale = clamp(1.0 - abs(yaw_error) / 1.6, 0.35, 1.0)
        xy_limit = self.max_xy_cmd * max(0.05, xy_speed_scale) * yaw_alignment_scale
        cmd = Twist()
        cmd.linear.x = clamp(self.linear_gain * body_x, -xy_limit, xy_limit)
        cmd.linear.y = clamp(self.linear_gain * body_y, -xy_limit, xy_limit)
        cmd.linear.z = clamp(self.linear_gain * ez, -self.max_z_cmd, self.max_z_cmd)
        cmd.angular.z = clamp(self.yaw_gain * yaw_error, -self.max_yaw_cmd, self.max_yaw_cmd)
        self.cmd_pub.publish(cmd)
        pos_tol = self.tolerance if position_tolerance is None else position_tolerance
        yaw_tol = self.yaw_tolerance if yaw_tolerance is None else yaw_tolerance
        return distance < pos_tol and abs(yaw_error) < yaw_tol

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_state(self) -> None:
        self.state_pub.publish(
            String(
                data=(
                    f"{self.state} profile={self.profile} payload={self.target_payload_code() or 'unknown'} "
                    f"qr={self.detected_payload or 'none'} gripper={self.gripper_state} "
                    f"hook={self.gripper_hook} phase={self.hook_phase} "
                    f"aligned={str(self.gripper_aligned).lower()} "
                    f"hook_aligned={str(self.gripper_hook_aligned).lower()} "
                    f"release_block={self.gripper_release_block} "
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
