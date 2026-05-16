import math
import time
from typing import Optional

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotate_vector_by_quaternion(
    vector: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    x, y, z = vector
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def approach(current: float, target: float, dt: float, time_constant: float) -> float:
    if time_constant <= 1e-6:
        return target
    alpha = 1.0 - math.exp(-dt / time_constant)
    return current + (target - current) * alpha


HOOK_APPROACH_POSES = {
    "A": (-4.45, 0.0, -0.45),
    "B": (4.45, 0.0, -0.45),
    "C": (0.0, 4.45, -0.45),
    "D": (0.0, -4.45, -0.45),
}

HOOK_PEG_POSES = {
    "A": (-4.72, 0.0, -0.46, math.pi),
    "B": (4.72, 0.0, -0.46, 0.0),
    "C": (0.0, 4.72, -0.46, math.pi * 0.5),
    "D": (0.0, -4.72, -0.46, -math.pi * 0.5),
}


class GripperManager(Node):
    """Drive the visual gripper and simulate payload attach/release."""

    def __init__(self) -> None:
        super().__init__("gripper_manager")
        self.declare_parameter("world_name", "kki_rov_pool")
        self.declare_parameter("payload_code", "A")
        self.declare_parameter("gripper_response_s", 0.16)
        self.declare_parameter("capture_forward_offset_m", 0.325)
        self.declare_parameter("capture_vertical_offset_m", -0.112)
        self.declare_parameter("capture_forward_tolerance_m", 0.080)
        self.declare_parameter("capture_lateral_tolerance_m", 0.035)
        self.declare_parameter("capture_vertical_tolerance_m", 0.075)
        self.declare_parameter("payload_width_m", 0.050)
        self.declare_parameter("closed_gap_m", 0.050)
        self.declare_parameter("open_gap_m", 0.150)
        self.declare_parameter("held_collision_proxy_name", "held_payload_collision_proxy")
        self.declare_parameter("hide_z", 6.0)
        self.declare_parameter("payload_contact_response_enabled", True)
        self.declare_parameter("payload_contact_push_gain", 0.85)
        self.declare_parameter("payload_floor_z", -0.79)
        self.declare_parameter("hang_on_release_enabled", True)
        self.declare_parameter("hook_snap_tolerance_m", 0.85)
        self.declare_parameter("hanging_constraint_enabled", True)
        self.declare_parameter("hanging_hole_offset_z_m", 0.0235)
        self.declare_parameter("hanging_effective_length_m", 0.045)
        self.declare_parameter("hanging_damping", 2.4)
        self.declare_parameter("hanging_release_velocity_gain", 0.45)
        self.declare_parameter("hanging_max_angle_rad", 0.45)

        self.world_name = str(self.get_parameter("world_name").value)
        self.payload_code = str(self.get_parameter("payload_code").value).upper()
        self.gripper_response_s = float(self.get_parameter("gripper_response_s").value)
        self.capture_forward_offset = float(self.get_parameter("capture_forward_offset_m").value)
        self.capture_vertical_offset = float(self.get_parameter("capture_vertical_offset_m").value)
        self.capture_forward_tolerance = float(self.get_parameter("capture_forward_tolerance_m").value)
        self.capture_lateral_tolerance = float(self.get_parameter("capture_lateral_tolerance_m").value)
        self.capture_vertical_tolerance = float(self.get_parameter("capture_vertical_tolerance_m").value)
        self.payload_width = float(self.get_parameter("payload_width_m").value)
        self.closed_gap = float(self.get_parameter("closed_gap_m").value)
        self.open_gap = float(self.get_parameter("open_gap_m").value)
        self.held_collision_proxy_name = str(
            self.get_parameter("held_collision_proxy_name").value
        )
        self.hide_z = float(self.get_parameter("hide_z").value)
        self.payload_contact_response_enabled = bool(
            self.get_parameter("payload_contact_response_enabled").value
        )
        self.payload_contact_push_gain = float(
            self.get_parameter("payload_contact_push_gain").value
        )
        self.payload_floor_z = float(self.get_parameter("payload_floor_z").value)
        self.hang_on_release_enabled = bool(
            self.get_parameter("hang_on_release_enabled").value
        )
        self.hook_snap_tolerance = float(self.get_parameter("hook_snap_tolerance_m").value)
        self.hanging_constraint_enabled = bool(
            self.get_parameter("hanging_constraint_enabled").value
        )
        self.hanging_hole_offset_z = float(
            self.get_parameter("hanging_hole_offset_z_m").value
        )
        self.hanging_effective_length = float(
            self.get_parameter("hanging_effective_length_m").value
        )
        self.hanging_damping = float(self.get_parameter("hanging_damping").value)
        self.hanging_release_velocity_gain = float(
            self.get_parameter("hanging_release_velocity_gain").value
        )
        self.hanging_max_angle = float(self.get_parameter("hanging_max_angle_rad").value)

        self.command = 0.0
        self.gripper_position = 0.0
        self.attached = False
        self.hung_hook: Optional[str] = None
        self.hang_angle = 0.0
        self.hang_rate = 0.0
        self.latest_odom: Optional[Odometry] = None
        self.detected_code: Optional[str] = None
        self.last_update = time.monotonic()
        self.last_alignment = (999.0, 999.0, 999.0)
        self.last_aligned = False
        self.last_collision = False
        self.last_contact_source = "none"

        self.payload_pose = Pose()
        self.payload_pose.position.x = 0.0
        self.payload_pose.position.y = 0.0
        self.payload_pose.position.z = self.payload_floor_z
        self.payload_pose.orientation.w = 1.0

        self.status_pub = self.create_publisher(String, "/rov/gripper_status", 10)
        self.create_subscription(Float64, "/rov/gripper_cmd", self.command_callback, 10)
        self.create_subscription(Odometry, "/model/gamantaray_rov/odometry", self.odom_callback, 10)
        self.create_subscription(String, "/rov/qr_code", self.qr_callback, 10)

        self.gz_node = GzNode()
        self.pose_service = f"/world/{self.world_name}/set_pose"
        self.gz_node.subscribe(
            Pose_V,
            f"/world/{self.world_name}/pose/info",
            self.pose_info_callback,
        )
        self.create_timer(0.05, self.update)

    def command_callback(self, msg: Float64) -> None:
        self.command = clamp(float(msg.data), 0.0, 1.0)

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def qr_callback(self, msg: String) -> None:
        if msg.data in {"A", "B", "C", "D"}:
            self.detected_code = msg.data

    def pose_info_callback(self, msg: Pose_V) -> None:
        if self.attached or self.hung_hook is not None:
            return

        target_model = f"kki_payload_{self.payload_code}"
        target_link = f"{target_model}::payload_link"
        for pose_msg in msg.pose:
            if pose_msg.name in {target_model, target_link}:
                self.payload_pose = self.gz_pose_to_ros_pose(pose_msg)
                return

    def update(self) -> None:
        now = time.monotonic()
        dt = min(0.05, max(0.0, now - self.last_update))
        self.last_update = now

        self.gripper_position = approach(
            self.gripper_position, self.command, dt, self.gripper_response_s
        )
        self.update_jaw_visuals()

        if not self.attached:
            if self.hung_hook is not None:
                self.update_hanging_payload(dt)
            else:
                self.apply_kinematic_payload_contact(dt)
            self.update_alignment_status()

        if self.command > 0.75 and not self.attached and self.can_attach():
            self.payload_code = self.detected_code or self.payload_code
            self.hung_hook = None
            self.attached = True
            self.get_logger().info(f"Payload {self.payload_code} attached to gripper")

        if self.command < 0.25 and self.attached:
            snapped_hook = self.release_payload_to_hook_if_close()
            self.attached = False
            self.hide_collision_proxy()
            if snapped_hook:
                self.get_logger().info(
                    f"Payload {self.payload_code} released on hook {snapped_hook}"
                )
            else:
                self.get_logger().info(f"Payload {self.payload_code} released")

        if self.attached:
            self.move_payload_to_gripper()

        dx, dy, dz = self.last_alignment
        jaw_gap = self.current_jaw_gap()
        state = (
            "attached"
            if self.attached
            else ("hung" if self.hung_hook is not None else ("closed" if self.command > 0.75 else "open"))
        )
        status = (
            f"{state} payload={self.payload_code} "
            f"jaw={self.gripper_position:.2f} gap={jaw_gap:.3f} "
            f"aligned={str(self.last_aligned).lower()} "
            f"collision={str(self.last_collision).lower()} "
            f"contact={self.last_contact_source} "
            f"hook={self.hung_hook or 'none'} swing={self.hang_angle:.3f} "
            f"err=({dx:.3f},{dy:.3f},{dz:.3f})"
        )
        self.status_pub.publish(String(data=status))

    def apply_kinematic_payload_contact(self, dt: float) -> None:
        self.last_collision = False
        self.last_contact_source = "none"
        if (
            not self.payload_contact_response_enabled
            or self.latest_odom is None
            or dt <= 0.0
        ):
            return

        local_x, local_y, local_z = self.payload_in_body_frame()
        body_contact = (
            0.10 <= local_x <= 0.36
            and abs(local_y) <= 0.20
            and -0.30 <= local_z <= 0.12
        )
        claw_contact = (
            0.22 <= local_x <= 0.47
            and abs(local_y) <= 0.13
            and abs(local_z - self.capture_vertical_offset) <= 0.13
        )

        if not body_contact and not claw_contact:
            return

        vx = self.latest_odom.twist.twist.linear.x
        vy = self.latest_odom.twist.twist.linear.y
        speed_xy = math.hypot(vx, vy)
        moving_toward_payload = (vx * local_x + vy * local_y) > 0.0

        self.last_collision = True
        self.last_contact_source = "claw" if claw_contact else "rov"
        if speed_xy < 0.015 or not moving_toward_payload:
            return

        push_x = vx * dt * self.payload_contact_push_gain
        push_y = vy * dt * self.payload_contact_push_gain
        if abs(push_x) + abs(push_y) < 0.002:
            scale = 0.002 / max(speed_xy * dt, 1e-6)
            push_x = vx * dt * scale
            push_y = vy * dt * scale

        self.move_payload_by_body_delta(push_x, push_y)

    def can_attach(self) -> bool:
        aligned = self.update_alignment_status()
        if not aligned:
            return False
        gripper_closed_on_object = (
            self.gripper_position > 0.82
            and self.current_jaw_gap() <= self.payload_width + 0.020
        )
        return gripper_closed_on_object

    def update_alignment_status(self) -> bool:
        if self.latest_odom is None:
            self.last_aligned = False
            return False
        local_x, local_y, local_z = self.payload_in_body_frame()
        err_x = local_x - self.capture_forward_offset
        err_y = local_y
        err_z = local_z - self.capture_vertical_offset
        self.last_alignment = (err_x, err_y, err_z)

        aligned = (
            abs(err_x) <= self.capture_forward_tolerance
            and abs(err_y) <= self.capture_lateral_tolerance
            and abs(err_z) <= self.capture_vertical_tolerance
        )
        self.last_aligned = aligned
        return aligned

    def move_payload_to_gripper(self) -> None:
        if self.latest_odom is None:
            return

        pose = self.held_payload_pose(self.latest_odom.pose.pose)
        self.payload_pose = pose
        self.set_model_pose(f"kki_payload_{self.payload_code}", pose, 25)
        self.set_model_pose(self.held_collision_proxy_name, pose, 5)

    def release_payload_to_hook_if_close(self) -> Optional[str]:
        if (
            not self.hang_on_release_enabled
            or not self.hanging_constraint_enabled
            or self.latest_odom is None
        ):
            return None

        hook_code = self.payload_code if self.payload_code in HOOK_PEG_POSES else None
        if hook_code is None:
            return None

        approach = HOOK_APPROACH_POSES[hook_code]
        hang_pose = self.hanging_payload_pose(hook_code, 0.0)
        rov_position = self.latest_odom.pose.pose.position
        payload_position = self.payload_pose.position

        rov_distance = math.sqrt(
            (rov_position.x - approach[0]) ** 2
            + (rov_position.y - approach[1]) ** 2
            + (rov_position.z - approach[2]) ** 2
        )
        payload_distance = math.sqrt(
            (payload_position.x - hang_pose.position.x) ** 2
            + (payload_position.y - hang_pose.position.y) ** 2
            + (payload_position.z - hang_pose.position.z) ** 2
        )
        if min(rov_distance, payload_distance) > self.hook_snap_tolerance:
            return None

        self.hang_angle = self.estimate_hanging_angle_from_payload(hook_code)
        self.hang_rate = self.estimate_hanging_release_rate(hook_code)
        if abs(self.hang_angle) < 0.02 and abs(self.hang_rate) > 0.2:
            self.hang_angle = clamp(self.hang_rate * 0.035, -0.12, 0.12)
        self.hang_angle = clamp(
            self.hang_angle, -self.hanging_max_angle, self.hanging_max_angle
        )
        self.hang_rate = clamp(
            self.hang_rate * self.hanging_release_velocity_gain, -3.0, 3.0
        )
        self.hung_hook = hook_code

        pose = self.hanging_payload_pose(hook_code, self.hang_angle)
        self.payload_pose = pose
        self.last_contact_source = f"hook_{hook_code}"
        self.set_model_pose(f"kki_payload_{hook_code}", pose, 50)
        return hook_code

    def update_hanging_payload(self, dt: float) -> None:
        if self.hung_hook is None or self.hung_hook not in HOOK_PEG_POSES:
            return

        length = max(0.02, self.hanging_effective_length)
        remaining = dt
        while remaining > 1e-6:
            step = min(0.01, remaining)
            acceleration = (
                -(9.81 / length) * math.sin(self.hang_angle)
                - self.hanging_damping * self.hang_rate
            )
            self.hang_rate = clamp(self.hang_rate + acceleration * step, -3.0, 3.0)
            self.hang_angle = clamp(
                self.hang_angle + self.hang_rate * step,
                -self.hanging_max_angle,
                self.hanging_max_angle,
            )
            remaining -= step

        if abs(self.hang_angle) < 0.002 and abs(self.hang_rate) < 0.015:
            self.hang_angle = 0.0
            self.hang_rate = 0.0

        self.payload_pose = self.hanging_payload_pose(self.hung_hook, self.hang_angle)
        self.last_collision = True
        self.last_contact_source = f"hook_{self.hung_hook}"
        self.set_model_pose(f"kki_payload_{self.hung_hook}", self.payload_pose, 10)

    def hanging_payload_pose(self, hook_code: str, angle: float) -> Pose:
        pivot_x, pivot_y, pivot_z, yaw = HOOK_PEG_POSES[hook_code]
        qx, qy, qz, qw = quaternion_from_euler(angle, 0.0, yaw)
        offset_x, offset_y, offset_z = rotate_vector_by_quaternion(
            (0.0, 0.0, self.hanging_hole_offset_z),
            (qx, qy, qz, qw),
        )

        pose = Pose()
        pose.position.x = pivot_x - offset_x
        pose.position.y = pivot_y - offset_y
        pose.position.z = pivot_z - offset_z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def estimate_hanging_angle_from_payload(self, hook_code: str) -> float:
        pivot_x, pivot_y, pivot_z, yaw = HOOK_PEG_POSES[hook_code]
        dx = self.payload_pose.position.x - pivot_x
        dy = self.payload_pose.position.y - pivot_y
        dz = self.payload_pose.position.z - pivot_z
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        local_y = -dx * sin_yaw + dy * cos_yaw
        return math.atan2(local_y, max(1e-6, -dz))

    def estimate_hanging_release_rate(self, hook_code: str) -> float:
        if self.latest_odom is None:
            return 0.0

        _, _, _, hook_yaw = HOOK_PEG_POSES[hook_code]
        rov_pose = self.latest_odom.pose.pose
        rov_yaw = yaw_from_quaternion(
            rov_pose.orientation.x,
            rov_pose.orientation.y,
            rov_pose.orientation.z,
            rov_pose.orientation.w,
        )
        body_vx = self.latest_odom.twist.twist.linear.x
        body_vy = self.latest_odom.twist.twist.linear.y
        world_vx = body_vx * math.cos(rov_yaw) - body_vy * math.sin(rov_yaw)
        world_vy = body_vx * math.sin(rov_yaw) + body_vy * math.cos(rov_yaw)
        local_swing_velocity = (
            -world_vx * math.sin(hook_yaw) + world_vy * math.cos(hook_yaw)
        )
        return local_swing_velocity / max(0.02, self.hanging_effective_length)

    def update_jaw_visuals(self) -> None:
        if self.latest_odom is None:
            return

        rov_pose = self.latest_odom.pose.pose
        open_angle = 0.34
        closed_angle = 0.02
        jaw_angle = open_angle + (closed_angle - open_angle) * self.gripper_position

        left_pose = self.local_pose_to_world(
            rov_pose,
            local_x=0.244,
            local_y=0.052,
            local_z=self.capture_vertical_offset,
            local_yaw=jaw_angle,
        )
        right_pose = self.local_pose_to_world(
            rov_pose,
            local_x=0.244,
            local_y=-0.052,
            local_z=self.capture_vertical_offset,
            local_yaw=-jaw_angle,
        )
        self.set_model_pose("gamantaray_rov_left_gripper_jaw", left_pose, 5)
        self.set_model_pose("gamantaray_rov_right_gripper_jaw", right_pose, 5)

    def set_model_pose(self, model_name: str, pose: Pose, timeout_ms: int) -> None:
        request = GzPose()
        request.name = model_name
        request.position.x = pose.position.x
        request.position.y = pose.position.y
        request.position.z = pose.position.z
        request.orientation.x = pose.orientation.x
        request.orientation.y = pose.orientation.y
        request.orientation.z = pose.orientation.z
        request.orientation.w = pose.orientation.w
        self.gz_node.request(self.pose_service, request, GzPose, Boolean, timeout_ms)

    def gz_pose_to_ros_pose(self, pose_msg: GzPose) -> Pose:
        pose = Pose()
        pose.position.x = pose_msg.position.x
        pose.position.y = pose_msg.position.y
        pose.position.z = pose_msg.position.z
        pose.orientation.x = pose_msg.orientation.x
        pose.orientation.y = pose_msg.orientation.y
        pose.orientation.z = pose_msg.orientation.z
        pose.orientation.w = pose_msg.orientation.w
        return pose

    def hide_collision_proxy(self) -> None:
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = self.hide_z
        pose.orientation.w = 1.0
        self.set_model_pose(self.held_collision_proxy_name, pose, 5)

    def payload_in_body_frame(self) -> tuple[float, float, float]:
        rov_pose = self.latest_odom.pose.pose
        yaw = yaw_from_quaternion(
            rov_pose.orientation.x,
            rov_pose.orientation.y,
            rov_pose.orientation.z,
            rov_pose.orientation.w,
        )
        dx = self.payload_pose.position.x - rov_pose.position.x
        dy = self.payload_pose.position.y - rov_pose.position.y
        dz = self.payload_pose.position.z - rov_pose.position.z
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        return local_x, local_y, dz

    def move_payload_by_body_delta(self, dx_body: float, dy_body: float) -> None:
        rov_pose = self.latest_odom.pose.pose
        yaw = yaw_from_quaternion(
            rov_pose.orientation.x,
            rov_pose.orientation.y,
            rov_pose.orientation.z,
            rov_pose.orientation.w,
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        self.payload_pose.position.x += dx_body * cos_yaw - dy_body * sin_yaw
        self.payload_pose.position.y += dx_body * sin_yaw + dy_body * cos_yaw
        self.payload_pose.position.x = clamp(self.payload_pose.position.x, -4.60, 4.60)
        self.payload_pose.position.y = clamp(self.payload_pose.position.y, -4.60, 4.60)
        self.payload_pose.position.z = self.payload_floor_z
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        self.payload_pose.orientation.x = qx
        self.payload_pose.orientation.y = qy
        self.payload_pose.orientation.z = qz
        self.payload_pose.orientation.w = qw
        self.set_model_pose(f"kki_payload_{self.payload_code}", self.payload_pose, 10)

    def current_jaw_gap(self) -> float:
        return self.open_gap + (self.closed_gap - self.open_gap) * self.gripper_position

    def local_pose_to_world(
        self,
        rov_pose: Pose,
        local_x: float,
        local_y: float,
        local_z: float,
        local_yaw: float,
    ) -> Pose:
        yaw = yaw_from_quaternion(
            rov_pose.orientation.x,
            rov_pose.orientation.y,
            rov_pose.orientation.z,
            rov_pose.orientation.w,
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        pose = Pose()
        pose.position.x = rov_pose.position.x + local_x * cos_yaw - local_y * sin_yaw
        pose.position.y = rov_pose.position.y + local_x * sin_yaw + local_y * cos_yaw
        pose.position.z = rov_pose.position.z + local_z
        qx, qy, qz, qw = quaternion_from_yaw(yaw + local_yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def held_payload_pose(self, rov_pose: Pose) -> Pose:
        pose = self.local_pose_to_world(
            rov_pose,
            local_x=self.capture_forward_offset,
            local_y=0.0,
            local_z=self.capture_vertical_offset,
            local_yaw=0.0,
        )
        return pose


def main() -> None:
    rclpy.init()
    node = GripperManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
