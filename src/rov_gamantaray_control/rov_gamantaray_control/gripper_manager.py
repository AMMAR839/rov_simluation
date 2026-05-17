import math
import time
from typing import Optional

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_pb2 import Double as GzDouble
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


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def parameter_as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


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
    "A": (-4.36, 0.0, -0.34),
    "B": (4.36, 0.0, -0.34),
    "C": (0.0, 4.36, -0.34),
    "D": (0.0, -4.36, -0.34),
}

HOOK_PEG_POSES = {
    "A": (-4.72, 0.0, -0.43, math.pi),
    "B": (4.72, 0.0, -0.43, 0.0),
    "C": (0.0, 4.72, -0.43, math.pi * 0.5),
    "D": (0.0, -4.72, -0.43, -math.pi * 0.5),
}

HOOK_PAYLOAD_VERTICAL_GUARDS = (
    (-4.86, 0.0, -0.455, -0.005, 0.018),
    (4.86, 0.0, -0.455, -0.005, 0.018),
    (0.0, 4.86, -0.455, -0.005, 0.018),
    (0.0, -4.86, -0.455, -0.005, 0.018),
)

HOOK_PAYLOAD_PEG_GUARDS = (
    ((-4.86, 0.0, -0.43), (-4.60, 0.0, -0.43), 0.010),
    ((4.86, 0.0, -0.43), (4.60, 0.0, -0.43), 0.010),
    ((0.0, 4.86, -0.43), (0.0, 4.60, -0.43), 0.010),
    ((0.0, -4.86, -0.43), (0.0, -4.60, -0.43), 0.010),
)

class GripperManager(Node):
    """Drive the visual gripper and simulate payload attach/release."""

    def __init__(self) -> None:
        super().__init__("gripper_manager")
        self.declare_parameter("world_name", "kki_rov_pool")
        self.declare_parameter("payload_code", "A")
        self.declare_parameter("gripper_response_s", 0.16)
        self.declare_parameter("jaw_pivot_x_m", 0.244)
        self.declare_parameter("jaw_pivot_y_m", 0.050)
        self.declare_parameter("jaw_open_angle_rad", 0.38)
        self.declare_parameter("jaw_closed_angle_rad", 0.035)
        self.declare_parameter("capture_forward_offset_m", 0.335)
        self.declare_parameter("capture_vertical_offset_m", -0.112)
        self.declare_parameter("capture_forward_tolerance_m", 0.040)
        self.declare_parameter("capture_lateral_tolerance_m", 0.035)
        self.declare_parameter("capture_vertical_tolerance_m", 0.075)
        self.declare_parameter("capture_alignment_hold_s", 0.15)
        self.declare_parameter("held_payload_response_s", 0.06)
        self.declare_parameter("held_payload_max_sway_m", 0.020)
        self.declare_parameter("payload_width_m", 0.050)
        self.declare_parameter("closed_gap_m", 0.050)
        self.declare_parameter("open_gap_m", 0.125)
        self.declare_parameter("mouth_clearance_m", 0.006)
        self.declare_parameter("payload_funnel_response_s", 0.12)
        self.declare_parameter("payload_funnel_min_gripper_position", 0.20)
        self.declare_parameter("gripper_actuation_mode", "kinematic")
        self.declare_parameter(
            "left_gripper_joint_topic",
            "/model/gamantaray_rov/joint/left_gripper_hinge/0/cmd_pos",
        )
        self.declare_parameter(
            "right_gripper_joint_topic",
            "/model/gamantaray_rov/joint/right_gripper_hinge/0/cmd_pos",
        )
        self.declare_parameter("held_collision_proxy_name", "held_payload_collision_proxy")
        self.declare_parameter("hide_z", 6.0)
        self.declare_parameter("payload_contact_model", "strict")
        self.declare_parameter("payload_contact_response_enabled", True)
        self.declare_parameter("payload_contact_push_gain", 0.85)
        self.declare_parameter("payload_contact_resolution_gain", 1.00)
        self.declare_parameter("payload_contact_max_step_m", 0.016)
        self.declare_parameter("grip_attach_requires_bilateral_contact", True)
        self.declare_parameter("grip_min_bilateral_contact_s", 0.20)
        self.declare_parameter("grip_clamp_margin_m", 0.006)
        self.declare_parameter("jaw_visual_stop_clearance_m", 0.014)
        self.declare_parameter("grip_required_depth_error_m", 0.050)
        self.declare_parameter("grip_required_height_error_m", 0.050)
        self.declare_parameter("gripper_frame_backstop_x_m", 0.330)
        self.declare_parameter("gripper_frame_backstop_gain", 0.90)
        self.declare_parameter("payload_floor_z", -0.79)
        self.declare_parameter("payload_drop_enabled", True)
        self.declare_parameter("payload_drop_buoyancy_ratio", 0.72)
        self.declare_parameter("payload_drop_linear_drag", 4.2)
        self.declare_parameter("payload_drop_quadratic_drag", 2.0)
        self.declare_parameter("payload_drop_terminal_speed_mps", 0.22)
        self.declare_parameter("payload_drop_initial_velocity_gain", 0.35)
        self.declare_parameter("payload_drop_angular_damping", 3.0)
        self.declare_parameter("payload_drop_max_tilt_rad", 0.30)
        self.declare_parameter("hang_on_release_enabled", True)
        self.declare_parameter("hook_snap_tolerance_m", 0.045)
        self.declare_parameter("hook_snap_rov_tolerance_m", 0.30)
        self.declare_parameter("hook_snap_hole_tolerance_m", 0.024)
        self.declare_parameter("hook_snap_axial_tolerance_m", 0.026)
        self.declare_parameter("hook_snap_yaw_tolerance_rad", 0.35)
        self.declare_parameter("hook_peg_length_m", 0.26)
        self.declare_parameter("hook_peg_pass_window_m", 0.028)
        self.declare_parameter("hook_latch_release_tolerance_m", 0.036)
        self.declare_parameter("hook_latch_axial_release_tolerance_m", 0.075)
        self.declare_parameter("hook_latch_memory_s", 3.00)
        self.declare_parameter("hook_entry_tip_min_t", 0.78)
        self.declare_parameter("hook_entry_memory_s", 4.00)
        self.declare_parameter("hanging_constraint_enabled", True)
        self.declare_parameter("hanging_hole_offset_z_m", 0.0235)
        self.declare_parameter("hanging_effective_length_m", 0.045)
        self.declare_parameter("hanging_damping", 2.4)
        self.declare_parameter("hanging_release_velocity_gain", 0.45)
        self.declare_parameter("hanging_max_angle_rad", 0.45)

        self.world_name = str(self.get_parameter("world_name").value)
        self.payload_code = str(self.get_parameter("payload_code").value).upper()
        self.gripper_response_s = float(self.get_parameter("gripper_response_s").value)
        self.jaw_pivot_x = float(self.get_parameter("jaw_pivot_x_m").value)
        self.jaw_pivot_y = float(self.get_parameter("jaw_pivot_y_m").value)
        self.jaw_open_angle = float(self.get_parameter("jaw_open_angle_rad").value)
        self.jaw_closed_angle = float(self.get_parameter("jaw_closed_angle_rad").value)
        self.capture_forward_offset = float(self.get_parameter("capture_forward_offset_m").value)
        self.capture_vertical_offset = float(self.get_parameter("capture_vertical_offset_m").value)
        self.capture_forward_tolerance = float(self.get_parameter("capture_forward_tolerance_m").value)
        self.capture_lateral_tolerance = float(self.get_parameter("capture_lateral_tolerance_m").value)
        self.capture_vertical_tolerance = float(self.get_parameter("capture_vertical_tolerance_m").value)
        self.capture_alignment_hold_s = float(
            self.get_parameter("capture_alignment_hold_s").value
        )
        self.held_payload_response_s = float(
            self.get_parameter("held_payload_response_s").value
        )
        self.held_payload_max_sway = float(
            self.get_parameter("held_payload_max_sway_m").value
        )
        self.payload_width = float(self.get_parameter("payload_width_m").value)
        self.closed_gap = float(self.get_parameter("closed_gap_m").value)
        self.open_gap = float(self.get_parameter("open_gap_m").value)
        self.mouth_clearance = float(self.get_parameter("mouth_clearance_m").value)
        self.payload_funnel_response_s = float(
            self.get_parameter("payload_funnel_response_s").value
        )
        self.payload_funnel_min_gripper_position = float(
            self.get_parameter("payload_funnel_min_gripper_position").value
        )
        self.gripper_actuation_mode = str(
            self.get_parameter("gripper_actuation_mode").value
        ).lower()
        if self.gripper_actuation_mode not in {"kinematic", "joint"}:
            raise ValueError("gripper_actuation_mode must be 'kinematic' or 'joint'")
        self.left_gripper_joint_topic = str(
            self.get_parameter("left_gripper_joint_topic").value
        )
        self.right_gripper_joint_topic = str(
            self.get_parameter("right_gripper_joint_topic").value
        )
        self.held_collision_proxy_name = str(
            self.get_parameter("held_collision_proxy_name").value
        )
        self.hide_z = float(self.get_parameter("hide_z").value)
        self.payload_contact_model = str(
            self.get_parameter("payload_contact_model").value
        ).lower()
        if self.payload_contact_model not in {"strict", "assisted"}:
            raise ValueError("payload_contact_model must be 'strict' or 'assisted'")
        self.payload_contact_response_enabled = parameter_as_bool(
            self.get_parameter("payload_contact_response_enabled").value
        )
        self.payload_contact_push_gain = float(
            self.get_parameter("payload_contact_push_gain").value
        )
        self.payload_contact_resolution_gain = float(
            self.get_parameter("payload_contact_resolution_gain").value
        )
        self.payload_contact_max_step = float(
            self.get_parameter("payload_contact_max_step_m").value
        )
        self.grip_attach_requires_bilateral_contact = parameter_as_bool(
            self.get_parameter("grip_attach_requires_bilateral_contact").value
        )
        self.grip_min_bilateral_contact_s = float(
            self.get_parameter("grip_min_bilateral_contact_s").value
        )
        self.grip_clamp_margin = float(self.get_parameter("grip_clamp_margin_m").value)
        self.jaw_visual_stop_clearance = float(
            self.get_parameter("jaw_visual_stop_clearance_m").value
        )
        self.grip_required_depth_error = float(
            self.get_parameter("grip_required_depth_error_m").value
        )
        self.grip_required_height_error = float(
            self.get_parameter("grip_required_height_error_m").value
        )
        self.gripper_frame_backstop_x = float(
            self.get_parameter("gripper_frame_backstop_x_m").value
        )
        self.gripper_frame_backstop_gain = float(
            self.get_parameter("gripper_frame_backstop_gain").value
        )
        self.payload_floor_z = float(self.get_parameter("payload_floor_z").value)
        self.payload_drop_enabled = parameter_as_bool(
            self.get_parameter("payload_drop_enabled").value
        )
        self.payload_drop_buoyancy_ratio = float(
            self.get_parameter("payload_drop_buoyancy_ratio").value
        )
        self.payload_drop_linear_drag = float(
            self.get_parameter("payload_drop_linear_drag").value
        )
        self.payload_drop_quadratic_drag = float(
            self.get_parameter("payload_drop_quadratic_drag").value
        )
        self.payload_drop_terminal_speed = float(
            self.get_parameter("payload_drop_terminal_speed_mps").value
        )
        self.payload_drop_initial_velocity_gain = float(
            self.get_parameter("payload_drop_initial_velocity_gain").value
        )
        self.payload_drop_angular_damping = float(
            self.get_parameter("payload_drop_angular_damping").value
        )
        self.payload_drop_max_tilt = float(
            self.get_parameter("payload_drop_max_tilt_rad").value
        )
        self.hang_on_release_enabled = parameter_as_bool(
            self.get_parameter("hang_on_release_enabled").value
        )
        self.hook_snap_tolerance = float(self.get_parameter("hook_snap_tolerance_m").value)
        self.hook_snap_rov_tolerance = float(
            self.get_parameter("hook_snap_rov_tolerance_m").value
        )
        self.hook_snap_hole_tolerance = float(
            self.get_parameter("hook_snap_hole_tolerance_m").value
        )
        self.hook_snap_axial_tolerance = float(
            self.get_parameter("hook_snap_axial_tolerance_m").value
        )
        self.hook_snap_yaw_tolerance = float(
            self.get_parameter("hook_snap_yaw_tolerance_rad").value
        )
        self.hook_peg_length = float(self.get_parameter("hook_peg_length_m").value)
        self.hook_peg_pass_window = float(
            self.get_parameter("hook_peg_pass_window_m").value
        )
        self.hook_latch_release_tolerance = float(
            self.get_parameter("hook_latch_release_tolerance_m").value
        )
        self.hook_latch_axial_release_tolerance = float(
            self.get_parameter("hook_latch_axial_release_tolerance_m").value
        )
        self.hook_latch_memory_s = float(
            self.get_parameter("hook_latch_memory_s").value
        )
        self.hook_entry_tip_min_t = float(
            self.get_parameter("hook_entry_tip_min_t").value
        )
        self.hook_entry_memory_s = float(
            self.get_parameter("hook_entry_memory_s").value
        )
        self.hanging_constraint_enabled = parameter_as_bool(
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
        self.model_payload_code = self.payload_code
        self.payload_code: Optional[str] = None
        self.attached = False
        self.dropping = False
        self.drop_velocity = [0.0, 0.0, 0.0]
        self.drop_yaw = 0.0
        self.drop_yaw_rate = 0.0
        self.drop_roll = 0.0
        self.drop_pitch = 0.0
        self.hung_hook: Optional[str] = None
        self.hook_latch_point: Optional[tuple[float, float, float]] = None
        self.hook_latch_point_candidate: Optional[tuple[float, float, float]] = None
        self.recent_hook_latch_code: Optional[str] = None
        self.recent_hook_latch_point: Optional[tuple[float, float, float]] = None
        self.recent_hook_latch_time: Optional[float] = None
        self.peg_entry_code: Optional[str] = None
        self.peg_entry_time: Optional[float] = None
        self.peg_tip_entry_code: Optional[str] = None
        self.peg_tip_entry_time: Optional[float] = None
        self.peg_tip_motion_state: dict[str, tuple[float, float, float]] = {}
        self.hang_angle = 0.0
        self.hang_rate = 0.0
        self.latest_odom: Optional[Odometry] = None
        self.detected_code: Optional[str] = None
        self.last_update = time.monotonic()
        self.aligned_since: Optional[float] = None
        self.last_alignment = (999.0, 999.0, 999.0)
        self.last_aligned = False
        self.last_hook_alignment = (999.0, 999.0, 999.0, 999.0)
        self.last_hook_aligned = False
        self.last_release_block_reason = "unknown_qr"
        self.last_grip_stress = 0.0
        self.last_collision = False
        self.last_contact_source = "none"
        self.last_left_pad_contact = False
        self.last_right_pad_contact = False
        self.last_bilateral_contact = False
        self.last_contact_penetration = 0.0
        self.bilateral_contact_since: Optional[float] = None

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
        self.left_joint_pub = None
        self.right_joint_pub = None
        if self.gripper_actuation_mode == "joint":
            self.left_joint_pub = self.gz_node.advertise(
                self.left_gripper_joint_topic, GzDouble
            )
            self.right_joint_pub = self.gz_node.advertise(
                self.right_gripper_joint_topic, GzDouble
            )
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
        code = msg.data.strip().upper()
        if code in {"A", "B", "C", "D"}:
            self.detected_code = code
            self.payload_code = code

    def pose_info_callback(self, msg: Pose_V) -> None:
        if self.attached or self.hung_hook is not None or self.dropping:
            return

        target_model = f"kki_payload_{self.model_payload_code}"
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

        if not self.attached:
            if self.hung_hook is not None:
                self.update_hanging_payload(dt)
            elif self.dropping:
                self.update_dropping_payload(dt)
            else:
                self.apply_kinematic_payload_contact(dt)
            self.update_alignment_status()
        self.update_hook_alignment_status()

        if self.command > 0.75 and not self.attached and self.can_attach():
            if self.detected_code is not None:
                self.payload_code = self.detected_code
            self.hung_hook = None
            self.hook_latch_point = None
            self.recent_hook_latch_code = None
            self.recent_hook_latch_point = None
            self.recent_hook_latch_time = None
            self.peg_entry_code = None
            self.peg_entry_time = None
            self.peg_tip_entry_code = None
            self.peg_tip_entry_time = None
            self.peg_tip_motion_state.clear()
            self.attached = True
            self.get_logger().info(
                f"Payload {self.payload_code or 'unknown'} attached to gripper"
            )

        if self.command < 0.25 and self.attached:
            snapped_hook = self.release_payload_to_hook_if_close()
            self.attached = False
            self.hide_collision_proxy()
            if snapped_hook:
                self.get_logger().info(
                    f"Payload {self.payload_code or 'unknown'} released on hook {snapped_hook}"
                )
            else:
                self.start_payload_drop()
                self.get_logger().info(f"Payload {self.payload_code or 'unknown'} released")

        if self.attached:
            self.move_payload_to_gripper(dt)

        self.update_jaw_visuals()

        dx, dy, dz = self.last_alignment
        hook_radial, hook_axial, hook_rov, hook_yaw = self.last_hook_alignment
        jaw_gap = self.current_jaw_gap()
        state = (
            "attached"
            if self.attached
            else (
                "hung"
                if self.hung_hook is not None
                else (
                    "dropping"
                    if self.dropping
                    else ("closed" if self.command > 0.75 else "open")
                )
            )
        )
        payload_label = self.payload_code or "unknown"
        status = (
            f"{state} payload={payload_label} "
            f"jaw={self.gripper_position:.2f} gap={jaw_gap:.3f} "
            f"aligned={str(self.last_aligned).lower()} "
            f"collision={str(self.last_collision).lower()} "
            f"contact={self.last_contact_source} "
            f"pinched={str(self.last_bilateral_contact).lower()} "
            f"pad=({str(self.last_left_pad_contact).lower()},{str(self.last_right_pad_contact).lower()}) "
            f"penetration={self.last_contact_penetration:.3f} "
            f"hook_aligned={str(self.last_hook_aligned).lower()} "
            f"hook={self.hung_hook or 'none'} swing={self.hang_angle:.3f} "
            f"drop_vz={self.drop_velocity[2]:.3f} "
            f"grip_stress={self.last_grip_stress:.2f} "
            f"err=({dx:.3f},{dy:.3f},{dz:.3f}) "
            f"hook_err=({hook_radial:.3f},{hook_axial:.3f},{hook_rov:.3f},{hook_yaw:.3f}) "
            f"release_block={self.last_release_block_reason}"
        )
        self.status_pub.publish(String(data=status))

    def apply_kinematic_payload_contact(self, dt: float) -> None:
        if self.payload_contact_model == "strict":
            self.apply_strict_payload_contact(dt)
            return

        self.last_collision = False
        self.last_contact_source = "none"
        self.last_left_pad_contact = False
        self.last_right_pad_contact = False
        self.last_bilateral_contact = False
        self.last_contact_penetration = 0.0
        self.bilateral_contact_since = None
        if (
            not self.payload_contact_response_enabled
            or self.latest_odom is None
            or dt <= 0.0
        ):
            return

        local_x, local_y, local_z = self.payload_in_body_frame()
        jaw_gap = self.current_jaw_gap()
        payload_half_width = self.payload_width * 0.5
        inner_clear_half = max(
            0.0,
            jaw_gap * 0.5 - payload_half_width - max(0.0, self.mouth_clearance),
        )
        mouth_min_x = self.jaw_pivot_x - 0.035
        mouth_max_x = self.capture_forward_offset + 0.105
        mouth_depth = mouth_min_x <= local_x <= mouth_max_x
        mouth_height = abs(local_z - self.capture_vertical_offset) <= 0.135
        inside_mouth_clearance = (
            mouth_depth
            and mouth_height
            and abs(local_y) <= inner_clear_half
            and jaw_gap >= self.payload_width + 2.0 * max(0.0, self.mouth_clearance)
        )

        if inside_mouth_clearance and self.gripper_position < self.payload_funnel_min_gripper_position:
            self.last_collision = False
            self.last_contact_source = "mouth_clearance"
            return

        if mouth_depth and mouth_height and self.gripper_position >= self.payload_funnel_min_gripper_position:
            self.last_collision = True
            self.last_contact_source = "claw_guided"
            self.guide_payload_inside_claw(local_x, local_y, dt)
            return

        body_contact = (
            0.04 <= local_x <= 0.19
            and abs(local_y) <= 0.19
            and -0.30 <= local_z <= 0.12
        )
        claw_half_width = max(0.075, jaw_gap * 0.55 + payload_half_width)
        claw_contact = (
            mouth_min_x <= local_x <= mouth_max_x
            and abs(local_y) <= claw_half_width
            and abs(local_z - self.capture_vertical_offset) <= 0.15
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

        if claw_contact:
            self.guide_payload_inside_claw(local_x, local_y, dt)
            return

        push_x = vx * dt * self.payload_contact_push_gain
        push_y = vy * dt * self.payload_contact_push_gain
        if abs(push_x) + abs(push_y) < 0.002:
            scale = 0.002 / max(speed_xy * dt, 1e-6)
            push_x = vx * dt * scale
            push_y = vy * dt * scale

        self.move_payload_by_body_delta(push_x, push_y)

    def jaw_component_contact(
        self,
        side: str,
        payload_local_x: float,
        payload_local_y: float,
        payload_local_z: float,
        jaw_angle: float,
        contact_skin: float,
        component: str,
    ) -> tuple[bool, float, float, float, float]:
        payload_half_width = self.payload_width * 0.5
        payload_half_thickness = 0.003
        payload_half_height = 0.050

        if side == "left":
            pivot_y = self.jaw_pivot_y
            base_yaw = jaw_angle
            components = {
                "inner_pad": (0.090, -0.026, 0.0, 0.058, 0.006, 0.014),
                "outer_finger": (0.048, 0.0025, 0.0, 0.094, 0.007, 0.011),
                "front_tip": (0.098, -0.017, -0.52, 0.028, 0.007, 0.012),
                "rear_link": (0.020, -0.010, -0.36, 0.027, 0.006, 0.010),
            }
        elif side == "right":
            pivot_y = -self.jaw_pivot_y
            base_yaw = -jaw_angle
            components = {
                "inner_pad": (0.090, 0.026, 0.0, 0.058, 0.006, 0.014),
                "outer_finger": (0.048, -0.0025, 0.0, 0.094, 0.007, 0.011),
                "front_tip": (0.098, 0.017, 0.52, 0.028, 0.007, 0.012),
                "rear_link": (0.020, 0.010, 0.36, 0.027, 0.006, 0.010),
            }
        else:
            raise ValueError("side must be 'left' or 'right'")
        if component not in components:
            raise ValueError("unknown jaw component")

        local_x, local_y, local_yaw, size_x, size_y, size_z = components[component]
        base_cos = math.cos(base_yaw)
        base_sin = math.sin(base_yaw)
        center_x = (
            self.jaw_pivot_x
            + local_x * base_cos
            - local_y * base_sin
        )
        center_y = pivot_y + local_x * base_sin + local_y * base_cos

        yaw = base_yaw + local_yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        dx = payload_local_x - center_x
        dy = payload_local_y - center_y
        payload_x_in_box = dx * cos_yaw + dy * sin_yaw
        payload_y_in_box = -dx * sin_yaw + dy * cos_yaw
        payload_z_error = payload_local_z - self.capture_vertical_offset

        half_x = size_x * 0.5
        half_y = size_y * 0.5
        half_z = size_z * 0.5
        payload_half_x_in_box = (
            payload_half_thickness * abs(cos_yaw)
            + payload_half_width * abs(sin_yaw)
        )
        payload_half_y_in_box = (
            payload_half_thickness * abs(sin_yaw)
            + payload_half_width * abs(cos_yaw)
        )

        overlap_x = half_x + payload_half_x_in_box + contact_skin - abs(payload_x_in_box)
        overlap_y = half_y + payload_half_y_in_box + contact_skin - abs(payload_y_in_box)
        overlap_z = half_z + payload_half_height + contact_skin - abs(payload_z_error)
        if overlap_x <= 0.0 or overlap_y <= 0.0 or overlap_z <= 0.0:
            return False, 0.0, 0.0, 0.0, center_y

        if overlap_x < overlap_y:
            direction = 1.0 if payload_x_in_box >= 0.0 else -1.0
            correction_x = direction * overlap_x * cos_yaw
            correction_y = direction * overlap_x * sin_yaw
            penetration = overlap_x
        else:
            direction = 1.0 if payload_y_in_box >= 0.0 else -1.0
            correction_x = direction * overlap_y * -sin_yaw
            correction_y = direction * overlap_y * cos_yaw
            penetration = overlap_y
        return True, penetration, correction_x, correction_y, center_y

    def jaw_inner_pad_contact(
        self,
        side: str,
        payload_local_x: float,
        payload_local_y: float,
        payload_local_z: float,
        jaw_angle: float,
        contact_skin: float,
    ) -> tuple[bool, float, float]:
        """Return contact state, lateral penetration, and pad center y."""
        contact, penetration, _dx, _dy, center_y = self.jaw_component_contact(
            side,
            payload_local_x,
            payload_local_y,
            payload_local_z,
            jaw_angle,
            contact_skin,
            "inner_pad",
        )
        return contact, penetration, center_y

    def apply_strict_payload_contact(self, dt: float) -> None:
        self.last_collision = False
        self.last_contact_source = "none"
        self.last_left_pad_contact = False
        self.last_right_pad_contact = False
        self.last_bilateral_contact = False
        self.last_contact_penetration = 0.0

        if (
            not self.payload_contact_response_enabled
            or self.latest_odom is None
            or dt <= 0.0
        ):
            self.bilateral_contact_since = None
            return

        local_x, local_y, local_z = self.payload_in_body_frame()
        raw_gap = self.raw_jaw_gap()
        payload_half_width = self.payload_width * 0.5
        payload_half_thickness = 0.003
        contact_skin = max(0.006, self.grip_clamp_margin)
        jaw_half_gap = raw_gap * 0.5
        pad_x_max = self.jaw_pivot_x + 0.165
        jaw_angle = self.jaw_open_angle + (
            self.jaw_closed_angle - self.jaw_open_angle
        ) * self.effective_gripper_position()

        left_contact, left_penetration, left_pad_y = self.jaw_inner_pad_contact(
            "left", local_x, local_y, local_z, jaw_angle, contact_skin
        )
        right_contact, right_penetration, right_pad_y = self.jaw_inner_pad_contact(
            "right", local_x, local_y, local_z, jaw_angle, contact_skin
        )

        dy_body = 0.0
        resolution_gain = clamp(self.payload_contact_resolution_gain, 0.0, 1.0)
        if left_contact:
            left_direction = -1.0 if local_y < left_pad_y else 1.0
            dy_body += left_direction * left_penetration * resolution_gain
        if right_contact:
            right_direction = -1.0 if local_y < right_pad_y else 1.0
            dy_body += right_direction * right_penetration * resolution_gain

        if left_contact and right_contact:
            self.last_contact_source = "bilateral_clamp"
        elif left_contact:
            self.last_contact_source = "left_pad"
        elif right_contact:
            self.last_contact_source = "right_pad"
        dy_body = clamp(
            dy_body,
            -max(0.001, self.payload_contact_max_step),
            max(0.001, self.payload_contact_max_step),
        )

        self.last_left_pad_contact = left_contact
        self.last_right_pad_contact = right_contact
        self.last_bilateral_contact = left_contact and right_contact
        self.last_contact_penetration = max(
            0.0,
            left_penetration if left_contact else 0.0,
            right_penetration if right_contact else 0.0,
        )

        if self.last_bilateral_contact:
            if self.bilateral_contact_since is None:
                self.bilateral_contact_since = time.monotonic()
        else:
            self.bilateral_contact_since = None

        if abs(dy_body) > 1e-5:
            self.last_collision = True
            self.move_payload_by_body_delta(0.0, dy_body)
            return

        if left_contact or right_contact:
            self.last_collision = True
            return

        component_dx = 0.0
        component_dy = 0.0
        component_source = "none"
        component_penetration = 0.0
        for side in ("left", "right"):
            for component in ("outer_finger", "front_tip", "rear_link"):
                (
                    component_contact,
                    penetration,
                    correction_x,
                    correction_y,
                    _center_y,
                ) = self.jaw_component_contact(
                    side,
                    local_x,
                    local_y,
                    local_z,
                    jaw_angle,
                    contact_skin,
                    component,
                )
                if not component_contact:
                    continue
                component_dx += correction_x * resolution_gain
                component_dy += correction_y * resolution_gain
                if penetration > component_penetration:
                    component_penetration = penetration
                    component_source = f"{side}_{component}"

        if component_penetration > 0.0:
            component_dx = clamp(
                component_dx,
                -max(0.001, self.payload_contact_max_step),
                max(0.001, self.payload_contact_max_step),
            )
            component_dy = clamp(
                component_dy,
                -max(0.001, self.payload_contact_max_step),
                max(0.001, self.payload_contact_max_step),
            )
            self.last_collision = True
            self.last_contact_source = component_source
            self.last_contact_penetration = component_penetration
            if abs(component_dx) + abs(component_dy) > 1e-5:
                self.move_payload_by_body_delta(component_dx, component_dy)
            return

        backstop_contact = (
            self.jaw_pivot_x + 0.010 <= local_x < self.gripper_frame_backstop_x
            and abs(local_y) <= max(0.075, jaw_half_gap + payload_half_width)
            and abs(local_z - self.capture_vertical_offset) <= 0.075
        )
        if backstop_contact:
            dx_body = (
                self.gripper_frame_backstop_x
                + payload_half_thickness
                - local_x
            ) * clamp(self.gripper_frame_backstop_gain, 0.0, 1.0)
            self.last_collision = True
            self.last_contact_source = "frame_backstop"
            self.last_contact_penetration = max(0.0, dx_body)
            if dx_body > 1e-5:
                self.move_payload_by_body_delta(dx_body, 0.0)
            return

        body_contact = (
            0.04 <= local_x <= 0.19
            and abs(local_y) <= 0.19
            and -0.30 <= local_z <= 0.12
        )
        gripper_frame_contact = (
            self.jaw_pivot_x - 0.040 <= local_x <= self.jaw_pivot_x + 0.060
            and 0.040 <= abs(local_y) <= 0.070
            and abs(local_z - self.capture_vertical_offset) <= 0.060
        )
        front_tip_contact = (
            pad_x_max <= local_x <= pad_x_max + 0.050
            and abs(local_y) <= max(0.045, jaw_half_gap + payload_half_width)
            and abs(local_z - self.capture_vertical_offset) <= 0.070
            and self.gripper_position > 0.10
        )

        if not (body_contact or gripper_frame_contact or front_tip_contact):
            return

        vx = self.latest_odom.twist.twist.linear.x
        vy = self.latest_odom.twist.twist.linear.y
        speed_xy = math.hypot(vx, vy)
        moving_toward_payload = (vx * local_x + vy * local_y) > 0.0

        self.last_collision = True
        if body_contact:
            self.last_contact_source = "rov_body"
        elif gripper_frame_contact:
            self.last_contact_source = "gripper_frame"
        else:
            self.last_contact_source = "front_tip"

        if speed_xy < 0.015 or not moving_toward_payload:
            return

        push_x = vx * dt * self.payload_contact_push_gain
        push_y = vy * dt * self.payload_contact_push_gain
        if abs(push_x) + abs(push_y) < 0.002:
            scale = 0.002 / max(speed_xy * dt, 1e-6)
            push_x = vx * dt * scale
            push_y = vy * dt * scale
        self.move_payload_by_body_delta(push_x, push_y)

    def guide_payload_inside_claw(self, local_x: float, local_y: float, dt: float) -> None:
        if self.latest_odom is None or dt <= 0.0:
            return

        jaw_gap = self.current_jaw_gap()
        payload_half_width = self.payload_width * 0.5
        safe_half = max(
            0.0,
            jaw_gap * 0.5 - payload_half_width - max(0.0, self.mouth_clearance),
        )
        if self.gripper_position >= self.payload_funnel_min_gripper_position:
            target_y = 0.0
            target_x = self.capture_forward_offset
        else:
            target_y = clamp(local_y, -safe_half, safe_half)
            target_x = clamp(local_x, self.jaw_pivot_x, self.capture_forward_offset)

        response = max(1e-3, self.payload_funnel_response_s)
        alpha = 1.0 - math.exp(-dt / response)
        dx = (target_x - local_x) * alpha
        dy = (target_y - local_y) * alpha

        # If the payload is exactly on a jaw pad, move at least a tiny amount
        # so the visual claw cannot keep cutting through it frame after frame.
        if abs(local_y) > safe_half and abs(dy) < 0.001:
            dy = -math.copysign(0.001, local_y)
        if abs(dx) + abs(dy) > 1e-5:
            self.move_payload_by_body_delta(dx, dy)

    def can_attach(self) -> bool:
        if self.dropping:
            return False
        aligned = self.update_alignment_status()
        if not aligned:
            return False
        if self.payload_contact_model == "strict":
            return self.can_attach_from_strict_contact()

        held_long_enough = (
            self.capture_alignment_hold_s <= 0.0
            or (
                self.aligned_since is not None
                and time.monotonic() - self.aligned_since >= self.capture_alignment_hold_s
            )
        )
        if not held_long_enough:
            return False
        gripper_closed_on_object = (
            self.gripper_position > 0.82
            and self.current_jaw_gap() <= self.payload_width + 0.020
        )
        return gripper_closed_on_object

    def can_attach_from_strict_contact(self) -> bool:
        if self.grip_attach_requires_bilateral_contact and not self.last_bilateral_contact:
            return False
        if self.grip_attach_requires_bilateral_contact:
            if self.bilateral_contact_since is None:
                return False
            if time.monotonic() - self.bilateral_contact_since < self.grip_min_bilateral_contact_s:
                return False

        local_x, _local_y, local_z = self.payload_in_body_frame()
        depth_error = abs(local_x - self.capture_forward_offset)
        height_error = abs(local_z - self.capture_vertical_offset)
        if depth_error > self.grip_required_depth_error:
            return False
        if height_error > self.grip_required_height_error:
            return False

        gripper_world_z = (
            self.latest_odom.pose.pose.position.z + self.capture_vertical_offset
        )
        if gripper_world_z < self.payload_floor_z - 0.006:
            return False

        gap = self.raw_jaw_gap()
        if gap > self.payload_width + self.grip_clamp_margin:
            return False
        if self.last_contact_penetration <= 0.0005 and self.grip_attach_requires_bilateral_contact:
            return False
        return True

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
        if aligned and self.aligned_since is None:
            self.aligned_since = time.monotonic()
        elif not aligned:
            self.aligned_since = None
        self.last_aligned = aligned
        return aligned

    def move_payload_to_gripper(self, dt: float) -> None:
        if self.latest_odom is None:
            return

        self.dropping = False
        target = self.held_payload_pose(self.latest_odom.pose.pose)
        if self.held_payload_response_s <= 1e-6:
            pose = target
        else:
            alpha = 1.0 - math.exp(-max(0.0, dt) / self.held_payload_response_s)
            pose = Pose()
            pose.position.x = self.payload_pose.position.x + (
                target.position.x - self.payload_pose.position.x
            ) * alpha
            pose.position.y = self.payload_pose.position.y + (
                target.position.y - self.payload_pose.position.y
            ) * alpha
            pose.position.z = self.payload_pose.position.z + (
                target.position.z - self.payload_pose.position.z
            ) * alpha

            current_yaw = yaw_from_quaternion(
                self.payload_pose.orientation.x,
                self.payload_pose.orientation.y,
                self.payload_pose.orientation.z,
                self.payload_pose.orientation.w,
            )
            target_yaw = yaw_from_quaternion(
                target.orientation.x,
                target.orientation.y,
                target.orientation.z,
                target.orientation.w,
            )
            yaw = current_yaw + normalize_angle(target_yaw - current_yaw) * alpha
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            sway = self.distance_between_positions(pose.position, target.position)
            max_sway = max(0.001, self.held_payload_max_sway)
            if sway > max_sway:
                scale = max_sway / sway
                pose.position.x = target.position.x + (pose.position.x - target.position.x) * scale
                pose.position.y = target.position.y + (pose.position.y - target.position.y) * scale
                pose.position.z = target.position.z + (pose.position.z - target.position.z) * scale

        if pose.position.z < self.payload_floor_z:
            pose.position.z = self.payload_floor_z
        self.resolve_payload_hook_collision(pose)

        self.last_grip_stress = clamp(
            self.distance_between_positions(pose.position, target.position)
            / max(0.001, self.held_payload_max_sway),
            0.0,
            1.0,
        )
        self.payload_pose = pose
        self.set_model_pose(f"kki_payload_{self.model_payload_code}", pose, 25)
        self.set_model_pose(self.held_collision_proxy_name, pose, 5)

    def start_payload_drop(self) -> None:
        if not self.payload_drop_enabled:
            self.dropping = False
            return

        self.dropping = True
        self.hung_hook = None
        self.hook_latch_point = None
        yaw = yaw_from_quaternion(
            self.payload_pose.orientation.x,
            self.payload_pose.orientation.y,
            self.payload_pose.orientation.z,
            self.payload_pose.orientation.w,
        )
        self.drop_yaw = yaw
        self.drop_roll = 0.0
        self.drop_pitch = 0.0
        self.drop_yaw_rate = 0.0
        self.drop_velocity = [0.0, 0.0, 0.0]

        if self.latest_odom is not None:
            rov_pose = self.latest_odom.pose.pose
            rov_yaw = yaw_from_quaternion(
                rov_pose.orientation.x,
                rov_pose.orientation.y,
                rov_pose.orientation.z,
                rov_pose.orientation.w,
            )
            body_vx = self.latest_odom.twist.twist.linear.x
            body_vy = self.latest_odom.twist.twist.linear.y
            body_vz = self.latest_odom.twist.twist.linear.z
            cos_yaw = math.cos(rov_yaw)
            sin_yaw = math.sin(rov_yaw)
            gain = self.payload_drop_initial_velocity_gain
            self.drop_velocity[0] = (body_vx * cos_yaw - body_vy * sin_yaw) * gain
            self.drop_velocity[1] = (body_vx * sin_yaw + body_vy * cos_yaw) * gain
            self.drop_velocity[2] = body_vz * gain
            self.drop_yaw_rate = self.latest_odom.twist.twist.angular.z * gain

    def update_dropping_payload(self, dt: float) -> None:
        if dt <= 0.0:
            return

        # z positive is upward. Effective gravity includes buoyancy; drag opposes motion.
        net_gravity = -9.81 * clamp(1.0 - self.payload_drop_buoyancy_ratio, 0.02, 1.0)
        drag_linear = max(0.0, self.payload_drop_linear_drag)
        drag_quad = max(0.0, self.payload_drop_quadratic_drag)

        for index in range(3):
            velocity = self.drop_velocity[index]
            acceleration = -drag_linear * velocity - drag_quad * velocity * abs(velocity)
            if index == 2:
                acceleration += net_gravity
            self.drop_velocity[index] += acceleration * dt

        terminal = max(0.03, self.payload_drop_terminal_speed)
        self.drop_velocity[2] = clamp(self.drop_velocity[2], -terminal, terminal * 0.35)

        self.payload_pose.position.x = clamp(
            self.payload_pose.position.x + self.drop_velocity[0] * dt,
            -4.60,
            4.60,
        )
        self.payload_pose.position.y = clamp(
            self.payload_pose.position.y + self.drop_velocity[1] * dt,
            -4.60,
            4.60,
        )
        self.payload_pose.position.z += self.drop_velocity[2] * dt

        self.drop_yaw_rate *= math.exp(-self.payload_drop_angular_damping * dt)
        self.drop_yaw += self.drop_yaw_rate * dt
        self.drop_pitch = clamp(
            -self.drop_velocity[0] * 0.55,
            -self.payload_drop_max_tilt,
            self.payload_drop_max_tilt,
        )
        self.drop_roll = clamp(
            self.drop_velocity[1] * 0.55,
            -self.payload_drop_max_tilt,
            self.payload_drop_max_tilt,
        )

        hook_guarded = self.resolve_payload_hook_collision(self.payload_pose)

        floor_contact = self.payload_pose.position.z <= self.payload_floor_z
        if floor_contact:
            self.payload_pose.position.z = self.payload_floor_z
            self.drop_velocity = [0.0, 0.0, 0.0]
            self.drop_yaw_rate = 0.0
            self.drop_roll = 0.0
            self.drop_pitch = 0.0
            self.dropping = False
            self.last_contact_source = "floor"
        elif hook_guarded:
            self.drop_velocity[0] *= 0.20
            self.drop_velocity[1] *= 0.20
            self.last_contact_source = "hook_guard"
        else:
            self.last_contact_source = "water_drag"

        qx, qy, qz, qw = quaternion_from_euler(
            self.drop_roll,
            self.drop_pitch,
            self.drop_yaw,
        )
        self.payload_pose.orientation.x = qx
        self.payload_pose.orientation.y = qy
        self.payload_pose.orientation.z = qz
        self.payload_pose.orientation.w = qw
        self.last_collision = hook_guarded or floor_contact
        self.set_model_pose(f"kki_payload_{self.model_payload_code}", self.payload_pose, 10)

    def resolve_payload_hook_collision(self, pose: Pose) -> bool:
        if self.hung_hook is not None:
            return False
        if self.attached and self.payload_hole_in_peg_entry_corridor(pose):
            return False

        payload_radius = 0.033
        payload_half_height = 0.055
        corrected = False
        for center_x, center_y, z_min, z_max, radius in HOOK_PAYLOAD_VERTICAL_GUARDS:
            if pose.position.z + payload_half_height < z_min:
                continue
            if pose.position.z - payload_half_height > z_max:
                continue
            corrected = (
                self.resolve_payload_point_guard(
                    pose,
                    center_x,
                    center_y,
                    payload_radius + radius + 0.006,
                )
                or corrected
            )

        for start, end, radius in HOOK_PAYLOAD_PEG_GUARDS:
            peg_z = start[2]
            if pose.position.z + payload_half_height < peg_z - radius:
                continue
            if pose.position.z - payload_half_height > peg_z + radius:
                continue
            corrected = (
                self.resolve_payload_peg_guard(
                    pose,
                    start,
                    end,
                    payload_radius + radius + 0.004,
                )
                or corrected
            )

        if corrected:
            self.last_collision = True
            self.last_contact_source = "hook_guard"
        return corrected

    def payload_hole_in_peg_entry_corridor(self, pose: Pose) -> bool:
        for index, (start, end, _radius) in enumerate(HOOK_PAYLOAD_PEG_GUARDS):
            hook_code = "ABCD"[index]
            if self.payload_can_pass_peg_for_code(hook_code, pose, record=True):
                return True
        return False

    def resolve_payload_peg_guard(
        self,
        pose: Pose,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        min_distance: float,
    ) -> bool:
        hook_code = self.hook_code_from_peg_segment(start, end)
        if hook_code is not None:
            if self.payload_has_active_peg_entry(hook_code):
                if self.resolve_payload_entered_peg_constraint(pose, start, end):
                    self.last_release_block_reason = "peg_lateral_constraint"
                    return True
            if self.payload_can_pass_peg_for_code(
                hook_code,
                pose,
                record=self.attached,
            ):
                return False

        radial_error, axial_error = self.payload_hole_error_to_segment(pose, start, end)
        xy_error, xy_axial_error, _xy_t = self.payload_hole_xy_error_and_t_to_segment(
            pose,
            start,
            end,
        )
        hole_clear = max(self.hook_peg_pass_window, self.hook_snap_hole_tolerance)
        axial_clear = max(self.hook_snap_axial_tolerance, 0.020)
        if xy_error <= hole_clear and xy_axial_error <= axial_clear:
            self.last_release_block_reason = "side_entry_blocked"
            return self.resolve_payload_side_entry_guard(pose, start, end, min_distance)
        if radial_error <= hole_clear and axial_error <= axial_clear:
            self.last_release_block_reason = "side_entry_blocked"
            return self.resolve_payload_side_entry_guard(pose, start, end, min_distance)

        return self.resolve_payload_segment_guard(pose, start, end, min_distance)

    def resolve_payload_entered_peg_constraint(
        self,
        pose: Pose,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> bool:
        hole_x, hole_y, hole_z = self.payload_hole_position_from_pose(pose)
        axis_x = end[0] - start[0]
        axis_y = end[1] - start[1]
        axis_z = end[2] - start[2]
        length_sq = axis_x * axis_x + axis_y * axis_y + axis_z * axis_z
        if length_sq < 1e-9:
            return False

        t_hole = (
            (hole_x - start[0]) * axis_x
            + (hole_y - start[1]) * axis_y
            + (hole_z - start[2]) * axis_z
        ) / length_sq
        t_clamped = clamp(t_hole, 0.0, 1.0)
        axial_error = 0.0
        if t_hole < 0.0:
            axial_error = math.sqrt(length_sq) * -t_hole
        elif t_hole > 1.0:
            axial_error = math.sqrt(length_sq) * (t_hole - 1.0)
        if axial_error > max(self.hook_latch_axial_release_tolerance, 0.080):
            return False

        target_x = start[0] + axis_x * t_clamped
        target_y = start[1] + axis_y * t_clamped
        target_z = start[2] + axis_z * t_clamped
        dx = target_x - hole_x
        dy = target_y - hole_y
        dz = target_z - hole_z
        lateral_error = math.sqrt(dx * dx + dy * dy + dz * dz)
        if lateral_error < 1e-5:
            return False

        pose.position.x += dx
        pose.position.y += dy
        pose.position.z += dz
        return True

    @staticmethod
    def hook_code_from_peg_segment(
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> Optional[str]:
        for index, (known_start, known_end, _radius) in enumerate(HOOK_PAYLOAD_PEG_GUARDS):
            if start == known_start and end == known_end:
                return "ABCD"[index]
        return None

    def resolve_payload_side_entry_guard(
        self,
        pose: Pose,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        min_distance: float,
    ) -> bool:
        hole_x, hole_y, _hole_z = self.payload_hole_position_from_pose(pose)
        axis_x = end[0] - start[0]
        axis_y = end[1] - start[1]
        length_sq = axis_x * axis_x + axis_y * axis_y
        if length_sq < 1e-9:
            return self.resolve_payload_point_guard(pose, start[0], start[1], min_distance)

        t = ((hole_x - start[0]) * axis_x + (hole_y - start[1]) * axis_y) / length_sq
        t = clamp(t, 0.0, 1.0)
        nearest_x = start[0] + axis_x * t
        nearest_y = start[1] + axis_y * t
        dx = hole_x - nearest_x
        dy = hole_y - nearest_y
        distance = math.hypot(dx, dy)

        if abs(axis_x) >= abs(axis_y):
            normal_x = 0.0
            normal_y = 1.0 if dy >= 0.0 else -1.0
            if abs(dy) < 1e-5:
                normal_y = self.side_entry_normal_sign(pose.position.y, nearest_y)
        else:
            normal_x = 1.0 if dx >= 0.0 else -1.0
            normal_y = 0.0
            if abs(dx) < 1e-5:
                normal_x = self.side_entry_normal_sign(pose.position.x, nearest_x)

        target_distance = max(min_distance, self.hook_latch_release_tolerance + 0.010)
        if distance >= target_distance:
            return False

        correction = target_distance - distance
        pose.position.x += normal_x * correction
        pose.position.y += normal_y * correction
        return True

    @staticmethod
    def side_entry_normal_sign(value: float, reference: float) -> float:
        delta = value - reference
        if abs(delta) > 1e-5:
            return 1.0 if delta > 0.0 else -1.0
        return 1.0

    def payload_hole_error_to_segment(
        self,
        pose: Pose,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> tuple[float, float]:
        radial_error, axial_error, _t_hole = self.payload_hole_error_and_t_to_segment(
            pose,
            start,
            end,
        )
        return radial_error, axial_error

    def payload_hole_xy_error_and_t_to_segment(
        self,
        pose: Pose,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        hole_x, hole_y, _hole_z = self.payload_hole_position_from_pose(pose)
        axis_x = end[0] - start[0]
        axis_y = end[1] - start[1]
        length_sq = axis_x * axis_x + axis_y * axis_y
        if length_sq < 1e-9:
            return 999.0, 999.0, -999.0

        t_hole = ((hole_x - start[0]) * axis_x + (hole_y - start[1]) * axis_y) / length_sq
        t_hole_clamped = clamp(t_hole, 0.0, 1.0)
        nearest_hole_x = start[0] + axis_x * t_hole_clamped
        nearest_hole_y = start[1] + axis_y * t_hole_clamped
        radial_error = math.hypot(hole_x - nearest_hole_x, hole_y - nearest_hole_y)
        axial_error = 0.0
        if t_hole < 0.0:
            axial_error = math.sqrt(length_sq) * -t_hole
        elif t_hole > 1.0:
            axial_error = math.sqrt(length_sq) * (t_hole - 1.0)
        return radial_error, axial_error, t_hole

    def payload_hole_error_and_t_to_segment(
        self,
        pose: Pose,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        hole_x, hole_y, hole_z = self.payload_hole_position_from_pose(pose)
        axis_x = end[0] - start[0]
        axis_y = end[1] - start[1]
        axis_z = end[2] - start[2]
        length_sq = axis_x * axis_x + axis_y * axis_y + axis_z * axis_z
        if length_sq < 1e-9:
            return 999.0, 999.0, -999.0

        t_hole = (
            (hole_x - start[0]) * axis_x
            + (hole_y - start[1]) * axis_y
            + (hole_z - start[2]) * axis_z
        ) / length_sq
        t_hole_clamped = clamp(t_hole, 0.0, 1.0)
        nearest_hole_x = start[0] + axis_x * t_hole_clamped
        nearest_hole_y = start[1] + axis_y * t_hole_clamped
        nearest_hole_z = start[2] + axis_z * t_hole_clamped
        radial_error = math.sqrt(
            (hole_x - nearest_hole_x) ** 2
            + (hole_y - nearest_hole_y) ** 2
            + (hole_z - nearest_hole_z) ** 2
        )
        axial_error = 0.0
        if t_hole < 0.0:
            axial_error = math.sqrt(length_sq) * -t_hole
        elif t_hole > 1.0:
            axial_error = math.sqrt(length_sq) * (t_hole - 1.0)
        return radial_error, axial_error, t_hole

    def payload_can_pass_peg_for_code(
        self,
        hook_code: str,
        pose: Pose,
        record: bool,
    ) -> bool:
        hook_index = "ABCD".find(hook_code)
        if hook_index < 0:
            return False

        start, end, _radius = HOOK_PAYLOAD_PEG_GUARDS[hook_index]
        radial_error, axial_error, t_hole = self.payload_hole_error_and_t_to_segment(
            pose,
            start,
            end,
        )
        radial_clear = max(self.hook_peg_pass_window, self.hook_snap_hole_tolerance)
        axial_clear = max(self.hook_snap_axial_tolerance, 0.026)
        now = time.monotonic()
        lateral_error, axis_t = self.payload_hole_lateral_error_and_t_to_axis(
            pose,
            start,
            end,
        )
        tip_pre_entry = self.payload_hole_in_tip_pre_entry_zone(
            lateral_error,
            axis_t,
            start,
            end,
            radial_clear,
        )
        tip_entry_motion = self.payload_tip_entry_motion_is_valid(
            hook_code,
            lateral_error,
            axis_t,
            start,
            end,
            radial_clear,
            now,
        )
        if record and tip_pre_entry and tip_entry_motion:
            self.peg_tip_entry_code = hook_code
            self.peg_tip_entry_time = now

        if radial_error > radial_clear or axial_error > axial_clear:
            if record and not tip_pre_entry and not self.payload_has_active_peg_entry(hook_code, now):
                self.clear_hook_entry_if_far(hook_code, radial_error, axial_error)
            if record:
                self.update_peg_tip_motion_state(hook_code, lateral_error, axis_t, now)
            return False

        has_tip_pre_entry = (
            self.peg_tip_entry_code == hook_code
            and self.peg_tip_entry_time is not None
            and now - self.peg_tip_entry_time <= max(0.0, self.hook_entry_memory_s)
        )
        entered_from_tip = (
            (has_tip_pre_entry or tip_entry_motion)
            and self.hook_entry_tip_min_t <= t_hole <= 1.0
        )
        already_entered = (
            self.peg_entry_code == hook_code
            and self.peg_entry_time is not None
            and now - self.peg_entry_time <= max(0.0, self.hook_entry_memory_s)
        )

        if record and entered_from_tip:
            self.peg_entry_code = hook_code
            self.peg_entry_time = now
            already_entered = True

        if record:
            self.update_peg_tip_motion_state(hook_code, lateral_error, axis_t, now)

        return entered_from_tip or already_entered

    def payload_hole_in_tip_pre_entry_zone(
        self,
        lateral_error: float,
        t_hole: float,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        radial_clear: float,
    ) -> bool:
        if t_hole <= 1.0:
            return False

        segment_length = math.dist(start, end)
        axial_past_tip = (t_hole - 1.0) * segment_length
        tip_window = max(0.035, self.hook_snap_axial_tolerance + 0.020)
        return lateral_error <= radial_clear and axial_past_tip <= tip_window

    def payload_tip_entry_motion_is_valid(
        self,
        hook_code: str,
        lateral_error: float,
        t_hole: float,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        radial_clear: float,
        now: float,
    ) -> bool:
        previous = self.peg_tip_motion_state.get(hook_code)
        if previous is None:
            return False

        previous_t, previous_lateral_error, previous_time = previous
        if now - previous_time > max(0.20, self.hook_entry_memory_s):
            return False

        segment_length = math.dist(start, end)
        inward_progress = (previous_t - t_hole) * segment_length
        axial_past_tip = max(0.0, (t_hole - 1.0) * segment_length)
        previous_axial_past_tip = max(0.0, (previous_t - 1.0) * segment_length)
        tip_window = max(0.035, self.hook_snap_axial_tolerance + 0.020)
        return (
            previous_t > 1.0
            and inward_progress >= 0.0015
            and lateral_error <= radial_clear
            and previous_lateral_error <= radial_clear
            and axial_past_tip <= tip_window
            and previous_axial_past_tip <= tip_window
        )

    def update_peg_tip_motion_state(
        self,
        hook_code: str,
        lateral_error: float,
        t_hole: float,
        now: float,
    ) -> None:
        self.peg_tip_motion_state[hook_code] = (t_hole, lateral_error, now)

    def payload_has_active_peg_entry(
        self,
        hook_code: str,
        now: Optional[float] = None,
    ) -> bool:
        if self.peg_entry_code != hook_code or self.peg_entry_time is None:
            return False
        if now is None:
            now = time.monotonic()
        return now - self.peg_entry_time <= max(0.0, self.hook_entry_memory_s)

    def payload_hole_lateral_error_and_t_to_axis(
        self,
        pose: Pose,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> tuple[float, float]:
        hole_x, hole_y, hole_z = self.payload_hole_position_from_pose(pose)
        axis_x = end[0] - start[0]
        axis_y = end[1] - start[1]
        axis_z = end[2] - start[2]
        length_sq = axis_x * axis_x + axis_y * axis_y + axis_z * axis_z
        if length_sq < 1e-9:
            return 999.0, -999.0

        t_hole = (
            (hole_x - start[0]) * axis_x
            + (hole_y - start[1]) * axis_y
            + (hole_z - start[2]) * axis_z
        ) / length_sq
        nearest_x = start[0] + axis_x * t_hole
        nearest_y = start[1] + axis_y * t_hole
        nearest_z = start[2] + axis_z * t_hole
        lateral_error = math.sqrt(
            (hole_x - nearest_x) ** 2
            + (hole_y - nearest_y) ** 2
            + (hole_z - nearest_z) ** 2
        )
        return lateral_error, t_hole

    def clear_hook_entry_if_far(
        self,
        hook_code: str,
        radial_error: float,
        axial_error: float,
    ) -> None:
        entry_matches = self.peg_entry_code == hook_code
        tip_entry_matches = self.peg_tip_entry_code == hook_code
        if not entry_matches and not tip_entry_matches:
            return
        radial_limit = max(self.hook_latch_release_tolerance, self.hook_peg_pass_window) + 0.012
        axial_limit = max(self.hook_latch_axial_release_tolerance, self.hook_snap_axial_tolerance) + 0.030
        if radial_error > radial_limit or axial_error > axial_limit:
            if entry_matches:
                self.peg_entry_code = None
                self.peg_entry_time = None
            if tip_entry_matches:
                self.peg_tip_entry_code = None
                self.peg_tip_entry_time = None
                self.peg_tip_motion_state.pop(hook_code, None)
            if self.recent_hook_latch_code == hook_code:
                self.recent_hook_latch_code = None
                self.recent_hook_latch_point = None
                self.recent_hook_latch_time = None

    def resolve_payload_segment_guard(
        self,
        pose: Pose,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        min_distance: float,
    ) -> bool:
        ax, ay = start[0], start[1]
        bx, by = end[0], end[1]
        seg_x = bx - ax
        seg_y = by - ay
        seg_len_sq = seg_x * seg_x + seg_y * seg_y
        if seg_len_sq < 1e-9:
            return self.resolve_payload_point_guard(pose, ax, ay, min_distance)

        t = ((pose.position.x - ax) * seg_x + (pose.position.y - ay) * seg_y) / seg_len_sq
        t = clamp(t, 0.0, 1.0)
        nearest_x = ax + seg_x * t
        nearest_y = ay + seg_y * t
        dx = pose.position.x - nearest_x
        dy = pose.position.y - nearest_y
        distance = math.hypot(dx, dy)
        if distance >= min_distance:
            return False
        if distance < 1e-6:
            dx, dy = self.outward_from_hook(nearest_x, nearest_y)
            pose.position.x += dx * min_distance
            pose.position.y += dy * min_distance
            return True

        scale = (min_distance - distance) / max(distance, 1e-6)
        pose.position.x += dx * scale
        pose.position.y += dy * scale
        return True

    def resolve_payload_point_guard(
        self,
        pose: Pose,
        center_x: float,
        center_y: float,
        min_distance: float,
    ) -> bool:
        dx = pose.position.x - center_x
        dy = pose.position.y - center_y
        distance = math.hypot(dx, dy)
        if distance >= min_distance:
            return False

        if distance < 1e-6:
            dx, dy = self.outward_from_hook(center_x, center_y)
            pose.position.x += dx * min_distance
            pose.position.y += dy * min_distance
            return True

        scale = (min_distance - distance) / max(distance, 1e-6)
        pose.position.x += dx * scale
        pose.position.y += dy * scale
        return True

    def release_payload_to_hook_if_close(self) -> Optional[str]:
        if (
            not self.hang_on_release_enabled
            or not self.hanging_constraint_enabled
            or self.latest_odom is None
        ):
            return None

        hook_code = self.select_hook_code_for_release()
        if hook_code is None:
            return None

        if not self.update_hook_alignment_for_code(hook_code):
            if not self.try_hook_alignment_from_current_grip_target(hook_code):
                if not self.use_recent_hook_latch(hook_code):
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
        self.hook_latch_point = self.hook_latch_point_candidate
        self.recent_hook_latch_code = None
        self.recent_hook_latch_point = None
        self.recent_hook_latch_time = None

        pose = self.hanging_payload_pose(hook_code, self.hang_angle)
        self.payload_pose = pose
        self.last_contact_source = f"hook_{hook_code}"
        self.set_model_pose(f"kki_payload_{self.model_payload_code}", pose, 50)
        return hook_code

    def select_hook_code_for_release(self) -> Optional[str]:
        if self.payload_code in HOOK_PEG_POSES:
            return self.payload_code
        if self.recent_hook_latch_code in HOOK_PEG_POSES:
            return self.recent_hook_latch_code

        best_code = None
        best_error = 999.0
        for code in "ABCD":
            if not self.payload_is_near_hook_release_window(code):
                continue
            radial_error, axial_error = self.payload_error_to_hook(code)
            score = radial_error + axial_error
            if score < best_error:
                best_code = code
                best_error = score
        return best_code

    def try_hook_alignment_from_current_grip_target(self, hook_code: str) -> bool:
        if self.latest_odom is None:
            return False
        if not self.payload_is_near_hook_release_window(hook_code):
            return False

        current_pose = self.payload_pose
        target_pose = self.held_payload_pose(self.latest_odom.pose.pose)
        self.payload_pose = target_pose
        if self.update_hook_alignment_for_code(hook_code):
            return True

        self.payload_pose = current_pose
        return False

    def payload_is_near_hook_release_window(self, hook_code: str) -> bool:
        radial_error, axial_error = self.payload_error_to_hook(hook_code)
        return (
            radial_error <= max(self.hook_peg_pass_window, self.hook_latch_release_tolerance)
            and axial_error <= max(self.hook_snap_axial_tolerance, self.hook_latch_axial_release_tolerance)
            and self.payload_can_pass_peg_for_code(
                hook_code,
                self.payload_pose,
                record=self.attached,
            )
        )

    def payload_error_to_hook(self, hook_code: str) -> tuple[float, float]:
        hook_index = "ABCD".find(hook_code)
        if hook_index < 0:
            return 999.0, 999.0
        start, end, _radius = HOOK_PAYLOAD_PEG_GUARDS[hook_index]
        return self.payload_hole_error_to_segment(self.payload_pose, start, end)

    def use_recent_hook_latch(self, hook_code: str) -> bool:
        if (
            self.recent_hook_latch_code != hook_code
            or self.recent_hook_latch_point is None
            or self.recent_hook_latch_time is None
        ):
            return False
        if time.monotonic() - self.recent_hook_latch_time > max(0.0, self.hook_latch_memory_s):
            return False

        self.hook_latch_point_candidate = self.recent_hook_latch_point
        self.last_hook_aligned = True
        self.last_release_block_reason = "ready_latched"
        return True

    def update_hook_alignment_status(self) -> bool:
        self.last_hook_alignment = (999.0, 999.0, 999.0, 999.0)
        self.last_hook_aligned = False
        self.hook_latch_point_candidate = None

        hook_codes = [self.payload_code] if self.payload_code in HOOK_PEG_POSES else list("ABCD")
        if not hook_codes:
            self.last_release_block_reason = "unknown_qr"
            return False
        if self.latest_odom is None:
            self.last_release_block_reason = "no_odom"
            return False

        best_status = None
        best_alignment = None
        best_reason = "unknown_qr" if self.payload_code not in HOOK_PEG_POSES else "hole_not_on_peg"
        for hook_code in hook_codes:
            if self.update_hook_alignment_for_code(hook_code):
                return True
            hook_radial, hook_axial, hook_rov, hook_yaw = self.last_hook_alignment
            score = hook_radial + hook_axial
            if best_status is None or score < best_status:
                best_status = score
                best_alignment = (hook_radial, hook_axial, hook_rov, hook_yaw)
                best_reason = self.last_release_block_reason

        if best_alignment is not None:
            self.last_hook_alignment = best_alignment
        self.last_release_block_reason = best_reason
        return False

    def update_hook_alignment_for_code(self, hook_code: str) -> bool:
        if hook_code not in HOOK_PEG_POSES:
            self.last_release_block_reason = "unknown_qr"
            return False
        if self.latest_odom is None:
            self.last_release_block_reason = "no_odom"
            return False

        peg_x, peg_y, peg_z, hook_yaw = HOOK_PEG_POSES[hook_code]
        axis = self.hook_axis_unit(hook_code)
        approach = HOOK_APPROACH_POSES[hook_code]
        rov_pose = self.latest_odom.pose.pose
        rov_position = rov_pose.position
        rov_yaw = yaw_from_quaternion(
            rov_pose.orientation.x,
            rov_pose.orientation.y,
            rov_pose.orientation.z,
            rov_pose.orientation.w,
        )
        hole_x, hole_y, hole_z = self.payload_hole_position()
        hole_dx = hole_x - peg_x
        hole_dy = hole_y - peg_y
        hole_dz = hole_z - peg_z
        axial_offset = hole_dx * axis[0] + hole_dy * axis[1] + hole_dz * axis[2]
        half_peg = max(0.01, self.hook_peg_length * 0.5)
        clamped_axial = clamp(axial_offset, -half_peg, half_peg)
        nearest_x = peg_x + axis[0] * clamped_axial
        nearest_y = peg_y + axis[1] * clamped_axial
        nearest_z = peg_z + axis[2] * clamped_axial
        radial_error = math.sqrt(
            (hole_x - nearest_x) ** 2
            + (hole_y - nearest_y) ** 2
            + (hole_z - nearest_z) ** 2
        )
        axial_error = max(0.0, abs(axial_offset) - half_peg)
        rov_distance = math.sqrt(
            (rov_position.x - approach[0]) ** 2
            + (rov_position.y - approach[1]) ** 2
            + (rov_position.z - approach[2]) ** 2
        )
        yaw_error = abs(normalize_angle(hook_yaw - rov_yaw))
        self.last_hook_alignment = (radial_error, axial_error, rov_distance, yaw_error)
        strict_radial_tolerance = min(self.hook_snap_hole_tolerance, self.hook_snap_tolerance)
        strict_ready = (
            radial_error <= strict_radial_tolerance
            and axial_error <= self.hook_snap_axial_tolerance
        )
        release_radial_tolerance = max(
            self.hook_snap_hole_tolerance,
            self.hook_peg_pass_window,
            self.hook_latch_release_tolerance,
        )
        release_axial_tolerance = max(
            self.hook_snap_axial_tolerance,
            self.hook_latch_axial_release_tolerance,
        )
        entry_ok = self.payload_can_pass_peg_for_code(
            hook_code,
            self.payload_pose,
            record=self.attached,
        )
        if (
            self.attached
            and radial_error <= release_radial_tolerance
            and axial_error <= release_axial_tolerance
            and not entry_ok
        ):
            self.last_release_block_reason = "side_entry_blocked"
            return False
        if (
            self.attached
            and radial_error <= release_radial_tolerance
            and axial_error <= release_axial_tolerance
            and entry_ok
        ):
            latch_point = (nearest_x, nearest_y, nearest_z)
            self.recent_hook_latch_code = hook_code
            self.recent_hook_latch_point = latch_point
            self.recent_hook_latch_time = time.monotonic()
            self.hook_latch_point_candidate = latch_point
            self.last_hook_aligned = True
            self.last_release_block_reason = "ready" if strict_ready else "ready_latched"
            return True

        if radial_error > strict_radial_tolerance:
            self.last_release_block_reason = "hole_not_on_peg"
            return False
        if axial_error > self.hook_snap_axial_tolerance:
            self.last_release_block_reason = "peg_depth_bad"
            return False
        self.last_hook_aligned = True
        self.last_release_block_reason = "ready"
        self.hook_latch_point_candidate = (nearest_x, nearest_y, nearest_z)
        return True

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
        self.set_model_pose(f"kki_payload_{self.model_payload_code}", self.payload_pose, 10)

    def hanging_payload_pose(self, hook_code: str, angle: float) -> Pose:
        pivot_x, pivot_y, pivot_z, yaw = self.hook_pivot_pose(hook_code)
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
        pivot_x, pivot_y, pivot_z, yaw = self.hook_pivot_pose(hook_code)
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

    def payload_hole_position(self) -> tuple[float, float, float]:
        return self.payload_hole_position_from_pose(self.payload_pose)

    def payload_hole_position_from_pose(self, pose: Pose) -> tuple[float, float, float]:
        offset_x, offset_y, offset_z = rotate_vector_by_quaternion(
            (0.0, 0.0, self.hanging_hole_offset_z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        )
        return (
            pose.position.x + offset_x,
            pose.position.y + offset_y,
            pose.position.z + offset_z,
        )

    def hook_pivot_pose(self, hook_code: str) -> tuple[float, float, float, float]:
        peg_x, peg_y, peg_z, yaw = HOOK_PEG_POSES[hook_code]
        if self.hung_hook == hook_code and self.hook_latch_point is not None:
            peg_x, peg_y, peg_z = self.hook_latch_point
        return peg_x, peg_y, peg_z, yaw

    @staticmethod
    def hook_axis_unit(hook_code: str) -> tuple[float, float, float]:
        if hook_code in {"A", "B"}:
            return (1.0, 0.0, 0.0)
        return (0.0, 1.0, 0.0)

    @staticmethod
    def outward_from_hook(center_x: float, center_y: float) -> tuple[float, float]:
        dx = -center_x
        dy = -center_y
        if abs(dx) + abs(dy) < 1e-6:
            return 1.0, 0.0
        distance = math.hypot(dx, dy)
        return dx / distance, dy / distance

    @staticmethod
    def distance_between_positions(a, b) -> float:
        return math.sqrt(
            (a.x - b.x) ** 2
            + (a.y - b.y) ** 2
            + (a.z - b.z) ** 2
        )

    def update_jaw_visuals(self) -> None:
        visual_gripper_position = self.effective_gripper_position()
        jaw_angle = self.jaw_open_angle + (
            self.jaw_closed_angle - self.jaw_open_angle
        ) * visual_gripper_position

        if self.gripper_actuation_mode == "joint":
            self.publish_joint_position(self.left_joint_pub, jaw_angle)
            self.publish_joint_position(self.right_joint_pub, -jaw_angle)
            return

        if self.latest_odom is None:
            return

        rov_pose = self.latest_odom.pose.pose
        left_pose = self.local_pose_to_world(
            rov_pose,
            local_x=self.jaw_pivot_x,
            local_y=self.jaw_pivot_y,
            local_z=self.capture_vertical_offset,
            local_yaw=jaw_angle,
        )
        right_pose = self.local_pose_to_world(
            rov_pose,
            local_x=self.jaw_pivot_x,
            local_y=-self.jaw_pivot_y,
            local_z=self.capture_vertical_offset,
            local_yaw=-jaw_angle,
        )
        self.set_model_pose("gamantaray_rov_left_gripper_jaw", left_pose, 5)
        self.set_model_pose("gamantaray_rov_right_gripper_jaw", right_pose, 5)

    @staticmethod
    def publish_joint_position(publisher, position: float) -> None:
        if publisher is None:
            return
        msg = GzDouble()
        msg.data = float(position)
        publisher.publish(msg)

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
        self.set_model_pose(f"kki_payload_{self.model_payload_code}", self.payload_pose, 10)

    def jaw_gap_from_position(self, position: float) -> float:
        return self.open_gap + (self.closed_gap - self.open_gap) * clamp(position, 0.0, 1.0)

    def raw_jaw_gap(self) -> float:
        return self.jaw_gap_from_position(self.gripper_position)

    def max_gripper_position_on_payload(self) -> float:
        desired_gap = self.payload_width + max(
            0.003,
            self.jaw_visual_stop_clearance,
        )
        denom = self.closed_gap - self.open_gap
        if abs(denom) < 1e-6:
            return 1.0
        return clamp((desired_gap - self.open_gap) / denom, 0.0, 1.0)

    def payload_between_jaws(self) -> bool:
        if self.attached:
            return True
        if self.latest_odom is None or self.dropping or self.hung_hook is not None:
            return False

        local_x, local_y, local_z = self.payload_in_body_frame()
        mouth_min_x = self.jaw_pivot_x - 0.035
        mouth_max_x = self.capture_forward_offset + 0.105
        raw_gap = self.raw_jaw_gap()
        lateral_limit = raw_gap * 0.5 + self.payload_width * 0.5 + 0.025
        return (
            mouth_min_x <= local_x <= mouth_max_x
            and abs(local_y) <= lateral_limit
            and abs(local_z - self.capture_vertical_offset) <= 0.140
        )

    def payload_in_claw_stop_zone(self) -> bool:
        if self.attached:
            return True
        if self.latest_odom is None or self.dropping or self.hung_hook is not None:
            return False

        local_x, local_y, local_z = self.payload_in_body_frame()
        payload_half_width = self.payload_width * 0.5
        jaw_half_gap = self.raw_jaw_gap() * 0.5
        x_min = self.jaw_pivot_x - 0.035
        x_max = self.capture_forward_offset + 0.120
        lateral_limit = (
            max(jaw_half_gap, self.open_gap * 0.5)
            + payload_half_width
            + max(0.008, self.grip_clamp_margin)
        )
        return (
            x_min <= local_x <= x_max
            and abs(local_y) <= lateral_limit
            and abs(local_z - self.capture_vertical_offset) <= 0.090
        )

    def safe_visual_gripper_position(self) -> float:
        limit = min(self.gripper_position, self.max_gripper_position_on_payload())
        if self.latest_odom is None or self.dropping or self.hung_hook is not None:
            return self.gripper_position
        if not self.payload_in_claw_stop_zone():
            return self.gripper_position

        local_x, local_y, local_z = self.payload_in_body_frame()
        allowed_penetration = max(0.0006, self.jaw_visual_stop_clearance * 0.12)
        low = 0.0
        high = limit
        for _ in range(12):
            mid = (low + high) * 0.5
            jaw_angle = self.jaw_open_angle + (
                self.jaw_closed_angle - self.jaw_open_angle
            ) * mid
            penetration = 0.0
            for side in ("left", "right"):
                for component in ("inner_pad", "outer_finger", "front_tip", "rear_link"):
                    contact, component_penetration, _dx, _dy, _center_y = (
                        self.jaw_component_contact(
                            side,
                            local_x,
                            local_y,
                            local_z,
                            jaw_angle,
                            0.0,
                            component,
                        )
                    )
                    if contact:
                        penetration = max(penetration, component_penetration)
            if penetration > allowed_penetration:
                high = mid
            else:
                low = mid
        return min(limit, low)

    def effective_gripper_position(self) -> float:
        if self.payload_contact_model == "strict":
            return self.safe_visual_gripper_position()
        if self.payload_between_jaws():
            return min(self.gripper_position, self.max_gripper_position_on_payload())
        return self.gripper_position

    def current_jaw_gap(self) -> float:
        return self.jaw_gap_from_position(self.effective_gripper_position())

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
        pose.position.z = max(self.payload_floor_z, pose.position.z)
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
