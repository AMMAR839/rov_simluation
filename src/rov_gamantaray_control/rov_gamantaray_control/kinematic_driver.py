import math
import time

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.transport13 import Node as GzNode
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def quaternion_from_yaw(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
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


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def approach(current: float, target: float, dt: float, time_constant: float) -> float:
    if time_constant <= 1e-6:
        return target
    alpha = 1.0 - math.exp(-dt / time_constant)
    return current + (target - current) * alpha


class KinematicDriver(Node):
    """Move the static Gazebo ROV from the allocator's thruster output."""

    def __init__(self) -> None:
        super().__init__("kinematic_driver")
        self.declare_parameter("world_name", "kki_rov_pool")
        self.declare_parameter("model_name", "gamantaray_rov")
        self.declare_parameter("max_horizontal_thrust_n", 22.0)
        self.declare_parameter("max_vertical_thrust_n", 18.0)
        self.declare_parameter("yaw_scale", 0.75)
        self.declare_parameter("max_xy_speed_mps", 0.75)
        self.declare_parameter("max_z_speed_mps", 0.35)
        self.declare_parameter("max_yaw_rate_rps", 0.75)
        self.declare_parameter("linear_response_s", 0.42)
        self.declare_parameter("vertical_response_s", 0.55)
        self.declare_parameter("yaw_response_s", 0.48)
        self.declare_parameter("attitude_response_s", 0.70)
        self.declare_parameter("max_visual_tilt_rad", 0.10)
        self.declare_parameter("prop_visuals_enabled", True)
        self.declare_parameter("prop_layout", "legacy_bluerov")
        self.declare_parameter("prop_spin_gain_rad_s", 60.0)
        self.declare_parameter("initial_x", -3.2)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", -0.28)
        self.declare_parameter("bottom_limit_z", -0.72)
        self.declare_parameter("surface_limit_z", -0.08)

        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.max_horizontal = float(self.get_parameter("max_horizontal_thrust_n").value)
        self.max_vertical = float(self.get_parameter("max_vertical_thrust_n").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.max_xy_speed = float(self.get_parameter("max_xy_speed_mps").value)
        self.max_z_speed = float(self.get_parameter("max_z_speed_mps").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self.linear_response_s = float(self.get_parameter("linear_response_s").value)
        self.vertical_response_s = float(self.get_parameter("vertical_response_s").value)
        self.yaw_response_s = float(self.get_parameter("yaw_response_s").value)
        self.attitude_response_s = float(self.get_parameter("attitude_response_s").value)
        self.max_visual_tilt = float(self.get_parameter("max_visual_tilt_rad").value)
        self.prop_visuals_enabled = bool(self.get_parameter("prop_visuals_enabled").value)
        self.prop_layout = str(self.get_parameter("prop_layout").value)
        self.prop_spin_gain = float(self.get_parameter("prop_spin_gain_rad_s").value)

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.z = float(self.get_parameter("initial_z").value)
        self.bottom_limit_z = float(self.get_parameter("bottom_limit_z").value)
        self.surface_limit_z = float(self.get_parameter("surface_limit_z").value)
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.body_vx = 0.0
        self.body_vy = 0.0
        self.body_vz = 0.0
        self.yaw_rate = 0.0
        self.last_thrusters = [0.0] * 6
        self.last_update = time.monotonic()

        self.odom_pub = self.create_publisher(Odometry, "/model/gamantaray_rov/odometry", 10)
        self.create_subscription(
            Float64MultiArray, "/rov/thruster_status", self.thruster_callback, 10
        )
        self.gz_node = GzNode()
        self.pose_service = f"/world/{self.world_name}/set_pose"
        self.prop_spin_angles = [0.0] * 6
        self.prop_specs = self.make_prop_specs(self.prop_layout)
        self.create_timer(0.05, self.update)

    def make_prop_specs(self, layout: str) -> list[dict]:
        if layout == "github_blue":
            return [
                {
                    "name": "gamantaray_rov_thruster1_prop_visual",
                    "offset": (0.1071, -0.0704, 0.000),
                    "rpy": (-1.571, 1.571, -0.785),
                    "direction": 1.0,
                    "limit": self.max_horizontal,
                    "spin_axis": "z",
                },
                {
                    "name": "gamantaray_rov_thruster2_prop_visual",
                    "offset": (0.1071, 0.0704, 0.000),
                    "rpy": (-1.571, 1.571, -2.356),
                    "direction": 1.0,
                    "limit": self.max_horizontal,
                    "spin_axis": "z",
                },
                {
                    "name": "gamantaray_rov_thruster3_prop_visual",
                    "offset": (-0.1148, -0.0704, 0.000),
                    "rpy": (-1.571, 1.571, 0.785),
                    "direction": -1.0,
                    "limit": self.max_horizontal,
                    "spin_axis": "z",
                },
                {
                    "name": "gamantaray_rov_thruster4_prop_visual",
                    "offset": (-0.1148, 0.0704, 0.000),
                    "rpy": (-1.571, 1.571, 2.356),
                    "direction": -1.0,
                    "limit": self.max_horizontal,
                    "spin_axis": "z",
                },
                {
                    "name": "gamantaray_rov_thruster5_prop_visual",
                    "offset": (0.000, -0.0834, 0.0589),
                    "rpy": (0.0, 0.0, 0.0),
                    "direction": 1.0,
                    "limit": self.max_vertical,
                    "spin_axis": "z",
                },
                {
                    "name": "gamantaray_rov_thruster6_prop_visual",
                    "offset": (0.000, 0.0834, 0.0589),
                    "rpy": (0.0, 0.0, 0.0),
                    "direction": -1.0,
                    "limit": self.max_vertical,
                    "spin_axis": "z",
                },
            ]

        return [
            {
                "name": "gamantaray_rov_thruster1_prop_visual",
                "offset": (0.1111, -0.0820, -0.0595),
                "rpy": (0.0, 0.0, 0.7853981634),
                "direction": 1.0,
                "limit": self.max_horizontal,
                "spin_axis": "x",
            },
            {
                "name": "gamantaray_rov_thruster2_prop_visual",
                "offset": (0.1111, 0.0820, -0.0595),
                "rpy": (0.0, 0.0, -0.7853981634),
                "direction": -1.0,
                "limit": self.max_horizontal,
                "spin_axis": "x",
            },
            {
                "name": "gamantaray_rov_thruster3_prop_visual",
                "offset": (-0.1210, -0.0820, -0.0595),
                "rpy": (0.0, 0.0, 2.3561944902),
                "direction": -1.0,
                "limit": self.max_horizontal,
                "spin_axis": "x",
            },
            {
                "name": "gamantaray_rov_thruster4_prop_visual",
                "offset": (-0.1210, 0.0820, -0.0595),
                "rpy": (0.0, 0.0, -2.3561944902),
                "direction": 1.0,
                "limit": self.max_horizontal,
                "spin_axis": "x",
            },
            {
                "name": "gamantaray_rov_thruster5_prop_visual",
                "offset": (0.0021, -0.0906, -0.0041),
                "rpy": (0.0, -1.5707963268, 0.0),
                "direction": 1.0,
                "limit": self.max_vertical,
                "spin_axis": "x",
            },
            {
                "name": "gamantaray_rov_thruster6_prop_visual",
                "offset": (0.0021, 0.0906, -0.0041),
                "rpy": (0.0, -1.5707963268, 0.0),
                "direction": -1.0,
                "limit": self.max_vertical,
                "spin_axis": "x",
            },
        ]

    def thruster_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 6:
            self.last_thrusters = [float(v) for v in msg.data[:6]]

    def update(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_update))
        self.last_update = now

        t1, t2, t3, t4, t5, t6 = self.last_thrusters
        denom = max(1e-6, 4.0 * self.max_horizontal)
        surge = (t1 + t2 + t3 + t4) / denom
        sway = (-t1 + t2 + t3 - t4) / denom
        yaw_cmd = (-t1 + t2 - t3 + t4) / max(1e-6, denom * self.yaw_scale)
        heave = (t5 + t6) / max(1e-6, 2.0 * self.max_vertical)

        surge = max(-1.0, min(1.0, surge))
        sway = max(-1.0, min(1.0, sway))
        yaw_cmd = max(-1.0, min(1.0, yaw_cmd))
        heave = max(-1.0, min(1.0, heave))

        target_vx = surge * self.max_xy_speed
        target_vy = sway * self.max_xy_speed
        target_vz = heave * self.max_z_speed
        target_yaw_rate = yaw_cmd * self.max_yaw_rate

        self.body_vx = approach(self.body_vx, target_vx, dt, self.linear_response_s)
        self.body_vy = approach(self.body_vy, target_vy, dt, self.linear_response_s)
        self.body_vz = approach(self.body_vz, target_vz, dt, self.vertical_response_s)
        self.yaw_rate = approach(self.yaw_rate, target_yaw_rate, dt, self.yaw_response_s)

        target_roll = max(-self.max_visual_tilt, min(self.max_visual_tilt, -sway * self.max_visual_tilt))
        target_pitch = max(-self.max_visual_tilt, min(self.max_visual_tilt, surge * self.max_visual_tilt * 0.75))
        self.roll = approach(self.roll, target_roll, dt, self.attitude_response_s)
        self.pitch = approach(self.pitch, target_pitch, dt, self.attitude_response_s)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (self.body_vx * cos_yaw - self.body_vy * sin_yaw) * dt
        self.y += (self.body_vx * sin_yaw + self.body_vy * cos_yaw) * dt
        self.z += self.body_vz * dt
        self.yaw += self.yaw_rate * dt

        self.x = max(-4.65, min(4.65, self.x))
        self.y = max(-4.65, min(4.65, self.y))
        self.z = max(self.bottom_limit_z, min(self.surface_limit_z, self.z))

        pose = self.make_pose()
        self.publish_odom(pose, self.body_vx, self.body_vy, self.body_vz, self.yaw_rate)
        self.set_model_pose(self.model_name, pose, 25)
        self.update_propeller_visuals(pose, dt)

    def make_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        qx, qy, qz, qw = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def publish_odom(self, pose: Pose, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "world"
        odom.child_frame_id = "gamantaray_rov/base_link"
        odom.pose.pose = pose
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = yaw_rate
        self.odom_pub.publish(odom)

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

    def update_propeller_visuals(self, pose: Pose, dt: float) -> None:
        if not self.prop_visuals_enabled:
            return

        for index, (spec, thrust) in enumerate(zip(self.prop_specs, self.last_thrusters)):
            normalized = max(-1.0, min(1.0, thrust / max(1e-6, spec["limit"])))
            self.prop_spin_angles[index] = math.remainder(
                self.prop_spin_angles[index]
                + normalized * self.prop_spin_gain * spec["direction"] * dt,
                2.0 * math.pi,
            )
            prop_pose = self.make_propeller_pose(pose, spec, self.prop_spin_angles[index])
            self.set_model_pose(spec["name"], prop_pose, 5)

    def make_propeller_pose(self, rov_pose: Pose, spec: dict, spin_angle: float) -> Pose:
        offset_x, offset_y, offset_z = spec["offset"]
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        prop_pose = Pose()
        prop_pose.position.x = rov_pose.position.x + offset_x * cos_yaw - offset_y * sin_yaw
        prop_pose.position.y = rov_pose.position.y + offset_x * sin_yaw + offset_y * cos_yaw
        prop_pose.position.z = rov_pose.position.z + offset_z

        roll, pitch, yaw = spec["rpy"]
        rov_q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        base_local_q = quaternion_from_euler(roll, pitch, yaw)
        if spec.get("spin_axis") == "z":
            spin_q = quaternion_from_euler(0.0, 0.0, spin_angle)
        else:
            spin_q = quaternion_from_euler(spin_angle, 0.0, 0.0)
        local_q = quaternion_multiply(base_local_q, spin_q)
        qx, qy, qz, qw = quaternion_multiply(rov_q, local_q)
        prop_pose.orientation.x = qx
        prop_pose.orientation.y = qy
        prop_pose.orientation.z = qz
        prop_pose.orientation.w = qw
        return prop_pose


def main() -> None:
    rclpy.init()
    node = KinematicDriver()
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
