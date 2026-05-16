import math
import time

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def approach(current: float, target: float, dt: float, time_constant: float) -> float:
    if time_constant <= 1e-6:
        return target
    alpha = 1.0 - math.exp(-dt / time_constant)
    return current + (target - current) * alpha


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class HydroWrenchDriver(Node):
    """Apply allocator thruster output as a persistent Gazebo wrench."""

    def __init__(self) -> None:
        super().__init__("hydro_wrench_driver")
        self.declare_parameter("world_name", "kki_rov_pool")
        self.declare_parameter("entity_name", "gamantaray_rov")
        self.declare_parameter("entity_type", "model")
        self.declare_parameter("horizontal_force_gain", 2.00)
        self.declare_parameter("vertical_force_gain", 0.80)
        self.declare_parameter("yaw_torque_gain", 0.20)
        self.declare_parameter("pose_assist_enabled", True)
        self.declare_parameter("max_horizontal_thrust_n", 22.0)
        self.declare_parameter("max_vertical_thrust_n", 18.0)
        self.declare_parameter("thruster_yaw_scale", 0.75)
        self.declare_parameter("max_xy_speed_mps", 0.42)
        self.declare_parameter("max_z_speed_mps", 0.22)
        self.declare_parameter("max_yaw_rate_rps", 0.45)
        self.declare_parameter("linear_response_s", 0.70)
        self.declare_parameter("vertical_response_s", 0.80)
        self.declare_parameter("yaw_response_s", 0.75)
        self.declare_parameter("initial_x", -3.2)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", -0.28)
        self.declare_parameter("bottom_limit_z", -0.72)
        self.declare_parameter("surface_limit_z", -0.08)

        self.world_name = str(self.get_parameter("world_name").value)
        self.entity_name = str(self.get_parameter("entity_name").value)
        self.entity_type_name = str(self.get_parameter("entity_type").value).lower()
        self.entity_type = self.resolve_entity_type(self.entity_type_name)
        self.horizontal_gain = float(self.get_parameter("horizontal_force_gain").value)
        self.vertical_gain = float(self.get_parameter("vertical_force_gain").value)
        self.yaw_gain = float(self.get_parameter("yaw_torque_gain").value)
        self.pose_assist_enabled = bool(self.get_parameter("pose_assist_enabled").value)
        self.max_horizontal = float(self.get_parameter("max_horizontal_thrust_n").value)
        self.max_vertical = float(self.get_parameter("max_vertical_thrust_n").value)
        self.thruster_yaw_scale = float(self.get_parameter("thruster_yaw_scale").value)
        self.max_xy_speed = float(self.get_parameter("max_xy_speed_mps").value)
        self.max_z_speed = float(self.get_parameter("max_z_speed_mps").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self.linear_response_s = float(self.get_parameter("linear_response_s").value)
        self.vertical_response_s = float(self.get_parameter("vertical_response_s").value)
        self.yaw_response_s = float(self.get_parameter("yaw_response_s").value)
        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.z = float(self.get_parameter("initial_z").value)
        self.bottom_limit_z = float(self.get_parameter("bottom_limit_z").value)
        self.surface_limit_z = float(self.get_parameter("surface_limit_z").value)

        self.last_thrusters = [0.0] * 6
        self.yaw = 0.0
        self.body_vx = 0.0
        self.body_vy = 0.0
        self.body_vz = 0.0
        self.yaw_rate = 0.0
        self.pose_initialized = False
        self.last_update = time.monotonic()

        self.gz_node = GzNode()
        self.pose_service = f"/world/{self.world_name}/set_pose"
        self.wrench_pub = self.gz_node.advertise(
            f"/world/{self.world_name}/wrench/persistent", EntityWrench
        )
        self.clear_pub = self.gz_node.advertise(f"/world/{self.world_name}/wrench/clear", Entity)

        self.create_subscription(
            Float64MultiArray, "/rov/thruster_status", self.thruster_callback, 10
        )
        self.create_subscription(Odometry, "/model/gamantaray_rov/odometry", self.odom_callback, 10)
        self.create_timer(0.05, self.update)

    def resolve_entity_type(self, value: str) -> int:
        if value == "model":
            return Entity.MODEL
        if value == "link":
            return Entity.LINK
        self.get_logger().warning(
            f"Unknown entity_type '{value}', using 'model'. Valid values: model, link."
        )
        return Entity.MODEL

    def thruster_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 6:
            self.last_thrusters = [float(v) for v in msg.data[:6]]

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if not self.pose_initialized:
            self.x = p.x
            self.y = p.y
            self.z = p.z
            self.pose_initialized = True

    def update(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_update))
        self.last_update = now

        t1, t2, t3, t4, t5, t6 = self.last_thrusters
        body_fx = (t1 + t2 + t3 + t4) * self.horizontal_gain
        body_fy = (-t1 + t2 + t3 - t4) * self.horizontal_gain
        body_fz = (t5 + t6) * self.vertical_gain
        body_tz = (-t1 + t2 - t3 + t4) * self.yaw_gain

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        world_fx = body_fx * cos_yaw - body_fy * sin_yaw
        world_fy = body_fx * sin_yaw + body_fy * cos_yaw

        self.publish_wrench(world_fx, world_fy, body_fz, body_tz)
        if self.pose_assist_enabled:
            self.update_pose_assist(t1, t2, t3, t4, t5, t6, dt)

    def update_pose_assist(
        self,
        t1: float,
        t2: float,
        t3: float,
        t4: float,
        t5: float,
        t6: float,
        dt: float,
    ) -> None:
        if dt <= 0.0:
            return

        horizontal_denom = max(1e-6, 4.0 * self.max_horizontal)
        vertical_denom = max(1e-6, 2.0 * self.max_vertical)
        yaw_denom = max(1e-6, horizontal_denom * self.thruster_yaw_scale)

        surge = clamp((t1 + t2 + t3 + t4) / horizontal_denom, -1.0, 1.0)
        sway = clamp((-t1 + t2 + t3 - t4) / horizontal_denom, -1.0, 1.0)
        yaw_cmd = clamp((-t1 + t2 - t3 + t4) / yaw_denom, -1.0, 1.0)
        heave = clamp((t5 + t6) / vertical_denom, -1.0, 1.0)

        self.body_vx = approach(
            self.body_vx, surge * self.max_xy_speed, dt, self.linear_response_s
        )
        self.body_vy = approach(
            self.body_vy, sway * self.max_xy_speed, dt, self.linear_response_s
        )
        self.body_vz = approach(
            self.body_vz, heave * self.max_z_speed, dt, self.vertical_response_s
        )
        self.yaw_rate = approach(
            self.yaw_rate, yaw_cmd * self.max_yaw_rate, dt, self.yaw_response_s
        )

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (self.body_vx * cos_yaw - self.body_vy * sin_yaw) * dt
        self.y += (self.body_vx * sin_yaw + self.body_vy * cos_yaw) * dt
        self.z = clamp(self.z + self.body_vz * dt, self.bottom_limit_z, self.surface_limit_z)
        self.yaw += self.yaw_rate * dt

        self.set_pose()

    def set_pose(self) -> None:
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        request = GzPose()
        request.name = self.entity_name
        request.position.x = float(self.x)
        request.position.y = float(self.y)
        request.position.z = float(self.z)
        request.orientation.x = qx
        request.orientation.y = qy
        request.orientation.z = qz
        request.orientation.w = qw
        self.gz_node.request(self.pose_service, request, GzPose, Boolean, 5)

    def publish_wrench(self, force_x: float, force_y: float, force_z: float, torque_z: float) -> None:
        msg = EntityWrench()
        msg.entity.name = self.entity_name
        msg.entity.type = self.entity_type
        msg.wrench.force.x = float(force_x)
        msg.wrench.force.y = float(force_y)
        msg.wrench.force.z = float(force_z)
        msg.wrench.torque.z = float(torque_z)
        self.wrench_pub.publish(msg)

    def clear_wrench(self) -> None:
        self.publish_wrench(0.0, 0.0, 0.0, 0.0)
        msg = Entity()
        msg.name = self.entity_name
        msg.type = self.entity_type
        self.clear_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = HydroWrenchDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.clear_wrench()
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
