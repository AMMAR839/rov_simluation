import math

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class WaterEffectsDriver(Node):
    """Move a visual-only surface wake when the ROV is close to the surface."""

    def __init__(self) -> None:
        super().__init__("water_effects_driver")
        self.declare_parameter("world_name", "kki_rov_pool")
        self.declare_parameter("wake_model_name", "rov_surface_wake")
        self.declare_parameter("surface_z", 0.018)
        self.declare_parameter("hide_z", 6.0)
        self.declare_parameter("active_depth_z", -0.18)
        self.declare_parameter("min_speed_mps", 0.06)

        self.world_name = str(self.get_parameter("world_name").value)
        self.wake_model_name = str(self.get_parameter("wake_model_name").value)
        self.surface_z = float(self.get_parameter("surface_z").value)
        self.hide_z = float(self.get_parameter("hide_z").value)
        self.active_depth_z = float(self.get_parameter("active_depth_z").value)
        self.min_speed = float(self.get_parameter("min_speed_mps").value)

        self.gz_node = GzNode()
        self.pose_service = f"/world/{self.world_name}/set_pose"
        self.create_subscription(Odometry, "/model/gamantaray_rov/odometry", self.odom_callback, 10)

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        speed_xy = math.hypot(v.x, v.y)
        near_surface = p.z >= self.active_depth_z

        request = GzPose()
        request.name = self.wake_model_name

        if near_surface and speed_xy >= self.min_speed:
            yaw = math.atan2(v.y, v.x) if speed_xy > 1e-4 else yaw_from_quaternion(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            request.position.x = p.x
            request.position.y = p.y
            request.position.z = self.surface_z
            request.orientation.x = qx
            request.orientation.y = qy
            request.orientation.z = qz
            request.orientation.w = qw
        else:
            request.position.x = 0.0
            request.position.y = 0.0
            request.position.z = self.hide_z
            request.orientation.w = 1.0

        self.gz_node.request(self.pose_service, request, GzPose, Boolean, 5)


def main() -> None:
    rclpy.init()
    node = WaterEffectsDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if "Unable to convert call argument" not in str(exc):
            raise
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
