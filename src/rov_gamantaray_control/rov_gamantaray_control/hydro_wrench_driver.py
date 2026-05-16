import math

import rclpy
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class HydroWrenchDriver(Node):
    """Apply allocator thruster output as a persistent Gazebo link wrench."""

    def __init__(self) -> None:
        super().__init__("hydro_wrench_driver")
        self.declare_parameter("world_name", "kki_rov_pool")
        self.declare_parameter("entity_name", "gamantaray_rov::base_link")
        self.declare_parameter("horizontal_force_gain", 0.05)
        self.declare_parameter("vertical_force_gain", 0.12)
        self.declare_parameter("yaw_torque_gain", 0.015)

        self.world_name = str(self.get_parameter("world_name").value)
        self.entity_name = str(self.get_parameter("entity_name").value)
        self.horizontal_gain = float(self.get_parameter("horizontal_force_gain").value)
        self.vertical_gain = float(self.get_parameter("vertical_force_gain").value)
        self.yaw_gain = float(self.get_parameter("yaw_torque_gain").value)

        self.last_thrusters = [0.0] * 6
        self.yaw = 0.0

        self.gz_node = GzNode()
        self.wrench_pub = self.gz_node.advertise(
            f"/world/{self.world_name}/wrench/persistent", EntityWrench
        )
        self.clear_pub = self.gz_node.advertise(f"/world/{self.world_name}/wrench/clear", Entity)

        self.create_subscription(
            Float64MultiArray, "/rov/thruster_status", self.thruster_callback, 10
        )
        self.create_subscription(Odometry, "/model/gamantaray_rov/odometry", self.odom_callback, 10)
        self.create_timer(0.05, self.update)

    def thruster_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 6:
            self.last_thrusters = [float(v) for v in msg.data[:6]]

    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def update(self) -> None:
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

    def publish_wrench(self, force_x: float, force_y: float, force_z: float, torque_z: float) -> None:
        msg = EntityWrench()
        msg.entity.name = self.entity_name
        msg.entity.type = Entity.LINK
        msg.wrench.force.x = float(force_x)
        msg.wrench.force.y = float(force_y)
        msg.wrench.force.z = float(force_z)
        msg.wrench.torque.z = float(torque_z)
        self.wrench_pub.publish(msg)

    def clear_wrench(self) -> None:
        self.publish_wrench(0.0, 0.0, 0.0, 0.0)
        msg = Entity()
        msg.name = self.entity_name
        msg.type = Entity.LINK
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
