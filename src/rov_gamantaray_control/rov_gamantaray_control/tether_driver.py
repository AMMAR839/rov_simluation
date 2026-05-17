import math

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class TetherDriver(Node):
    """Move visual tether beads between the surface anchor and ROV."""

    def __init__(self) -> None:
        super().__init__("tether_driver")
        self.declare_parameter("world_name", "kki_rov_pool")
        self.declare_parameter("segment_prefix", "rov_tether_segment_")
        self.declare_parameter("segment_count", 28)
        self.declare_parameter("anchor_x", -4.55)
        self.declare_parameter("anchor_y", -4.55)
        self.declare_parameter("anchor_z", 0.06)
        self.declare_parameter("rov_attach_z_offset", 0.12)
        self.declare_parameter("min_sag_m", 0.05)
        self.declare_parameter("max_sag_m", 0.32)
        self.declare_parameter("max_length_m", 12.0)
        self.declare_parameter("floor_z", -0.84)

        self.world_name = str(self.get_parameter("world_name").value)
        self.segment_prefix = str(self.get_parameter("segment_prefix").value)
        self.segment_count = int(self.get_parameter("segment_count").value)
        self.anchor = (
            float(self.get_parameter("anchor_x").value),
            float(self.get_parameter("anchor_y").value),
            float(self.get_parameter("anchor_z").value),
        )
        self.rov_attach_z_offset = float(self.get_parameter("rov_attach_z_offset").value)
        self.min_sag = float(self.get_parameter("min_sag_m").value)
        self.max_sag = float(self.get_parameter("max_sag_m").value)
        self.max_length = float(self.get_parameter("max_length_m").value)
        self.floor_z = float(self.get_parameter("floor_z").value)

        self.gz_node = GzNode()
        self.pose_service = f"/world/{self.world_name}/set_pose"
        self.status_pub = self.create_publisher(String, "/rov/tether_status", 10)
        self.create_subscription(Odometry, "/model/gamantaray_rov/odometry", self.odom_callback, 10)
        self.get_logger().info("Tether visual driver ready")

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        target = (float(p.x), float(p.y), float(p.z) + self.rov_attach_z_offset)
        ax, ay, az = self.anchor
        tx, ty, tz = target
        dx = tx - ax
        dy = ty - ay
        dz = tz - az
        straight_length = math.sqrt(dx * dx + dy * dy + dz * dz)
        tension = straight_length / max(0.01, self.max_length)
        state = "slack"
        if tension > 0.96:
            state = "high"
        elif tension > 0.78:
            state = "nominal"

        sag_scale = 1.0 - min(0.85, max(0.0, tension - 0.55) * 1.2)
        sag = max(self.min_sag, min(self.max_sag, 0.07 + 0.035 * straight_length))
        sag *= sag_scale

        for index in range(self.segment_count):
            s = (index + 1.0) / (self.segment_count + 1.0)
            x = ax + dx * s
            y = ay + dy * s
            z = az + dz * s - sag * math.sin(math.pi * s)
            z = max(self.floor_z + 0.025, z)
            self.set_pose(f"{self.segment_prefix}{index + 1:02d}", x, y, z)

        self.status_pub.publish(
            String(
                data=(
                    f"state={state} length={straight_length:.2f} "
                    f"max={self.max_length:.2f} tension={tension:.2f}"
                )
            )
        )

    def set_pose(self, model_name: str, x: float, y: float, z: float) -> None:
        request = GzPose()
        request.name = model_name
        request.position.x = float(x)
        request.position.y = float(y)
        request.position.z = float(z)
        request.orientation.w = 1.0
        self.gz_node.request(self.pose_service, request, GzPose, Boolean, 5)


def main() -> None:
    rclpy.init()
    node = TetherDriver()
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
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
