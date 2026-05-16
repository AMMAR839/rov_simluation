import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


HELP = """
ROV teleop:
  w/s maju/mundur       a/d geser kiri/kanan
  r/f naik/turun        q/e yaw kiri/kanan
  o buka gripper        p tutup gripper
  space stop gerak      Ctrl-C keluar

Mode ini momentary: ROV bergerak saat tombol ditahan. Setelah tombol
dilepas, key repeat berhenti dan command otomatis nol dalam waktu singkat.
"""


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("rov_teleop_keyboard")
        self.declare_parameter("linear_speed", 0.65)
        self.declare_parameter("vertical_speed", 0.45)
        self.declare_parameter("yaw_speed", 0.50)
        self.declare_parameter("key_hold_timeout_s", 0.35)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.vertical_speed = float(self.get_parameter("vertical_speed").value)
        self.yaw_speed = float(self.get_parameter("yaw_speed").value)
        self.key_hold_timeout_s = float(self.get_parameter("key_hold_timeout_s").value)
        self.cmd = Twist()
        self.last_motion_key_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/rov/cmd_vel", 10)
        self.gripper_pub = self.create_publisher(Float64, "/rov/gripper_cmd", 10)
        self.create_timer(0.05, self.publish_current_command)

    def publish_key(self, key: str) -> None:
        motion_cmd = Twist()
        is_motion_key = False
        if key == "w":
            motion_cmd.linear.x = self.linear_speed
            is_motion_key = True
        elif key == "s":
            motion_cmd.linear.x = -self.linear_speed
            is_motion_key = True
        elif key == "a":
            motion_cmd.linear.y = self.linear_speed
            is_motion_key = True
        elif key == "d":
            motion_cmd.linear.y = -self.linear_speed
            is_motion_key = True
        elif key == "r":
            motion_cmd.linear.z = self.vertical_speed
            is_motion_key = True
        elif key == "f":
            motion_cmd.linear.z = -self.vertical_speed
            is_motion_key = True
        elif key == "q":
            motion_cmd.angular.z = self.yaw_speed
            is_motion_key = True
        elif key == "e":
            motion_cmd.angular.z = -self.yaw_speed
            is_motion_key = True
        elif key == " ":
            self.cmd = Twist()
            self.last_motion_key_time = 0.0
        elif key == "o":
            self.gripper_pub.publish(Float64(data=0.0))
        elif key == "p":
            self.gripper_pub.publish(Float64(data=1.0))

        if is_motion_key and key != " ":
            self.cmd = motion_cmd
            self.last_motion_key_time = time.monotonic()

    def publish_current_command(self) -> None:
        if (
            self.last_motion_key_time > 0.0
            and time.monotonic() - self.last_motion_key_time > self.key_hold_timeout_s
        ):
            self.cmd = Twist()
            self.last_motion_key_time = 0.0
        self.cmd_pub.publish(self.cmd)


def get_key() -> str:
    readable, _, _ = select.select([sys.stdin], [], [], 0.05)
    if readable:
        return sys.stdin.read(1)
    return ""


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = get_key()
            if key:
                node.publish_key(key)
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
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
