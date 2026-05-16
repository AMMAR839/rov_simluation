import errno
import os
import struct
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class JoystickDriver(Node):
    """Read /dev/input/js* directly and publish ROV command topics."""

    def __init__(self) -> None:
        super().__init__("rov_joystick")
        self.declare_parameter("device_path", "/dev/input/js0")
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("linear_scale", 0.75)
        self.declare_parameter("vertical_scale", 0.55)
        self.declare_parameter("yaw_scale", 0.65)
        self.declare_parameter("axis_surge", 1)
        self.declare_parameter("axis_sway", 0)
        self.declare_parameter("axis_heave", 3)
        self.declare_parameter("axis_yaw", 2)
        self.declare_parameter("invert_surge", True)
        self.declare_parameter("invert_sway", True)
        self.declare_parameter("invert_heave", True)
        self.declare_parameter("invert_yaw", False)
        self.declare_parameter("enable_button", -1)
        self.declare_parameter("button_surge_forward", -1)
        self.declare_parameter("button_surge_reverse", -1)
        self.declare_parameter("button_sway_left", -1)
        self.declare_parameter("button_sway_right", -1)
        self.declare_parameter("button_heave_up", -1)
        self.declare_parameter("button_heave_down", -1)
        self.declare_parameter("button_yaw_left", -1)
        self.declare_parameter("button_yaw_right", -1)
        self.declare_parameter("button_close_gripper", 0)
        self.declare_parameter("button_open_gripper", 1)

        self.device_path = str(self.get_parameter("device_path").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.vertical_scale = float(self.get_parameter("vertical_scale").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.axis_surge = int(self.get_parameter("axis_surge").value)
        self.axis_sway = int(self.get_parameter("axis_sway").value)
        self.axis_heave = int(self.get_parameter("axis_heave").value)
        self.axis_yaw = int(self.get_parameter("axis_yaw").value)
        self.invert_surge = bool(self.get_parameter("invert_surge").value)
        self.invert_sway = bool(self.get_parameter("invert_sway").value)
        self.invert_heave = bool(self.get_parameter("invert_heave").value)
        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)
        self.enable_button = int(self.get_parameter("enable_button").value)
        self.button_surge_forward = int(self.get_parameter("button_surge_forward").value)
        self.button_surge_reverse = int(self.get_parameter("button_surge_reverse").value)
        self.button_sway_left = int(self.get_parameter("button_sway_left").value)
        self.button_sway_right = int(self.get_parameter("button_sway_right").value)
        self.button_heave_up = int(self.get_parameter("button_heave_up").value)
        self.button_heave_down = int(self.get_parameter("button_heave_down").value)
        self.button_yaw_left = int(self.get_parameter("button_yaw_left").value)
        self.button_yaw_right = int(self.get_parameter("button_yaw_right").value)
        self.button_close = int(self.get_parameter("button_close_gripper").value)
        self.button_open = int(self.get_parameter("button_open_gripper").value)

        self.axes: dict[int, float] = {}
        self.buttons: dict[int, int] = {}
        self.fd: Optional[int] = None
        self.warned_missing = False

        self.cmd_pub = self.create_publisher(Twist, "/rov/cmd_vel", 10)
        self.gripper_pub = self.create_publisher(Float64, "/rov/gripper_cmd", 10)
        self.create_timer(0.05, self.update)

    def open_device(self) -> bool:
        if self.fd is not None:
            return True
        try:
            self.fd = os.open(self.device_path, os.O_RDONLY | os.O_NONBLOCK)
            self.warned_missing = False
            self.get_logger().info(f"Joystick connected: {self.device_path}")
            return True
        except OSError as exc:
            if not self.warned_missing:
                self.get_logger().warning(
                    f"Cannot open {self.device_path}: {exc}. "
                    "Plug in a gamepad or check /dev/input/js* permission."
                )
                self.warned_missing = True
            return False

    def update(self) -> None:
        if not self.open_device():
            self.cmd_pub.publish(Twist())
            return

        try:
            while True:
                data = os.read(self.fd, JS_EVENT_SIZE)
                if len(data) != JS_EVENT_SIZE:
                    break
                _, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, data)
                event_type &= ~JS_EVENT_INIT
                if event_type == JS_EVENT_AXIS:
                    self.axes[number] = self.normalize_axis(value)
                elif event_type == JS_EVENT_BUTTON:
                    self.buttons[number] = int(value)
                    self.handle_button(number, int(value))
        except BlockingIOError:
            pass
        except OSError as exc:
            if exc.errno in {errno.ENODEV, errno.EIO}:
                self.get_logger().warning("Joystick disconnected")
                os.close(self.fd)
                self.fd = None
                self.cmd_pub.publish(Twist())
                return
            raise

        self.cmd_pub.publish(self.make_twist())

    def normalize_axis(self, value: int) -> float:
        normalized = clamp(value / 32767.0, -1.0, 1.0)
        if abs(normalized) < self.deadzone:
            return 0.0
        return normalized

    def axis_value(self, axis: int, invert: bool) -> float:
        value = self.axes.get(axis, 0.0)
        return -value if invert else value

    def button_pressed(self, button: int) -> bool:
        return button >= 0 and self.buttons.get(button, 0) == 1

    def button_pair_value(self, positive_button: int, negative_button: int) -> float:
        positive = 1.0 if self.button_pressed(positive_button) else 0.0
        negative = 1.0 if self.button_pressed(negative_button) else 0.0
        return positive - negative

    def motion_value(
        self,
        axis: int,
        invert: bool,
        positive_button: int,
        negative_button: int,
    ) -> float:
        return clamp(
            self.axis_value(axis, invert)
            + self.button_pair_value(positive_button, negative_button),
            -1.0,
            1.0,
        )

    def make_twist(self) -> Twist:
        msg = Twist()
        if self.enable_button >= 0 and not self.button_pressed(self.enable_button):
            return msg

        msg.linear.x = (
            self.motion_value(
                self.axis_surge,
                self.invert_surge,
                self.button_surge_forward,
                self.button_surge_reverse,
            )
            * self.linear_scale
        )
        msg.linear.y = (
            self.motion_value(
                self.axis_sway,
                self.invert_sway,
                self.button_sway_left,
                self.button_sway_right,
            )
            * self.linear_scale
        )
        msg.linear.z = (
            self.motion_value(
                self.axis_heave,
                self.invert_heave,
                self.button_heave_up,
                self.button_heave_down,
            )
            * self.vertical_scale
        )
        msg.angular.z = (
            self.motion_value(
                self.axis_yaw,
                self.invert_yaw,
                self.button_yaw_left,
                self.button_yaw_right,
            )
            * self.yaw_scale
        )
        return msg

    def handle_button(self, number: int, pressed: int) -> None:
        if not pressed:
            return
        if number == self.button_close:
            self.gripper_pub.publish(Float64(data=1.0))
            self.get_logger().info("Gripper close")
        elif number == self.button_open:
            self.gripper_pub.publish(Float64(data=0.0))
            self.get_logger().info("Gripper open")

    def destroy_node(self) -> bool:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = JoystickDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
