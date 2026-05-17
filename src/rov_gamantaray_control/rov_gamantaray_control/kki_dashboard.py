import math
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


VALID_CODES = {"A", "B", "C", "D"}
BG_TOP = (250, 252, 254)
BG_BOTTOM = (238, 244, 249)
CARD = (255, 255, 255)
CARD_SOFT = (240, 247, 251)
CAMERA_BG = (18, 26, 32)
TEXT = (34, 47, 61)
MUTED = (96, 112, 128)
LINE = (190, 207, 221)
ACCENT = (176, 124, 20)
OK = (74, 222, 128)
WARN = (55, 156, 238)
BAD = (86, 104, 255)


def parse_key_value_status(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for item in text.split():
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        values[key.strip()] = value.strip()
    return values


class KkiDashboard(Node):
    """OpenCV-based KKI operator dashboard."""

    def __init__(self) -> None:
        super().__init__("kki_dashboard")
        self.declare_parameter("team_name", os.environ.get("KKI_TEAM_NAME", "Gamantara"))
        self.declare_parameter(
            "university_name",
            os.environ.get("KKI_UNIVERSITY_NAME", "Universitas Gadjah Mada"),
        )
        self.declare_parameter("wall_image_topic", "/rov/camera/wall/image")
        self.declare_parameter("bottom_image_topic", "/rov/camera/bottom/image")
        self.declare_parameter("qr_topic", "/rov/qr_code")
        self.declare_parameter("odom_topic", "/model/gamantaray_rov/odometry")
        self.declare_parameter("gripper_status_topic", "/rov/gripper_status")
        self.declare_parameter("mission_state_topic", "/rov/mission_state")
        self.declare_parameter("tether_status_topic", "/rov/tether_status")
        self.declare_parameter("pool_floor_z", -0.88)
        self.declare_parameter("window_name", "KKI ROV GUI")
        self.declare_parameter("window_enabled", True)
        self.declare_parameter("max_trajectory_points", 1800)

        self.team_name = str(self.get_parameter("team_name").value)
        self.university_name = str(self.get_parameter("university_name").value)
        self.pool_floor_z = float(self.get_parameter("pool_floor_z").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.window_enabled = bool(self.get_parameter("window_enabled").value)
        self.max_trajectory_points = int(self.get_parameter("max_trajectory_points").value)

        self.wall_image: np.ndarray | None = None
        self.bottom_image: np.ndarray | None = None
        self.qr_code = ""
        self.qr_time = 0.0
        self.altitude_m = 0.0
        self.depth_m = 0.0
        self.position = (-3.2, 0.0, -0.28)
        self.yaw = 0.0
        self.trajectory: list[tuple[float, float]] = []
        self.gripper_status = "no_status"
        self.mission_state = "manual"
        self.tether_status = "tether=unknown"
        self.last_odom_time = 0.0

        self.create_subscription(
            Image,
            str(self.get_parameter("wall_image_topic").value),
            self.wall_image_callback,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("bottom_image_topic").value),
            self.bottom_image_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("qr_topic").value),
            self.qr_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("gripper_status_topic").value),
            self.gripper_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            self.mission_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("tether_status_topic").value),
            self.tether_callback,
            10,
        )
        self.create_timer(0.10, self.render)
        self.get_logger().info("KKI dashboard ready")

    def wall_image_callback(self, msg: Image) -> None:
        image = self.image_msg_to_bgr(msg)
        if image is not None:
            self.wall_image = image

    def bottom_image_callback(self, msg: Image) -> None:
        image = self.image_msg_to_bgr(msg)
        if image is not None:
            self.bottom_image = image

    def qr_callback(self, msg: String) -> None:
        code = msg.data.strip().upper()
        if code in VALID_CODES:
            self.qr_code = code
            self.qr_time = time.monotonic()

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.position = (float(p.x), float(p.y), float(p.z))
        self.yaw = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.altitude_m = max(0.0, float(p.z) - self.pool_floor_z)
        self.depth_m = max(0.0, -float(p.z))
        self.last_odom_time = time.monotonic()

        if not self.trajectory:
            self.trajectory.append((float(p.x), float(p.y)))
            return

        last_x, last_y = self.trajectory[-1]
        if math.hypot(float(p.x) - last_x, float(p.y) - last_y) >= 0.025:
            self.trajectory.append((float(p.x), float(p.y)))
            if len(self.trajectory) > self.max_trajectory_points:
                self.trajectory = self.trajectory[-self.max_trajectory_points :]

    def gripper_callback(self, msg: String) -> None:
        self.gripper_status = msg.data

    def mission_callback(self, msg: String) -> None:
        self.mission_state = msg.data

    def tether_callback(self, msg: String) -> None:
        self.tether_status = msg.data

    @staticmethod
    def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def image_msg_to_bgr(self, msg: Image) -> np.ndarray | None:
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            encoding = msg.encoding.lower()
            height = int(msg.height)
            width = int(msg.width)
            step = int(msg.step)
            if height <= 0 or width <= 0:
                return None

            if encoding in {"bgr8", "rgb8"}:
                row = data.reshape((height, step))
                image = row[:, : width * 3].reshape((height, width, 3)).copy()
                if encoding == "rgb8":
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                return image
            if encoding in {"bgra8", "rgba8"}:
                row = data.reshape((height, step))
                image = row[:, : width * 4].reshape((height, width, 4)).copy()
                code = cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR
                return cv2.cvtColor(image, code)
            if encoding in {"mono8", "8uc1"}:
                row = data.reshape((height, step))
                image = row[:, :width].reshape((height, width)).copy()
                return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        except Exception as exc:
            self.get_logger().warning(f"Dashboard image conversion failed: {exc}")
        return None

    def render(self) -> None:
        canvas = self.make_background(1600, 900)
        self.draw_header(canvas)
        self.draw_camera_panel(canvas, self.wall_image, (38, 128, 505, 315), "CAMERA 1", "Front / QR Cam")
        self.draw_camera_panel(
            canvas,
            self.bottom_image,
            (565, 128, 505, 315),
            "CAMERA 2",
            "Bottom / Side Cam",
        )
        self.draw_status_panel(canvas, (1092, 128, 470, 315))
        self.draw_altitude_panel(canvas, (38, 470, 360, 315))
        self.draw_trajectory_panel(canvas, (420, 470, 570, 315))
        self.draw_rov_design_panel(canvas, (1012, 470, 550, 315))
        self.draw_footer(canvas)

        if not self.window_enabled:
            return
        try:
            cv2.imshow(self.window_name, canvas)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.window_enabled = False
            self.get_logger().warning(f"Dashboard window disabled: {exc}")

    def draw_header(self, canvas: np.ndarray) -> None:
        now = datetime.now().strftime("%A, %d %B %Y  %H:%M:%S")
        self.put_text(canvas, "KKI ROV GUI", (38, 52), 1.05, TEXT, 2)
        self.put_text(canvas, "Gamantara - Universitas Gadjah Mada", (40, 84), 0.58, ACCENT, 1)

        self.rounded_rect(canvas, (760, 28, 802, 70), CARD, 8, fill=True)
        self.rounded_rect(canvas, (760, 28, 802, 70), LINE, 8, fill=False, thickness=1)
        self.put_text(canvas, "Top Information Bar", (784, 52), 0.50, MUTED, 1)
        self.put_text(canvas, f"Team: {self.team_name}", (784, 78), 0.55, TEXT, 1)
        self.put_text(canvas, f"University: {self.university_name}", (1010, 78), 0.55, TEXT, 1)
        self.put_text(canvas, now, (1274, 78), 0.52, TEXT, 1)

        odom_age = time.monotonic() - self.last_odom_time if self.last_odom_time else 999.0
        self.status_chip(canvas, (505, 36, 100, 32), "ODOM", odom_age < 1.0)
        qr_age = time.monotonic() - self.qr_time if self.qr_time else 999.0
        self.status_chip(canvas, (616, 36, 84, 32), "QR", self.qr_code in VALID_CODES and qr_age < 5.0)

    def draw_camera_panel(
        self,
        canvas: np.ndarray,
        image: np.ndarray | None,
        rect: tuple[int, int, int, int],
        title: str,
        subtitle: str,
    ) -> None:
        x, y, w, h = rect
        self.panel(canvas, rect, title)
        self.put_text(canvas, subtitle, (x + w - 170, y + 27), 0.50, MUTED, 1)
        inner = (x + 18, y + 50, w - 36, h - 68)
        ix, iy, iw, ih = inner
        self.rounded_rect(canvas, inner, CAMERA_BG, 6, fill=True)
        self.rounded_rect(canvas, inner, (44, 68, 84), 6, fill=False, thickness=1)
        if image is None:
            self.put_text(canvas, "NO IMAGE", (ix + 160, iy + ih // 2), 0.82, (88, 108, 124), 2)
            return
        shown = self.fit_image(image, iw, ih)
        sy = iy + (ih - shown.shape[0]) // 2
        sx = ix + (iw - shown.shape[1]) // 2
        canvas[sy : sy + shown.shape[0], sx : sx + shown.shape[1]] = shown

    def draw_status_panel(self, canvas: np.ndarray, rect: tuple[int, int, int, int]) -> None:
        self.panel(canvas, rect, "QR CODE & STATUS")
        x, y, _w, _h = rect
        qr_age = time.monotonic() - self.qr_time if self.qr_time else 999.0
        qr_valid = self.qr_code in VALID_CODES and qr_age < 5.0
        qr_text = self.qr_code if qr_valid else "NONE"
        qr_color = OK if qr_valid else BAD
        side_text = qr_text if qr_valid else "A/B/C/D"
        self.rounded_rect(canvas, (x + 24, y + 58, 196, 92), CARD_SOFT, 8, fill=True)
        self.rounded_rect(canvas, (x + 24, y + 58, 196, 92), qr_color, 8, fill=False, thickness=2)
        self.put_text(canvas, "TARGET SIDE", (x + 42, y + 86), 0.45, MUTED, 1)
        self.put_text(canvas, side_text, (x + 42, y + 132), 1.22, qr_color, 3)

        valid_text = "VALID" if qr_valid else "INVALID"
        self.rounded_rect(canvas, (x + 238, y + 58, 210, 92), CARD_SOFT, 8, fill=True)
        self.rounded_rect(canvas, (x + 238, y + 58, 210, 92), qr_color, 8, fill=False, thickness=2)
        self.put_text(canvas, "QR RESULT", (x + 256, y + 86), 0.45, MUTED, 1)
        self.put_text(canvas, valid_text, (x + 256, y + 132), 0.94, qr_color, 2)

        gripper = parse_key_value_status(self.gripper_status)
        mission = self.mission_state.split()[0] if self.mission_state else "manual"
        tether = parse_key_value_status(self.tether_status)
        self.status_tile(canvas, (x + 24, y + 174, 202, 46), "GRIPPER", self.gripper_status.split()[0] if self.gripper_status else "n/a")
        self.status_tile(canvas, (x + 246, y + 174, 202, 46), "PAYLOAD", gripper.get("payload", "unknown"))
        self.status_tile(canvas, (x + 24, y + 236, 202, 46), "HOOK", gripper.get("hook", "none"))
        self.status_tile(canvas, (x + 246, y + 236, 202, 46), "TETHER", tether.get("state", "unknown"))
        self.put_text(canvas, f"Mission: {mission}", (x + 24, y + 302), 0.57, TEXT, 1)

    def draw_altitude_panel(self, canvas: np.ndarray, rect: tuple[int, int, int, int]) -> None:
        self.panel(canvas, rect, "ALTITUDE")
        x, y, w, h = rect
        altitude_color = OK if self.altitude_m > 0.12 else WARN
        self.put_text(canvas, "Height from pool floor", (x + 24, y + 78), 0.55, MUTED, 1)
        self.put_text(canvas, f"{self.altitude_m:0.2f} m", (x + 24, y + 142), 1.45, altitude_color, 3)
        gauge_x, gauge_y, gauge_w, gauge_h = x + 270, y + 66, 42, 178
        self.rounded_rect(canvas, (gauge_x, gauge_y, gauge_w, gauge_h), (230, 238, 244), 8, fill=True)
        fill = max(0, min(gauge_h - 10, int((self.altitude_m / 1.6) * (gauge_h - 10))))
        self.rounded_rect(
            canvas,
            (gauge_x + 6, gauge_y + gauge_h - 6 - fill, gauge_w - 12, fill),
            altitude_color,
            6,
            fill=True,
        )
        self.put_text(canvas, f"Depth below surface: {self.depth_m:0.2f} m", (x + 24, y + 190), 0.58, TEXT, 1)
        self.put_text(
            canvas,
            f"Position: x={self.position[0]:0.2f} y={self.position[1]:0.2f} z={self.position[2]:0.2f}",
            (x + 24, y + 225),
            0.55,
            MUTED,
            1,
        )
        age = time.monotonic() - self.last_odom_time if self.last_odom_time else 999.0
        odom_ok = age < 1.0
        self.put_text(
            canvas,
            f"Sensor: {'OK' if odom_ok else 'NO ODOM'}",
            (x + 24, y + h - 38),
            0.62,
            OK if odom_ok else BAD,
            2,
        )

    def draw_trajectory_panel(self, canvas: np.ndarray, rect: tuple[int, int, int, int]) -> None:
        self.panel(canvas, rect, "TRAJECTORY MAP")
        x, y, w, h = rect
        ix, iy, iw, ih = x + 30, y + 55, w - 60, h - 85
        self.rounded_rect(canvas, (ix, iy, iw, ih), (236, 244, 248), 8, fill=True)
        for t in np.linspace(0.0, 1.0, 5):
            gx = int(ix + t * iw)
            gy = int(iy + t * ih)
            cv2.line(canvas, (gx, iy), (gx, iy + ih), (212, 225, 234), 1)
            cv2.line(canvas, (ix, gy), (ix + iw, gy), (212, 225, 234), 1)
        self.rounded_rect(canvas, (ix, iy, iw, ih), LINE, 8, fill=False, thickness=1)

        if len(self.trajectory) >= 2:
            points = [self.pool_to_map(px, py, ix, iy, iw, ih) for px, py in self.trajectory]
            for p1, p2 in zip(points[:-1], points[1:]):
                cv2.line(canvas, p1, p2, ACCENT, 2)

        rx, ry = self.pool_to_map(self.position[0], self.position[1], ix, iy, iw, ih)
        cv2.circle(canvas, (rx, ry), 6, OK, -1)
        hx = int(rx + math.cos(self.yaw) * 18)
        hy = int(ry - math.sin(self.yaw) * 18)
        cv2.line(canvas, (rx, ry), (hx, hy), OK, 2)
        self.put_text(canvas, "S", (ix + 8, iy + ih - 8), 0.55, TEXT, 1)
        self.put_text(canvas, "A", (ix + 8, iy + ih // 2), 0.55, BAD, 1)
        self.put_text(canvas, "B", (ix + iw - 24, iy + ih // 2), 0.55, OK, 1)
        self.put_text(canvas, "C", (ix + iw // 2, iy + 22), 0.55, WARN, 1)
        self.put_text(canvas, "D", (ix + iw // 2, iy + ih - 8), 0.55, ACCENT, 1)
        self.put_text(canvas, "- start point (S)  - path line  - end point/current ROV", (x + 32, y + h - 28), 0.48, MUTED, 1)

    def draw_rov_design_panel(self, canvas: np.ndarray, rect: tuple[int, int, int, int]) -> None:
        self.panel(canvas, rect, "ROV DESIGN")
        x, y, w, h = rect
        cx, cy = x + w // 2, y + h // 2 + 10
        cv2.rectangle(canvas, (cx - 145, cy - 70), (cx + 145, cy + 70), TEXT, 3)
        cv2.rectangle(canvas, (cx - 125, cy - 48), (cx + 125, cy + 48), ACCENT, 2)
        cv2.rectangle(canvas, (cx - 80, cy - 35), (cx + 80, cy + 35), (214, 180, 88), -1)
        cv2.rectangle(canvas, (cx - 80, cy - 35), (cx + 80, cy + 35), TEXT, 2)
        for sx in (-115, 115):
            for sy in (-42, 42):
                cv2.circle(canvas, (cx + sx, cy + sy), 22, (238, 245, 248), -1)
                cv2.circle(canvas, (cx + sx, cy + sy), 22, TEXT, 2)
                cv2.line(canvas, (cx + sx - 16, cy + sy), (cx + sx + 16, cy + sy), ACCENT, 2)
                cv2.line(canvas, (cx + sx, cy + sy - 16), (cx + sx, cy + sy + 16), ACCENT, 2)
        cv2.line(canvas, (cx + 145, cy), (cx + 198, cy), TEXT, 5)
        cv2.line(canvas, (cx + 198, cy), (cx + 228, cy - 22), TEXT, 4)
        cv2.line(canvas, (cx + 198, cy), (cx + 228, cy + 22), TEXT, 4)
        self.put_text(canvas, "+X", (cx + 175, cy - 80), 0.55, TEXT, 1)
        cv2.arrowedLine(canvas, (cx + 150, cy - 66), (cx + 220, cy - 66), OK, 2)
        self.put_text(canvas, "- 2D/3D ROV image  - axis indicator", (x + 34, y + h - 38), 0.54, MUTED, 1)

    def draw_footer(self, canvas: np.ndarray) -> None:
        self.rounded_rect(canvas, (38, 820, 1524, 52), CARD, 8, fill=True)
        self.rounded_rect(canvas, (38, 820, 1524, 52), LINE, 8, fill=False, thickness=1)
        age = time.monotonic() - self.last_odom_time if self.last_odom_time else 999.0
        connection = "OK" if age < 1.0 else "NO ODOM"
        logging = "READY"
        self.put_text(canvas, "Footer Status Bar", (60, 852), 0.50, MUTED, 1)
        self.put_text(canvas, f"Mode: {self.mission_state.split()[0] if self.mission_state else 'manual'}", (320, 852), 0.55, TEXT, 1)
        self.put_text(canvas, f"Connection: {connection}", (550, 852), 0.55, OK if connection == "OK" else BAD, 1)
        self.put_text(canvas, f"Sensor Status: {connection}", (825, 852), 0.55, TEXT, 1)
        self.put_text(canvas, f"Logging: {logging}", (1110, 852), 0.55, MUTED, 1)
        self.put_text(canvas, "Gamantara UGM", (1380, 852), 0.55, ACCENT, 1)

    @staticmethod
    def fit_image(image: np.ndarray, width: int, height: int) -> np.ndarray:
        ih, iw = image.shape[:2]
        scale = min(width / max(1, iw), height / max(1, ih))
        new_size = (max(1, int(iw * scale)), max(1, int(ih * scale)))
        return cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)

    @staticmethod
    def pool_to_map(
        x: float,
        y: float,
        ix: int,
        iy: int,
        iw: int,
        ih: int,
    ) -> tuple[int, int]:
        px = int(ix + (x + 5.0) / 10.0 * iw)
        py = int(iy + (5.0 - y) / 10.0 * ih)
        return max(ix, min(ix + iw, px)), max(iy, min(iy + ih, py))

    @staticmethod
    def make_background(width: int, height: int) -> np.ndarray:
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        for row in range(height):
            t = row / max(1, height - 1)
            color = tuple(int(BG_TOP[i] * (1.0 - t) + BG_BOTTOM[i] * t) for i in range(3))
            canvas[row, :] = color
        for x in range(0, width, 64):
            cv2.line(canvas, (x, 0), (x, height), (232, 240, 246), 1)
        for y in range(0, height, 64):
            cv2.line(canvas, (0, y), (width, y), (232, 240, 246), 1)
        return canvas

    @staticmethod
    def panel(canvas: np.ndarray, rect: tuple[int, int, int, int], title: str) -> None:
        x, y, w, h = rect
        KkiDashboard.rounded_rect(canvas, (x + 4, y + 6, w, h), (220, 230, 238), 8, fill=True)
        KkiDashboard.rounded_rect(canvas, rect, CARD, 8, fill=True)
        KkiDashboard.rounded_rect(canvas, rect, LINE, 8, fill=False, thickness=1)
        cv2.line(canvas, (x + 14, y + 39), (x + w - 14, y + 39), (218, 230, 238), 1)
        KkiDashboard.put_text(canvas, title, (x + 18, y + 27), 0.60, TEXT, 1)
        cv2.circle(canvas, (x + w - 24, y + 20), 4, ACCENT, -1)

    @staticmethod
    def framed_box(canvas: np.ndarray, rect: tuple[int, int, int, int], title: str) -> None:
        x, y, w, h = rect
        KkiDashboard.rounded_rect(canvas, rect, CARD, 8, fill=True)
        KkiDashboard.rounded_rect(canvas, rect, LINE, 8, fill=False, thickness=1)
        label_w = max(170, len(title) * 12)
        label_x = x + w // 2 - label_w // 2
        cv2.rectangle(canvas, (label_x, y - 10), (label_x + label_w, y + 12), BG_TOP, -1)
        KkiDashboard.put_text(canvas, title, (label_x + 12, y + 8), 0.50, MUTED, 1)

    @staticmethod
    def status_line(canvas: np.ndarray, x: int, y: int, label: str, value: str) -> None:
        KkiDashboard.put_text(canvas, f"- {label}:", (x, y), 0.55, MUTED, 1)
        KkiDashboard.put_text(canvas, value, (x + 105, y), 0.55, TEXT, 1)

    @staticmethod
    def status_tile(canvas: np.ndarray, rect: tuple[int, int, int, int], label: str, value: str) -> None:
        x, y, w, h = rect
        KkiDashboard.rounded_rect(canvas, rect, CARD_SOFT, 6, fill=True)
        KkiDashboard.put_text(canvas, label, (x + 12, y + 18), 0.42, MUTED, 1)
        KkiDashboard.put_text(canvas, value[:16], (x + 12, y + 37), 0.56, TEXT, 1)

    @staticmethod
    def status_chip(canvas: np.ndarray, rect: tuple[int, int, int, int], label: str, active: bool) -> None:
        x, y, w, h = rect
        color = OK if active else BAD
        KkiDashboard.rounded_rect(canvas, rect, CARD, 8, fill=True)
        KkiDashboard.rounded_rect(canvas, rect, LINE, 8, fill=False, thickness=1)
        cv2.circle(canvas, (x + 18, y + h // 2), 5, color, -1)
        KkiDashboard.put_text(canvas, label, (x + 32, y + 22), 0.46, TEXT, 1)

    @staticmethod
    def rounded_rect(
        canvas: np.ndarray,
        rect: tuple[int, int, int, int],
        color: tuple[int, int, int],
        radius: int,
        fill: bool,
        thickness: int = 1,
    ) -> None:
        x, y, w, h = rect
        radius = max(0, min(radius, w // 2, h // 2))
        line_type = cv2.LINE_AA
        if fill:
            cv2.rectangle(canvas, (x + radius, y), (x + w - radius, y + h), color, -1, line_type)
            cv2.rectangle(canvas, (x, y + radius), (x + w, y + h - radius), color, -1, line_type)
            cv2.circle(canvas, (x + radius, y + radius), radius, color, -1, line_type)
            cv2.circle(canvas, (x + w - radius, y + radius), radius, color, -1, line_type)
            cv2.circle(canvas, (x + radius, y + h - radius), radius, color, -1, line_type)
            cv2.circle(canvas, (x + w - radius, y + h - radius), radius, color, -1, line_type)
            return
        cv2.line(canvas, (x + radius, y), (x + w - radius, y), color, thickness, line_type)
        cv2.line(canvas, (x + radius, y + h), (x + w - radius, y + h), color, thickness, line_type)
        cv2.line(canvas, (x, y + radius), (x, y + h - radius), color, thickness, line_type)
        cv2.line(canvas, (x + w, y + radius), (x + w, y + h - radius), color, thickness, line_type)
        cv2.ellipse(canvas, (x + radius, y + radius), (radius, radius), 180, 0, 90, color, thickness, line_type)
        cv2.ellipse(canvas, (x + w - radius, y + radius), (radius, radius), 270, 0, 90, color, thickness, line_type)
        cv2.ellipse(canvas, (x + w - radius, y + h - radius), (radius, radius), 0, 0, 90, color, thickness, line_type)
        cv2.ellipse(canvas, (x + radius, y + h - radius), (radius, radius), 90, 0, 90, color, thickness, line_type)

    @staticmethod
    def put_text(
        canvas: np.ndarray,
        text: str,
        origin: tuple[int, int],
        scale: float,
        color: tuple[int, int, int],
        thickness: int = 1,
    ) -> None:
        cv2.putText(
            canvas,
            text,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )


def main() -> None:
    rclpy.init()
    node = KkiDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
