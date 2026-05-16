import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class QrDetector(Node):
    def __init__(self) -> None:
        super().__init__("qr_detector")
        self.declare_parameter("image_topic", "/rov/camera/wall/image")
        self.declare_parameter("debug_image_topic", "/rov/qr_debug/image")

        image_topic = str(self.get_parameter("image_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.last_code = ""

        self.code_pub = self.create_publisher(String, "/rov/qr_code", 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"Listening for QR codes on {image_topic}")

    def image_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return

        codes, points = self.detect_codes(image)
        for code in codes:
            if code:
                self.code_pub.publish(String(data=code))
                if code != self.last_code:
                    self.get_logger().info(f"QR detected: {code}")
                    self.last_code = code

        if points is not None:
            debug = self.draw_points(image.copy(), points)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    def detect_codes(self, image):
        codes = []
        all_points = None
        try:
            ok, decoded, points, _ = self.detector.detectAndDecodeMulti(image)
            if ok:
                codes.extend([code for code in decoded if code])
                all_points = points
        except Exception:
            all_points = None

        if not codes:
            code, points, _ = self.detector.detectAndDecode(image)
            if code:
                codes.append(code)
                all_points = points
        return codes, all_points

    @staticmethod
    def draw_points(image, points):
        if points is None:
            return image
        for quad in points.reshape(-1, 4, 2):
            for i in range(4):
                p1 = tuple(int(v) for v in quad[i])
                p2 = tuple(int(v) for v in quad[(i + 1) % 4])
                cv2.line(image, p1, p2, (0, 255, 0), 2)
        return image


def main() -> None:
    rclpy.init()
    node = QrDetector()
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
