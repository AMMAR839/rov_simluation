import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


VALID_PAYLOAD_CODES = {"A", "B", "C", "D"}


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

        detections = self.detect_codes(image)
        selected = self.select_best_detection(detections, image.shape)
        if selected is not None:
            code, _ = selected
            self.code_pub.publish(String(data=code))
            if code != self.last_code:
                self.get_logger().info(f"QR detected: {code}")
                self.last_code = code

        if detections:
            debug = self.draw_detections(image.copy(), detections, selected)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    def detect_codes(self, image):
        detections = []
        try:
            ok, decoded, points, _ = self.detector.detectAndDecodeMulti(image)
            if ok and points is not None:
                for code, quad in zip(decoded, points.reshape(-1, 4, 2)):
                    normalized = self.normalize_code(code)
                    if normalized:
                        detections.append((normalized, quad))
        except Exception:
            detections = []

        if not detections:
            code, points, _ = self.detector.detectAndDecode(image)
            normalized = self.normalize_code(code)
            if normalized and points is not None:
                detections.append((normalized, points.reshape(4, 2)))
        return detections

    @staticmethod
    def normalize_code(code):
        normalized = str(code).strip().upper()
        if normalized in VALID_PAYLOAD_CODES:
            return normalized
        return ""

    @staticmethod
    def select_best_detection(detections, image_shape):
        if not detections:
            return None

        height, width = image_shape[:2]
        image_center = (width * 0.5, height * 0.5)
        diagonal = max(1.0, (width * width + height * height) ** 0.5)

        def score(detection):
            _, quad = detection
            area = abs(cv2.contourArea(quad.astype("float32")))
            center_x = float(quad[:, 0].mean())
            center_y = float(quad[:, 1].mean())
            center_error = (
                (center_x - image_center[0]) ** 2
                + (center_y - image_center[1]) ** 2
            ) ** 0.5
            center_weight = 1.0 - min(0.40, 0.40 * center_error / diagonal)
            return area * center_weight

        return max(detections, key=score)

    @staticmethod
    def draw_detections(image, detections, selected):
        selected_quad = selected[1] if selected is not None else None
        for code, quad in detections:
            is_selected = selected_quad is not None and (quad == selected_quad).all()
            color = (0, 255, 0) if is_selected else (0, 200, 255)
            for i in range(4):
                p1 = tuple(int(v) for v in quad[i])
                p2 = tuple(int(v) for v in quad[(i + 1) % 4])
                cv2.line(image, p1, p2, color, 2)
            label_position = tuple(int(v) for v in quad[0])
            cv2.putText(
                image,
                code,
                label_position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )
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
