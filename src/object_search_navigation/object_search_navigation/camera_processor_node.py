import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraProcessorNode(Node):
    def __init__(self):
        super().__init__('camera_processor_node')
        self.get_logger().info('====== Camera Processor Node started =======')

        self.bridge = CvBridge()

        # Configuration parameters
        self.declare_parameter('target_width', 640)
        self.declare_parameter('target_height', 480)
        self.declare_parameter('max_fps', 10.0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('apply_clahe', False)
        self.declare_parameter('apply_denoise', False)

        self.target_width = self.get_parameter('target_width').value
        self.target_height = self.get_parameter('target_height').value
        self.max_fps = self.get_parameter('max_fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.apply_clahe = self.get_parameter('apply_clahe').value
        self.apply_denoise = self.get_parameter('apply_denoise').value

        self.last_pub_time = self.get_clock().now()
        self.frame_count = 0
        self.processing_errors = 0

        # QoS for camera topics (best effort for low latency)
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_sensor
        )

        # Publish compressed images for network efficiency
        self.processed_pub = self.create_publisher(
            CompressedImage,
            '/processed/camera_feed/compressed',
            10
        )

        self.get_logger().info(f'Camera processor configured: {self.target_width}x{self.target_height} @ {self.max_fps} FPS, JPEG quality: {self.jpeg_quality}')

    def image_callback(self, msg: Image):
        try:
            # FPS limiting to reduce network load
            now = self.get_clock().now()
            dt = (now - self.last_pub_time).nanoseconds / 1e9
            if dt < (1.0 / self.max_fps):
                return

            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess image
            processed = self.preprocess_image(cv_image)

            # JPEG compression
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
            success, jpg_buffer = cv2.imencode('.jpg', processed, encode_params)
            if not success:
                raise RuntimeError("JPEG encoding failed")

            # Create CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = jpg_buffer.tobytes()

            self.processed_pub.publish(compressed_msg)

            self.last_pub_time = now
            self.frame_count += 1
            
            if self.frame_count % 50 == 0:
                compression_ratio = len(msg.data) / len(compressed_msg.data) if len(compressed_msg.data) > 0 else 0
                self.get_logger().info(
                    f'Processed {self.frame_count} frames | '
                    f'Compression: {compression_ratio:.1f}x | '
                    f'Size: {len(compressed_msg.data)/1024:.1f} KB'
                )

        except Exception as e:
            self.processing_errors += 1
            if self.processing_errors % 10 == 1:  # Log every 10th error to avoid spam
                self.get_logger().error(f'Error processing image: {e}')

    def preprocess_image(self, image):
        """Preprocess image: resize, enhance contrast, denoise"""
        h, w = image.shape[:2]
        
        # Resize if needed
        if (w != self.target_width) or (h != self.target_height):
            image = cv2.resize(
                image, 
                (self.target_width, self.target_height), 
                interpolation=cv2.INTER_LINEAR
            )

        # CLAHE for contrast enhancement
        if self.apply_clahe:
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l = clahe.apply(l)
            lab = cv2.merge([l, a, b])
            image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        # Denoising (expensive, disabled by default)
        if self.apply_denoise:
            image = cv2.fastNlMeansDenoisingColored(image, None, 10, 10, 7, 21)

        return image


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
