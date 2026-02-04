#!/usr/bin/env python3
"""
Image Decompressor Node
Receives compressed images from the robot and decompresses them for YOLO inference.
Runs on the PC/VM side.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageDecompressorNode(Node):
    def __init__(self):
        super().__init__('image_decompressor_node')
        self.get_logger().info('====== Image Decompressor Node started =======')

        # Parameters
        self.declare_parameter('image_topic', '/camera/rgb/image_compressed')
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'Robot Camera Feed')
        self.declare_parameter('resize_width', 0)  # 0 = no resize
        self.declare_parameter('resize_height', 0)

        image_topic = self.get_parameter('image_topic').value
        self.show_window = self.get_parameter('show_window').value
        self.window_name = self.get_parameter('window_name').value
        self.resize_w = self.get_parameter('resize_width').value
        self.resize_h = self.get_parameter('resize_height').value

        self.bridge = CvBridge()
        self.frame_count = 0
        self.decompression_errors = 0

        # QoS for compressed images (BEST_EFFORT for Wi-Fi streaming)
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Subscribe to compressed images from robot (C++ RealSense publisher)
        self.compressed_sub = self.create_subscription(
            CompressedImage,
            image_topic,
            self.compressed_callback,
            qos_sensor
        )

        # Publish decompressed images for YOLO
        self.decompressed_pub = self.create_publisher(
            Image,
            '/camera/image_decompressed',
            10
        )

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(f'Subscribed to: {image_topic}')
        self.get_logger().info(f'Display window: {self.show_window}')
        self.get_logger().info('Waiting for compressed images from robot...')

    def compressed_callback(self, msg: CompressedImage):
        try:
            # Decode JPEG (simple and efficient)
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR

            if frame is None:
                raise RuntimeError("Failed to decode JPEG image")

            # Optional resize (to reduce CPU load)
            if self.resize_w > 0 and self.resize_h > 0:
                frame = cv2.resize(frame, (self.resize_w, self.resize_h), 
                                 interpolation=cv2.INTER_AREA)

            # Display window for debugging
            if self.show_window:
                cv2.imshow(self.window_name, frame)
                cv2.waitKey(1)

            # Convert to ROS Image message for YOLO
            decompressed_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            decompressed_msg.header = msg.header  # Preserve timestamp

            self.decompressed_pub.publish(decompressed_msg)

            self.frame_count += 1
            if self.frame_count % 50 == 0:
                self.get_logger().info(
                    f'Decompressed {self.frame_count} frames | '
                    f'Size: {len(msg.data)/1024:.1f} KB | '
                    f'Resolution: {frame.shape[1]}x{frame.shape[0]}'
                )

        except Exception as e:
            self.decompression_errors += 1
            if self.decompression_errors % 10 == 1:
                self.get_logger().error(f'Error decompressing image: {e}')

    def __del__(self):
        if self.show_window:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ImageDecompressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
