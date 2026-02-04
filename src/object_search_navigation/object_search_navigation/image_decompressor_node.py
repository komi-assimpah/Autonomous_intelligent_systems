
from sympy import true
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
import time

class ImageDecompressorNode(Node):
    def __init__(self):
        super().__init__('image_decompressor')
        
        self.declare_parameter('image_topic', '/camera/rgbd/compressed')
        self.declare_parameter('output_rgb_topic', '/camera/image_decompressed')
        self.declare_parameter('output_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('show_window', True)
        
        self.image_topic = self.get_parameter('image_topic').value
        self.output_rgb_topic = self.get_parameter('output_rgb_topic').value
        self.output_depth_topic = self.get_parameter('output_depth_topic').value
        self.show_window = self.get_parameter('show_window').value
        
        # Use SensorDataQoS (BEST_EFFORT) to match robot publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.listener_callback,
            qos)
            
        self.rgb_pub = self.create_publisher(Image, self.output_rgb_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.output_depth_topic, 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        
        self.br = CvBridge()
        self.frame_count = 0
        self.total_bytes = 0
        self.start_time = time.time()
        
        self.last_depth_image = None
        
        self.get_logger().info(f'✅ RGB-D Decompressor started on {self.image_topic}')

    def listener_callback(self, msg):
        try:
            format_str = msg.format
            data = msg.data
            
            rgb_image = None
            depth_image = None
            
            # --- CASE 1: Custom Packed RGB-D ---
            if "rgbd" in format_str:
                # Header: [rgb_size (4)][depth_size (4)]
                offset = 0
                rgb_size = struct.unpack('<I', bytes(data[offset:offset+4]))[0]
                offset += 4
                depth_size = struct.unpack('<I', bytes(data[offset:offset+4]))[0]
                offset += 4
                
                # RGB Data
                rgb_data = np.frombuffer(data[offset:offset+rgb_size], np.uint8)
                rgb_image = cv2.imdecode(rgb_data, cv2.IMREAD_COLOR)
                offset += rgb_size
                
                # Depth Data
                depth_data = np.frombuffer(data[offset:offset+depth_size], np.uint8)
                depth_image = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED) # 16-bit PNG
                
                self.frame_count += 1
                self.total_bytes += len(data)
                
            # --- CASE 2: Standard Compressed Image (Fallback) ---
            elif "jpeg" in format_str or "jpg" in format_str:
                np_arr = np.frombuffer(data, np.uint8)
                rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                self.frame_count += 1
            
            # --- Publish RGB ---
            if rgb_image is not None:
                img_msg = self.br.cv2_to_imgmsg(rgb_image, encoding="bgr8")
                img_msg.header = msg.header
                self.rgb_pub.publish(img_msg)
                
                # Show window
                if self.show_window:
                    display_img = rgb_image.copy()
                    
                    if depth_image is not None:
                        # Visualize depth overlay (normalize 0-2m to 0-255)
                        depth_vis = cv2.convertScaleAbs(depth_image, alpha=0.03) # Alpha depends on range
                        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                        
                        # Concatenate side by side for vis
                        # Resize depth to match rgb if needed
                        if depth_vis.shape[:2] == display_img.shape[:2]:
                            display_img = np.hstack((display_img, depth_vis))
                            
                    cv2.imshow("Robot Stream (RGB + Depth)", display_img)
                    cv2.waitKey(1)

            # --- Publish Depth ---
            if depth_image is not None:
                depth_msg = self.br.cv2_to_imgmsg(depth_image, encoding="mono16")
                depth_msg.header = msg.header
                depth_msg.header.frame_id = "camera_depth_frame"
                self.depth_pub.publish(depth_msg)
                
                # Publish camera info
                self.publish_camera_info(depth_msg.header, depth_image.shape[1], depth_image.shape[0])

            # Stats
            if self.frame_count % 100 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                avg_size = self.total_bytes / self.frame_count / 1024.0 # KB
                self.get_logger().info(f"FPS: {fps:.1f} | Avg Size: {avg_size:.1f} KB")

        except Exception as e:
            self.get_logger().error(f'Decompression failed: {e}')

    def publish_camera_info(self, header, width, height):
        # Publish dummy Intrinsics for D435i (Approximate)
        # to allow pointcloud generation on PC without real camera info
        # Ideal: Send camera info from robot, but bandwidth...
        msg = CameraInfo()
        msg.header = header
        msg.height = height
        msg.width = width
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Approximate intrinsics for 424x240
        fx = 300.0
        fy = 300.0
        cx = width / 2.0
        cy = height / 2.0
        
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.cam_info_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    image_decompressor_node = ImageDecompressorNode()
    rclpy.spin(image_decompressor_node)
    image_decompressor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
