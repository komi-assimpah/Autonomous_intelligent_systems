import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import sensor_msgs_py.point_cloud2 as pc2


class Inference(Node):
    
    def __init__(self):
        super().__init__('inference')
        
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('model_path', 'yolo11n-seg.pt')
        self.declare_parameter('conf_threshold', 0.5)
        
        # Publishers
        self.publisher_ = self.create_publisher(String, '/detection/command', 10)
        self.image_pub_ = self.create_publisher(Image, '/inference/image_processed', 10)
        self.position_pub_ = self.create_publisher(PointStamped, '/object/position', 10)
        
        # Image subscription
        self.subscription = self.create_subscription(
            Image, '/processed/camera_feed', self.image_callback, 10)
        
        # PointCloud subscription for 3D position
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/depth_camera/points', self.pointcloud_callback, 10)
        
        self.br = CvBridge()

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)

        self.get_logger().info(f'Chargement du modèle: {model_path}...')
        self.model = YOLO(model_path)
        self.get_logger().info('Modèle chargé !')

        self.target_object = self.get_parameter('target_class').value
        
        # State tracking
        self.target_found = False
        self.bbox_center = None
        self.latest_pointcloud = None
        self.image_width = 640
        self.image_height = 480
        
        self.get_logger().info(f'Classe cible: {self.target_object}')

    def pointcloud_callback(self, msg):
        """Store latest pointcloud for 3D position extraction"""
        self.latest_pointcloud = msg
    
    def get_3d_position(self, u, v):
        """Extract 3D position from PointCloud at pixel (u, v)"""
        if self.latest_pointcloud is None:
            return None
        
        try:
            cloud_width = self.latest_pointcloud.width
            cloud_height = self.latest_pointcloud.height
            
            # Scale pixel coordinates to pointcloud dimensions
            pc_u = int(u * cloud_width / self.image_width)
            pc_v = int(v * cloud_height / self.image_height)
            
            # Clamp to valid range
            pc_u = max(0, min(pc_u, cloud_width - 1))
            pc_v = max(0, min(pc_v, cloud_height - 1))
            
            # Calculate point index for organized pointcloud
            point_index = pc_v * cloud_width + pc_u
            
            # Read all points and get the one we need
            points_gen = pc2.read_points(
                self.latest_pointcloud, 
                field_names=('x', 'y', 'z'),
                skip_nans=False
            )
            
            for i, point in enumerate(points_gen):
                if i == point_index:
                    x, y, z = point[0], point[1], point[2]
                    if not np.isnan(x) and not np.isnan(y) and not np.isnan(z):
                        return (float(x), float(y), float(z))
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Error extracting 3D position: {e}')
        
        return None
    
    def image_callback(self, msg):
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = cv_image.shape[:2]
        except Exception as e:
            self.get_logger().error(f'Erreur de conversion: {e}')
            return
        
        object_detected, annotated_frame, bbox = self.run_inference(cv_image)

        # Always publish annotated frame
        if annotated_frame is not None:
            processed_msg = self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_pub_.publish(processed_msg)

        # Continuous orientation: keep sending offset until object is centered
        if object_detected:
            center_x = self.image_width / 2
            offset = (bbox[0] - center_x) / center_x  # -1 to +1
            
            if not self.target_found:
                # First detection
                self.target_found = True
                self.get_logger().warn(f'🎯 {self.target_object.upper()} DÉTECTÉ !')
                self.get_logger().info(f'📍 Position 2D: ({bbox[0]}, {bbox[1]}) pixels')
            
            # Check if centered (within 10% of center)
            if abs(offset) < 0.1:
                # Object is centered - STOP
                stop_msg = String()
                stop_msg.data = "STOP"
                self.publisher_.publish(stop_msg)
                
                self.get_logger().info('🛑 Objet centré - Robot arrêté!')
                
                # Extract and publish 3D position
                pos_3d = self.get_3d_position(bbox[0], bbox[1])
                if pos_3d:
                    x, y, z = pos_3d
                    self.get_logger().info(f'📍 Position 3D: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m')
                    
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = 'camera_link'
                    point_msg.point.x = float(x)
                    point_msg.point.y = float(y)
                    point_msg.point.z = float(z)
                    self.position_pub_.publish(point_msg)
                
                # Disable further processing
                self.target_found = True
                self.subscription = None  # Unsubscribe
            else:
                # Still need to orient - send ORIENT with offset
                orient_msg = String()
                orient_msg.data = f"ORIENT:{offset:.3f}"
                self.publisher_.publish(orient_msg)
                
                direction = "gauche" if offset < 0 else "droite"
                self.get_logger().info(f'🔄 Orientation: offset={offset:.2f} → {direction}')

    def run_inference(self, frame):
        results = self.model(frame, verbose=False, conf=self.conf_threshold)
        
        detected = False
        annotated_frame = frame 
        bbox_center = (0, 0)

        for r in results:
            annotated_frame = r.plot() 

            # TODO: Uncomment for production with real 3D objects
            # if r.masks is None:
            #     continue
            
            if r.boxes is None or len(r.boxes) == 0:
                continue

            for box in r.boxes:
                cls_id = int(box.cls[0])
                current_class = self.model.names.get(cls_id, str(cls_id))

                if current_class == self.target_object:
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = xyxy
                    center_u = int((x1 + x2) / 2)
                    center_v = int((y1 + y2) / 2)
                    bbox_center = (center_u, center_v)
                    detected = True
                    break 
        
        return detected, annotated_frame, bbox_center


def main(args=None):
    rclpy.init(args=args)
    node = Inference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()