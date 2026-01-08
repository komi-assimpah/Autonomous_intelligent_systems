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
        self.target_position_pub_ = self.create_publisher(PointStamped, '/object/position', 10)
        
        # NEW: Publishers for segmentation mask and class name
        self.mask_pub_ = self.create_publisher(Image, '/object/segmentation_mask', 10)
        self.class_pub_ = self.create_publisher(String, '/object/class_name', 10)


        
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
        
        object_detected, annotated_frame, bbox, mask = self.run_inference(cv_image)

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
            
            # Publish segmentation mask if available
            if mask is not None:
                mask_msg = self.br.cv2_to_imgmsg(mask, encoding="mono8")
                mask_msg.header.stamp = self.get_clock().now().to_msg()
                mask_msg.header.frame_id = 'camera_link'
                self.mask_pub_.publish(mask_msg)
                
                # Publish class name
                class_msg = String()
                class_msg.data = self.target_object
                self.class_pub_.publish(class_msg)
            
            # Check if centered (within 10% of center)
            if abs(offset) < 0.1:
                # Object is centered - STOP
                stop_msg = String()
                stop_msg.data = "STOP"
                self.publisher_.publish(stop_msg)
                
                self.get_logger().info('🛑 Objet centré - Robot arrêté!')
                
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
                    self.target_position_pub_.publish(point_msg)
            else:
                orient_msg = String()
                orient_msg.data = f"ORIENT:{offset:.3f}"
                self.publisher_.publish(orient_msg)
                
                direction = "gauche" if offset < 0 else "droite"
                self.get_logger().info(f'🔄 Orientation: offset={offset:.2f} → {direction}')

    def run_inference(self, frame):
        """
        Run YOLO segmentation inference on frame.
        
        Returns:
            detected: bool - True if target object found
            annotated_frame: np.array - Frame with annotations
            bbox_center: tuple(int, int) - Center of bounding box (u, v)
            mask: np.array or None - Binary segmentation mask for target object
        """
        results = self.model(frame, verbose=False, conf=self.conf_threshold)
        
        detected = False
        annotated_frame = frame 
        bbox_center = (0, 0)
        target_mask = None

        for r in results:
            annotated_frame = r.plot() 
            
            if r.boxes is None or len(r.boxes) == 0:
                continue

            for idx, box in enumerate(r.boxes):
                cls_id = int(box.cls[0])
                current_class = self.model.names.get(cls_id, str(cls_id))

                if current_class == self.target_object:
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = xyxy
                    center_u = int((x1 + x2) / 2)
                    center_v = int((y1 + y2) / 2)
                    bbox_center = (center_u, center_v)
                    detected = True
                    
                    # Extract segmentation mask for this object
                    if r.masks is not None and idx < len(r.masks):
                        # Get mask data (normalized 0-1, same size as inference)
                        mask_data = r.masks.data[idx].cpu().numpy()
                        # Resize to original image size
                        target_mask = cv2.resize(
                            mask_data, 
                            (frame.shape[1], frame.shape[0]),
                            interpolation=cv2.INTER_NEAREST
                        )
                        # Convert to binary uint8 (0 or 255)
                        target_mask = (target_mask > 0.5).astype(np.uint8) * 255
                        
                        self.get_logger().info(
                            f'🎭 Masque segmentation: {np.sum(target_mask > 0)} pixels de {current_class}'
                        )
                    break 
        
        return detected, annotated_frame, bbox_center, target_mask


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