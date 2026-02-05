import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


from visualization_msgs.msg import Marker

class Inference(Node):
    
    def __init__(self):
        super().__init__('inference')
        
        self.declare_parameter('target_class', 'dog')
        self.declare_parameter('model_path', 'yolo11n-seg.pt')
        self.declare_parameter('conf_threshold', 0.5)
        
        self.publisher_ = self.create_publisher(String, '/detection/command', 10)
        self.image_pub_ = self.create_publisher(Image, '/inference/image_processed', 10)
        self.mask_pub_ = self.create_publisher(Image, '/object/segmentation_mask', 10)
        self.class_pub_ = self.create_publisher(String, '/object/class_name', 10)
        self.position_pub_ = self.create_publisher(PointStamped, '/object/position', 10)
        self.marker_pub_ = self.create_publisher(Marker, '/object/marker', 10)

        self.rgb_sub = self.create_subscription(Image, '/camera/image_decompressed', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        self.latest_depth = None
        
        self.br = CvBridge()
        
        # TF2 for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)

        self.get_logger().info(f'Chargement du modèle: {model_path}...')
        self.model = YOLO(model_path)
        self.get_logger().info('Modèle chargé !')

        self.target_object = self.get_parameter('target_class').value
        
        available_classes = list(self.model.names.values())
        if self.target_object not in available_classes:
            error_msg = (
                f"ERREUR CRITIQUE: La classe cible '{self.target_object}' est inconnue du modèle !\n"
                f"Classes disponibles ({len(available_classes)}): {available_classes}\n"
                f"-> Vérifiez l'orthographe ou changez de modèle."
            )
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
            
        self.target_found = False
        self.position_computed = False
        
        self.get_logger().info(f'Classe cible: {self.target_object}')
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.latest_depth = self.br.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def compute_3d_position(self, bbox, mask):
        """Compute 3D position from depth image and mask center."""
        if self.latest_depth is None:
            return None
        
        # Find mask centroid
        mask_coords = np.where(mask > 0)
        if len(mask_coords[0]) == 0:
            return None
        
        center_y = int(np.median(mask_coords[0]))
        center_x = int(np.median(mask_coords[1]))
        
        # Read depth value (in millimeters)
        depth_mm = self.latest_depth[center_y, center_x]
        if depth_mm == 0 or depth_mm > 10000:  # Invalid or too far
            return None
        
        z = depth_mm / 1000.0  # Convert to meters
        
        # Camera intrinsics for 424x240
        height, width = self.latest_depth.shape
        fx = 300.0
        fy = 300.0
        cx = width / 2.0
        cy = height / 2.0
        
        # Back-project to 3D
        x = (center_x - cx) * z / fx
        y = (center_y - cy) * z / fy
        
        # Create PointStamped message
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'camera_link'
        point_msg.point.x = float(x)
        point_msg.point.y = float(y)
        point_msg.point.z = float(z)
        
        return point_msg
    
    def image_callback(self, msg):
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
            image_height, image_width = cv_image.shape[:2]
        except Exception as e:
            self.get_logger().error(f'Erreur de conversion: {e}')
            return
        
        object_detected, annotated_frame, bbox, mask = self.run_inference(cv_image)

        if annotated_frame is not None:
            processed_msg = self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_pub_.publish(processed_msg)

        if object_detected:
            if not self.target_found:
                self.target_found = True
                self.get_logger().warn(f'{self.target_object.upper()} DÉTECTÉ !')
                
                stop_msg = String()
                stop_msg.data = "TARGET_FOUND"
                self.publisher_.publish(stop_msg)
            
            if mask is not None:
                class_msg = String()
                class_msg.data = self.target_object
                self.class_pub_.publish(class_msg)

                mask_msg = self.br.cv2_to_imgmsg(mask, encoding="mono8")
                mask_msg.header.stamp = self.get_clock().now().to_msg()
                mask_msg.header.frame_id = 'camera_link'
                self.mask_pub_.publish(mask_msg)
                
                # Compute 3D position once
                if not self.position_computed:
                    position_3d_camera = self.compute_3d_position(bbox, mask)
                    if position_3d_camera is not None:
                        # Transform to map frame
                        position_3d_map = self.transform_to_map(position_3d_camera)
                        if position_3d_map is not None:
                            self.position_pub_.publish(position_3d_camera)  # Still publish camera frame for navigation
                            self.publish_marker(position_3d_map)  # Marker in map frame
                            self.position_computed = True
                            self.get_logger().info(
                                f'Position 3D (camera): x={position_3d_camera.point.x:.2f}m, '
                                f'y={position_3d_camera.point.y:.2f}m, z={position_3d_camera.point.z:.2f}m'
                            )
                            self.get_logger().info(
                                f'Position 3D (map): x={position_3d_map.point.x:.2f}m, '
                                f'y={position_3d_map.point.y:.2f}m, z={position_3d_map.point.z:.2f}m'
                            )

    def transform_to_map(self, point_camera):
        """Transform PointStamped from camera_link to map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                point_camera.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Transform the point
            point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            return point_map
            
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')
            return None
    
    def publish_marker(self, point_msg):
        marker = Marker()
        marker.header = point_msg.header
        marker.ns = "object_detection"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point_msg.point.x
        marker.pose.position.y = point_msg.point.y
        marker.pose.position.z = point_msg.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Blue
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg() # Forever
        self.marker_pub_.publish(marker)

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
                        mask_data = r.masks.data[idx].cpu().numpy()
                        target_mask = cv2.resize(
                            mask_data, 
                            (frame.shape[1], frame.shape[0]),
                            interpolation=cv2.INTER_NEAREST
                        )
                        target_mask = (target_mask > 0.5).astype(np.uint8) * 255
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