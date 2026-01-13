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
        
        self.declare_parameter('target_class', 'dog')
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
        """Extract 3D position from PointCloud at pixel (u, v) with neighborhood search"""
        if self.latest_pointcloud is None:
            return None
        
        try:
            cloud_width = self.latest_pointcloud.width
            cloud_height = self.latest_pointcloud.height
            
            # Scale pixel coordinates
            pc_u = int(u * cloud_width / self.image_width)
            pc_v = int(v * cloud_height / self.image_height)
            
            # Define search kernel (e.g., 5x5 window) to find valid depth
            kernel_size = 5
            half_kernel = kernel_size // 2
            
            valid_points = []
            
            for dv in range(-half_kernel, half_kernel + 1):
                for du in range(-half_kernel, half_kernel + 1):
                    uu = max(0, min(pc_u + du, cloud_width - 1))
                    vv = max(0, min(pc_v + dv, cloud_height - 1))
                    
                    point_index = vv * cloud_width + uu
                    
                    # We need to act quickly, so we read just this point generator-style or full read?
                    # pc2.read_points yields generators. Doing this in nested loop might be slow if we read full cloud each time.
                    # Optimization: Read a small crop? or just iterate generator?
                    # Better: Read all points once? No, too slow (300k points).
                    # Actually read_points with uvs is not supported directly in standard read_points unless we pass uvs.
                    # But sensor_msgs_py.point_cloud2.read_points_numpy is faster if available.
                    # Let's keep it simple: Read just the specific index is tricky with read_points as it iterates.
                    # Wait, read_points yields ALL points. We can't index directly efficiently without reading all previous.
                    # Optimization: Use read_points_numpy results if possible, or just be careful.
                    # Actually: The previous code was iterating `enumerate(points_gen)` until index.
                    # That is VERY slow if index is late.
                    pass
            
            # Efficient approach: Read all points once into a numpy array (structured).
            # This is much faster for lookup.
            # However, converting 300k points might be heavy at 30Hz.
            # Let's try to just be robust with the single point first, or check the loop logic. 
            # The previous code: `for i, point in enumerate(points_gen): if i == point_index: ...`
            # This is O(N) for every pixel lookup! And since we only do it ONCE when stopped, maybe acceptable.
            
            # Let's use read_points with uvs option? No, standard read_points doesn't have it.
            # But we can assume organized cloud.
            
            # Let's just fix the infinite/nan check for now, and maybe average a few points if possible.
            # Since we only call this when robot stops (rarely), we can afford a bit of cost.
            
            points_gen = pc2.read_points(
                self.latest_pointcloud, 
                field_names=('x', 'y', 'z'),
                skip_nans=False
            )
             
            # Optimization: We only want points near (pc_u, pc_v).
            # Reading the whole generator to find one index is inefficient but let's stick to logic that works.
            # To search neighborhood, we need multiple points.
            
            # Better: convert full cloud to numpy ONLY if we need (when checking).
            # It might be 50ms.
            
            # Let's try a simpler robust check:
            # We want (x,y,z) at `point_index`.
            # Note: `read_points` iterates in row-major order.
            
            # We can skip to the approximate location? No.
            
            # Let's rewrite to use `read_points_list` which reads all.
            points_list = list(points_gen) # heavy?
            
            # Window search
            target_depths = []
            
            for dv in range(-half_kernel, half_kernel + 1):
                for du in range(-half_kernel, half_kernel + 1):
                    uu = max(0, min(pc_u + du, cloud_width - 1))
                    vv = max(0, min(pc_v + dv, cloud_height - 1))
                    idx = vv * cloud_width + uu
                    
                    if idx < len(points_list):
                        x, y, z = points_list[idx]
                        if not np.isnan(x) and not np.isinf(x) and \
                           not np.isnan(y) and not np.isinf(y) and \
                           not np.isnan(z) and not np.isinf(z):
                             target_depths.append((x, y, z))

            if target_depths:
                # Return median or average to be robust
                median_pt = np.median(target_depths, axis=0)
                return tuple(median_pt)
                    
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
                orient_msg.data = f"CADRAGE:{offset:.3f}"
                self.publisher_.publish(orient_msg)
                
                direction = "gauche" if offset < 0 else "droite"
                self.get_logger().info(f'🔄 Cadrage: offset={offset:.2f} → {direction}')

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