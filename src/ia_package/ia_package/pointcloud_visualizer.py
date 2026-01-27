import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

class Color:
    def __init__(self, r=0.0, g=0.0, b=0.0, a=0.0):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

    def divide_by(self, value):
        if value == 0:
            raise ValueError("Division by 0!")
        self.r /= value
        self.g /= value
        self.b /= value
    
    def to_rgb_uint32(self):
        """Convertit en format RGB packed pour PointCloud2."""
        r = int(self.r * 255) if self.r <= 1.0 else int(self.r)
        g = int(self.g * 255) if self.g <= 1.0 else int(self.g)
        b = int(self.b * 255) if self.b <= 1.0 else int(self.b)
        return (r << 16) | (g << 8) | b


class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Point:

    def __init__(self, x=0.0, y=0.0, z=0.0, rgb=0):
        self.xyz = Vector3(x, y, z)
        self.color = Color()
        self.label_name = 'unknown'
        self.set_color_from_rgb(rgb)
    
    def set_color_from_rgb(self, rgb):
        self.color.r = ((rgb >> 16) & 0xFF) / 255.0
        self.color.g = ((rgb >> 8) & 0xFF) / 255.0
        self.color.b = (rgb & 0xFF) / 255.0
        self.color.a = 1.0
    



class PointCloudManager:
    def __init__(self):
        self.points = []
    
    def load_from_ros_pointcloud(self, ros_cloud: PointCloud2):
        self.points = []
        
        try:
            for point_data in pc2.read_points(ros_cloud, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True):
                x, y, z, rgb = point_data
                point = Point(x, y, z, rgb)
                self.points.append(point)
        except Exception:
            for point_data in pc2.read_points(ros_cloud, 
                                               field_names=('x', 'y', 'z'),
                                               skip_nans=True):
                x, y, z = point_data
                point = Point(x, y, z, 0)
                self.points.append(point)
    
    def apply_segmentation_mask(self, mask, highlight_color: Color, cloud_width: int, cloud_height: int):        
        if mask is None or len(self.points) == 0:
            return []
        
        mask_height, mask_width = mask.shape[:2]
        masked_points = []
        
        for v in range(mask_height):
            for u in range(mask_width):
                if mask[v, u] > 0:  # This pixel belongs to the object
                    # Scale mask coordinates to pointcloud coordinates
                    pc_u = int(u * cloud_width / mask_width)
                    pc_u = max(0, min(pc_u, cloud_width - 1))
                    
                    pc_v = int(v * cloud_height / mask_height)
                    pc_v = max(0, min(pc_v, cloud_height - 1))
                    
                    point_index = pc_v * cloud_width + pc_u
                    
                    if point_index < len(self.points):
                        point = self.points[point_index]
                        if not (point.xyz.x == 0 and point.xyz.y == 0 and point.xyz.z == 0):
                            point.color = highlight_color
                            masked_points.append(point)
        
        return masked_points
    
    def to_ros_pointcloud2(self, frame_id: str, stamp) -> PointCloud2:
        """
        Args:
            frame_id: Frame de référence (ex: 'camera_link')
            stamp: Timestamp ROS2
        
        Returns:
            Message PointCloud2 publiable dans RViz2
        """
        header = Header()
        header.frame_id = frame_id
        header.stamp = stamp
        
        points_data = []
        for point in self.points:
            rgb_packed = point.color.to_rgb_uint32()
            points_data.append([
                point.xyz.x,
                point.xyz.y,
                point.xyz.z,
                rgb_packed
            ])
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        return pc2.create_cloud(header, fields, points_data)


# ============================================================================
# NŒUD ROS2
# ============================================================================

class VisualizerROS(Node):
    def __init__(self):
        super().__init__('point_cloud_visualizer')
        self.declare_parameter('marker_size', 0.3)
        
        self.declare_parameter('highlight_color_r', 209) # Orange vif
        self.declare_parameter('highlight_color_g', 103)
        self.declare_parameter('highlight_color_b', 42)

        self.marker_size = self.get_parameter('marker_size').value
        self.target_class = ""
        
        self.highlight_color = Color(
            self.get_parameter('highlight_color_r').value,
            self.get_parameter('highlight_color_g').value,
            self.get_parameter('highlight_color_b').value,
            1.0
        )
        
        self.pc_manager = PointCloudManager()
        self.latest_ros_cloud = None
        self.latest_mask = None
        self.position_computed = False
        
        # TF2 for transforming camera_link -> map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/depth_camera/points', self.pointcloud_callback, 10)
        
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        self.br = CvBridge()
        
        self.mask_sub = self.create_subscription(Image, '/object/segmentation_mask', self.mask_callback,10)
        
        from std_msgs.msg import String
        self.command_sub = self.create_subscription(String, '/detection/command', self.command_callback, 10)
        self.class_name_sub = self.create_subscription(String, '/object/class_name', self.class_name_callback, 10)
        
        self.marker_pub = self.create_publisher(Marker, '/object/detection_marker', 10)
        self.filtered_pc_pub = self.create_publisher(PointCloud2, '/object/pointcloud_filtered', 10)
        self.map_marker_pub = self.create_publisher(Marker, '/object/detection_marker_map', 10)
        self.position_pub = self.create_publisher(PointStamped, '/object/position', 10)
        
        self.get_logger().info('[OK] Semantic PointCloud Visualizer démarré')
        self.get_logger().info(f'   - Couleur highlight: RGB({self.highlight_color.r}, {self.highlight_color.g}, {self.highlight_color.b})')
        self.get_logger().info('En attente de détections (masque + position)...')
    
    def mask_callback(self, msg):
        try:
            self.latest_mask = self.br.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Error mask conversion: {e}')
            return
        
        if self.position_computed:
            return
        
        if self.latest_ros_cloud is None:
            return
        
        # Compute 3D position from mask centroid
        self.get_logger().info('Calculating 3D position...')
        position = self.compute_3d_from_mask()
        if position is not None:
            self.position_computed = True  # Don't recompute
            self.position_pub.publish(position)
            self.process_detection(position)
            self.get_logger().info('Position 3D calculate as soon as target detected!')
        else:
            self.get_logger().warn('Impossible to calculate the position 3D')
    
    def command_callback(self, msg):
        """Reset position flag when STOP received (mission complete)."""
        if msg.data == "STOP":
            self.position_computed = False  # Ready for next detection

    def class_name_callback(self, msg):
        new_class = msg.data
        if self.target_class != new_class:
            self.target_class = new_class
            # self.get_logger().info(f'Cible mise à jour : {self.target_class.upper()}')
    
    def pointcloud_callback(self, msg: PointCloud2):
        self.latest_ros_cloud = msg
    
    def compute_3d_from_mask(self):
        """Compute 3D position from mask centroid using median filtering."""
        if self.latest_mask is None or self.latest_ros_cloud is None:
            return None
        
        import numpy as np
        
        mask_coords = np.where(self.latest_mask > 0)
        if len(mask_coords[0]) == 0:
            return None
        
        center_v = int(np.median(mask_coords[0]))
        center_u = int(np.median(mask_coords[1]))
        
        cloud_width = self.latest_ros_cloud.width
        cloud_height = self.latest_ros_cloud.height
        mask_height, mask_width = self.latest_mask.shape[:2]
        
        pc_u = int(center_u * cloud_width / mask_width)
        pc_v = int(center_v * cloud_height / mask_height)
        
        kernel_size = 5
        half_kernel = kernel_size // 2
        
        points_gen = pc2.read_points(
            self.latest_ros_cloud, 
            field_names=('x', 'y', 'z'),
            skip_nans=False
        )
        points_list = list(points_gen)
        
        valid_points = []
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
                        valid_points.append((x, y, z))
        
        if not valid_points:
            return None
        
        median_pt = np.median(valid_points, axis=0)
        
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'camera_link'
        point_msg.point.x = float(median_pt[0])
        point_msg.point.y = float(median_pt[1])
        point_msg.point.z = float(median_pt[2])
        
        self.get_logger().info(f'[Camera frame] Position 3D: x={median_pt[0]:.2f}m, y={median_pt[1]:.2f}m, z={median_pt[2]:.2f}m')
        
        return point_msg
    
    def transform_to_map(self, point_camera: PointStamped) -> PointStamped:
        """Transform a PointStamped from camera_link to map."""
        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                point_camera.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            point_map.header.frame_id = 'map'
            
            self.get_logger().debug(
                f'Transformation réussie: {point_camera.header.frame_id} → map'
            )
            return point_map
            
        except Exception as e:
            self.get_logger().warn(
                f'Impossible de transformer vers map: {e}'
            )
            return None

    
    def process_detection(self, msg: PointStamped):
        if self.latest_ros_cloud is None:
            self.get_logger().warn('Objet détecté mais aucun PointCloud disponible!')
            return
        
        self.get_logger().info(
            f'\nOBJET DÉTECTÉ: {self.target_class.upper()}\n'
            f'   Position CAMERA: x={msg.point.x:.2f}m, y={msg.point.y:.2f}m, z={msg.point.z:.2f}m'
        )
        
        self.pc_manager.load_from_ros_pointcloud(self.latest_ros_cloud)
        cloud_width = self.latest_ros_cloud.width
        cloud_height = self.latest_ros_cloud.height
        self.get_logger().info(f'Nuage chargé: {len(self.pc_manager.points)} points ({cloud_width}x{cloud_height})')
        
        if self.latest_mask is not None:
            masked_points = self.pc_manager.apply_segmentation_mask(
                self.latest_mask,
                self.highlight_color,
                cloud_width,
                cloud_height
            )
            self.get_logger().info(f'Points segmentés par masque: {len(masked_points)}')
            
        temp_manager = PointCloudManager()
        temp_manager.points = masked_points
        
        marker = self.create_detection_marker(msg)
        self.marker_pub.publish(marker)
        self.get_logger().info('    Marker VERT publié sur /object/detection_marker (camera_link)')
        
        filtered_cloud = temp_manager.to_ros_pointcloud2(
            msg.header.frame_id,
            self.get_clock().now().to_msg()
        )
        self.filtered_pc_pub.publish(filtered_cloud)
        self.get_logger().info('    Nuage segmenté publié sur /object/pointcloud_filtered')
        
        point_map = self.transform_to_map(msg)
        if point_map is not None:
            map_marker = self.create_map_marker(point_map)
            self.map_marker_pub.publish(map_marker)
            self.get_logger().info(
                f'    Marker ROUGE publié sur /object/detection_marker_map (map)\n'
                f'      Position MAP: x={point_map.point.x:.2f}m, y={point_map.point.y:.2f}m, z={point_map.point.z:.2f}m'
            )
        else:
            self.get_logger().warn('Impossible de créer le marqueur map (transformation échouée)')


    def create_detection_marker(self, position: PointStamped) -> Marker:
        """Crée un Marker 3D (cube vert) pour RViz2."""
        marker = Marker()
        marker.header.frame_id = position.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_detection"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = position.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = self.marker_size
        marker.color.r = 0.0
        marker.color.g = 1.0  # Vert
        marker.color.b = 0.0
        marker.color.a = 0.6
        marker.lifetime.sec = 0
        return marker
    
    def create_map_marker(self, position: PointStamped) -> Marker:
        """Crée un Marker 3D (cube ROUGE) dans le repère MAP pour RViz2."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_detection_map"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = position.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = self.marker_size
        marker.color.r = 1.0  # ROUGE
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 0
        return marker



def main(args=None):
    rclpy.init(args=args)
    node = VisualizerROS()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt du nœud Semantic PointCloud Visualizer')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()