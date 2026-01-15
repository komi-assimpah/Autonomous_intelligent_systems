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
    SEMANTIC_CLASSES = [
        {'name': 'background',   'color': Color(255, 102, 102, 1.0)},
        {'name': 'aeroplane',    'color': Color(255, 178, 102, 1.0)},
        {'name': 'bicycle',      'color': Color(255, 255, 102, 1.0)},
        {'name': 'bird',         'color': Color(178, 255, 102, 1.0)},
        {'name': 'boat',         'color': Color(102, 255, 102, 1.0)},
        {'name': 'bottle',       'color': Color(255, 0, 0, 1.0)},
        {'name': 'bus',          'color': Color(102, 255, 255, 1.0)},
        {'name': 'car',          'color': Color(102, 178, 255, 1.0)},
        {'name': 'cat',          'color': Color(102, 102, 255, 1.0)},
        {'name': 'chair',        'color': Color(124, 252, 0, 1.0)},
        {'name': 'cow',          'color': Color(255, 102, 255, 1.0)},
        {'name': 'diningtable',  'color': Color(255, 102, 178, 1.0)},
        {'name': 'dog',          'color': Color(209, 103, 42, 1.0)},   # Orange
        {'name': 'horse',        'color': Color(160, 82, 45, 1.0)},
        {'name': 'motorbike',    'color': Color(188, 143, 143, 1.0)},
        {'name': 'person',       'color': Color(0, 0, 255, 1.0)},      # Bleu
        {'name': 'pottedplant',  'color': Color(0, 250, 154, 1.0)},
        {'name': 'sheep',        'color': Color(255, 250, 205, 1.0)},
        {'name': 'sofa',         'color': Color(210, 180, 140, 1.0)},
        {'name': 'train',        'color': Color(0, 0, 0, 1.0)},
        {'name': 'tvmonitor',    'color': Color(30, 144, 255, 1.0)},
    ]
    
    def __init__(self, x=0.0, y=0.0, z=0.0, rgb=0):
        self.xyz = Vector3(x, y, z)
        self.color = Color()
        self.label_name = 'unknown'
        self.label_index = -1
        
        # Extraire RGB du format packed uint32
        self.set_color_from_rgb(rgb)
    
    def set_color_from_rgb(self, rgb):
        """Définit la couleur à partir d'un uint32 RGB packed."""
        self.color.r = ((rgb >> 16) & 0xFF) / 255.0
        self.color.g = ((rgb >> 8) & 0xFF) / 255.0
        self.color.b = (rgb & 0xFF) / 255.0
        self.color.a = 1.0
    
    def set_semantic_class(self, class_name):
        """
        Applique une classe sémantique au point.
        Change la couleur selon la classe (Dog → orange, Person → bleu, etc.)
        """
        class_name_lower = class_name.lower()
        for idx, cls in enumerate(self.SEMANTIC_CLASSES):
            if cls['name'] == class_name_lower:
                self.label_name = class_name_lower
                self.label_index = idx
                self.color = cls['color']
                return True
        return False
    
    def matches_target(self, target_class):
        """Vérifie si ce point appartient à la classe cible."""
        return self.label_name == target_class.lower()


class PointCloudManager:
    """
    Gère un nuage de points avec support sémantique.
    Permet de filtrer, colorier et manipuler les points selon leur classe.
    """
    
    def __init__(self):
        self.points = []
    
    def load_from_ros_pointcloud(self, ros_cloud: PointCloud2):
        """
        Charge les points depuis un message ROS2 PointCloud2.
        
        Args:
            ros_cloud: Message sensor_msgs/PointCloud2
        """
        self.points = []
        
        try:
            for point_data in pc2.read_points(ros_cloud, 
                                               field_names=('x', 'y', 'z', 'rgb'),
                                               skip_nans=True):
                x, y, z, rgb = point_data
                point = Point(x, y, z, rgb)
                self.points.append(point)
        except Exception:
            # Si pas de champ RGB, essayer sans
            for point_data in pc2.read_points(ros_cloud, 
                                               field_names=('x', 'y', 'z'),
                                               skip_nans=True):
                x, y, z = point_data
                point = Point(x, y, z, 0)
                self.points.append(point)
    
    def filter_around_position(self, target_pos: Vector3, radius: float):
        """
        Filtre les points dans un rayon donné autour d'une position.
        
        Args:
            target_pos: Position centrale (Vector3)
            radius: Rayon de filtrage en mètres
        
        Returns:
            Liste de Point dans le rayon
        """
        filtered = []
        for point in self.points:
            dist = np.sqrt(
                (point.xyz.x - target_pos.x)**2 +
                (point.xyz.y - target_pos.y)**2 +
                (point.xyz.z - target_pos.z)**2
            )
            if dist < radius:
                filtered.append(point)
        return filtered
    
    def filter_by_class(self, target_class: str):
        """
        Filtre les points appartenant à une classe sémantique.
        
        Args:
            target_class: Nom de la classe (ex: 'person', 'dog')
        
        Returns:
            Liste de Point de cette classe
        """
        return [p for p in self.points if p.matches_target(target_class)]
    
    def colorize_target_class(self, target_class: str, highlight_color: Color):
        """
        Change la couleur de tous les points d'une classe donnée.
        Utile pour highlighter l'objet cherché en rouge vif par exemple.
        
        Args:
            target_class: Classe à colorier
            highlight_color: Couleur de surbrillance
        """
        for point in self.points:
            if point.matches_target(target_class):
                point.color = highlight_color
    
    def apply_segmentation_mask(self, mask, highlight_color: Color, cloud_width: int, cloud_height: int):
        """
        Apply segmentation mask to colorize points belonging to the detected object.
        
        The mask is a 2D image where non-zero pixels belong to the target object.
        We project these 2D coordinates to find corresponding 3D points in the organized pointcloud.
        
        Args:
            mask: np.array - Binary mask from YOLO (HxW, values 0 or 255)
            highlight_color: Color to apply to masked points
            cloud_width: Width of the organized pointcloud
            cloud_height: Height of the organized pointcloud
        
        Returns:
            List of Point that belong to the masked object
        """
        import numpy as np
        
        if mask is None or len(self.points) == 0:
            return []
        
        mask_height, mask_width = mask.shape[:2]
        masked_points = []
        
        # Create a mapping from pointcloud index to point
        # For organized pointcloud: index = v * width + u
        for v in range(mask_height):
            for u in range(mask_width):
                if mask[v, u] > 0:  # This pixel belongs to the object
                    # Scale mask coordinates to pointcloud coordinates
                    pc_u = int(u * cloud_width / mask_width)
                    pc_v = int(v * cloud_height / mask_height)
                    
                    # Clamp to valid range
                    pc_u = max(0, min(pc_u, cloud_width - 1))
                    pc_v = max(0, min(pc_v, cloud_height - 1))
                    
                    # Calculate point index in organized pointcloud
                    point_index = pc_v * cloud_width + pc_u
                    
                    if point_index < len(self.points):
                        point = self.points[point_index]
                        # Check for valid point (not NaN)
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

class SemanticPointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('semantic_pointcloud_visualizer')
        
        self.declare_parameter('filter_radius', 0.5)
        self.declare_parameter('marker_size', 0.3)
        self.declare_parameter('target_class', 'Dog')
        self.declare_parameter('highlight_color_r', 209) # Orange vif
        self.declare_parameter('highlight_color_g', 103)
        self.declare_parameter('highlight_color_b', 42)
        
        self.filter_radius = self.get_parameter('filter_radius').value
        self.marker_size = self.get_parameter('marker_size').value
        self.target_class = self.get_parameter('target_class').value
        
        self.highlight_color = Color(
            self.get_parameter('highlight_color_r').value,
            self.get_parameter('highlight_color_g').value,
            self.get_parameter('highlight_color_b').value,
            1.0
        )
        
        self.pc_manager = PointCloudManager()
        self.latest_ros_cloud = None
        self.latest_mask = None
        self.position_computed = False  # Flag to compute 3D only once per detection
        
        # TF2 for transforming camera_link -> map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # === SUBSCRIBERS ===
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.pointcloud_callback,
            10
        )
        
        # Subscribe to segmentation mask from inference.py
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        self.br = CvBridge()
        
        self.mask_sub = self.create_subscription(
            Image,
            '/object/segmentation_mask',
            self.mask_callback,
            10
        )
        
        # Subscribe to detection command to trigger 3D computation only on TARGET_FOUND
        from std_msgs.msg import String
        self.command_sub = self.create_subscription(
            String,
            '/detection/command',
            self.command_callback,
            10
        )
        
        # === PUBLISHERS ===
        self.marker_pub = self.create_publisher(
            Marker,
            '/object/detection_marker',
            10
        )
        
        self.filtered_pc_pub = self.create_publisher(
            PointCloud2,
            '/object/pointcloud_filtered',
            10
        )
        
        # Publisher for RED marker in MAP frame
        self.map_marker_pub = self.create_publisher(
            Marker,
            '/object/detection_marker_map',
            10
        )
        
        # Publisher for 3D position (computed here now)
        self.position_pub = self.create_publisher(
            PointStamped,
            '/object/position',
            10
        )
        
        self.get_logger().info('[OK] Semantic PointCloud Visualizer démarré')
        self.get_logger().info(f'   - TARGET: {self.target_class}')
        self.get_logger().info(f'   - Rayon filtrage: {self.filter_radius}m')
        self.get_logger().info(f'   - Couleur highlight: RGB({self.highlight_color.r}, {self.highlight_color.g}, {self.highlight_color.b})')
        self.get_logger().info('En attente de détections (masque + position)...')
    
    def mask_callback(self, msg):
        """On first mask: compute 3D and show markers. Then skip."""
        try:
            self.latest_mask = self.br.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Erreur conversion masque: {e}')
            return
        
        # Only compute once per detection session
        if self.position_computed:
            return
        
        if self.latest_ros_cloud is None:
            return
        
        # Compute 3D position from mask centroid
        position = self.compute_3d_from_mask()
        if position is not None:
            self.position_computed = True  # Don't recompute
            self.position_pub.publish(position)
            self.process_detection(position)
            self.get_logger().info('Position 3D calculée dès la première détection!')
    
    def command_callback(self, msg):
        """Reset position flag when STOP received (mission complete)."""
        if msg.data == "STOP":
            self.position_computed = False  # Ready for next detection
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Stocke le dernier nuage de points."""
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
        
        # Scale to pointcloud coordinates
        pc_u = int(center_u * cloud_width / mask_width)
        pc_v = int(center_v * cloud_height / mask_height)
        
        # Read points with median window (5x5)
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
        """
        Transforme un PointStamped de camera_link vers map.
        
        Args:
            point_camera: Point dans le repère camera_link
        
        Returns:
            Point transformé dans le repère map, ou None si transformation impossible
        """
        try:
            # Use Time() (time=0) to get the LATEST available transform
            # instead of a specific timestamp that may not exist yet
            transform = self.tf_buffer.lookup_transform(
                'map',
                point_camera.header.frame_id,
                rclpy.time.Time(),  # Use latest available
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Transformer le point
            point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            
            # Forcer explicitement le frame_id à 'map' pour être sûr
            point_map.header.frame_id = 'map'
            
            self.get_logger().debug(
                f'Transformation réussie: {point_camera.header.frame_id} → map'
            )
            return point_map
            
        except Exception as e:
            self.get_logger().warn(
                f'⚠️  Impossible de transformer vers map: {e}'
            )
            return None

    
    def process_detection(self, msg: PointStamped):
        """
        Process a detection: colorize points, publish markers.
        Called internally when mask is received and 3D position computed.
        
        Actions:
        1. Charge le PointCloud dans le manager
        2. Applique le masque de segmentation pour colorier les points de l'objet
        3. Publie Marker + nuage coloré dans RViz2
        """
        if self.latest_ros_cloud is None:
            self.get_logger().warn('⚠️  Objet détecté mais aucun PointCloud disponible!')
            return
        
        # Log détaillé
        self.get_logger().info(
            f'\n✅ OBJET DÉTECTÉ: {self.target_class.upper()}\n'
            f'   📍 Position: x={msg.point.x:.2f}m, y={msg.point.y:.2f}m, z={msg.point.z:.2f}m'
        )
        
        # 1. Charger le nuage dans le manager
        self.pc_manager.load_from_ros_pointcloud(self.latest_ros_cloud)
        cloud_width = self.latest_ros_cloud.width
        cloud_height = self.latest_ros_cloud.height
        self.get_logger().info(f'   📊 Nuage chargé: {len(self.pc_manager.points)} points ({cloud_width}x{cloud_height})')
        
        # 2. Appliquer le masque de segmentation OU fallback sur filtrage par position
        if self.latest_mask is not None:
            # Use segmentation mask for precise object identification
            masked_points = self.pc_manager.apply_segmentation_mask(
                self.latest_mask,
                self.highlight_color,
                cloud_width,
                cloud_height
            )
            self.get_logger().info(f'   🎭 Points segmentés par masque: {len(masked_points)}')
            
            if len(masked_points) == 0:
                self.get_logger().warn('   ⚠️  Aucun point 3D correspondant au masque!')
                # Fallback: use position-based filtering
                target_pos = Vector3(msg.point.x, msg.point.y, msg.point.z)
                masked_points = self.pc_manager.filter_around_position(target_pos, self.filter_radius)
                for p in masked_points:
                    p.color = self.highlight_color
                self.get_logger().info(f'   🔄 Fallback position: {len(masked_points)} points')
        else:
            # No mask available - fallback to position-based filtering
            self.get_logger().warn('   ⚠️  Pas de masque disponible, utilisation du filtrage par position')
            target_pos = Vector3(msg.point.x, msg.point.y, msg.point.z)
            masked_points = self.pc_manager.filter_around_position(target_pos, self.filter_radius)
            for p in masked_points:
                p.color = self.highlight_color
            self.get_logger().info(f'   🎯 Points filtrés (rayon {self.filter_radius}m): {len(masked_points)}')
        
        if len(masked_points) == 0:
            self.get_logger().warn('   ⚠️  Aucun point trouvé!')
            return
        
        # 3. Créer un manager temporaire avec les points colorés
        temp_manager = PointCloudManager()
        temp_manager.points = masked_points
        
        # 4. Publier le Marker VERT à la position détectée (centre bbox) dans camera_link
        marker = self.create_detection_marker(msg)
        self.marker_pub.publish(marker)
        self.get_logger().info('   ✅ Marker VERT publié sur /object/detection_marker (camera_link)')
        
        # 5. Publier le nuage filtré et coloré
        filtered_cloud = temp_manager.to_ros_pointcloud2(
            msg.header.frame_id,
            self.get_clock().now().to_msg()
        )
        self.filtered_pc_pub.publish(filtered_cloud)
        self.get_logger().info('   ✅ Nuage segmenté publié sur /object/pointcloud_filtered')
        
        # 6. Transformer la position vers le repère MAP et publier le marqueur ROUGE
        point_map = self.transform_to_map(msg)
        if point_map is not None:
            map_marker = self.create_map_marker(point_map)
            self.map_marker_pub.publish(map_marker)
            self.get_logger().info(
                f'   🔴 Marker ROUGE publié sur /object/detection_marker_map (map)\n'
                f'      Position MAP: x={point_map.point.x:.2f}m, y={point_map.point.y:.2f}m, z={point_map.point.z:.2f}m'
            )
        else:
            self.get_logger().warn('   ⚠️  Impossible de créer le marqueur map (transformation échouée)')


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
        marker.header.frame_id = 'map'  # EXPLICITEMENT dans repère MAP
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
    node = SemanticPointCloudVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Arrêt du nœud Semantic PointCloud Visualizer')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()