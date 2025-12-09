import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import random


class SimpleDetectorNode(Node):
    """
    Détecteur d'objets simplifié pour tests.
    L'équipe IA remplacera la méthode detect_object() par YOLO/autre modèle.
    """
    def __init__(self):
        super().__init__('simple_detector_node')
        self.get_logger().info('🔍 Simple Object Detector started')
        
        self.bridge = CvBridge()
        self.target_object = 'dog'  # Objet cible (paramétrable)
        self.detection_threshold = 0.7  # Confiance minimale
        self.frame_count = 0
        
        self.image_sub = self.create_subscription(
            Image,
            '/processed/camera_feed',
            self.image_callback,
            10
        )
        
        self.detection_pub = self.create_publisher(
            String,
            '/detection/command',
            10
        )
        
        self.get_logger().info(f'==> Looking for: {self.target_object}')
        #TODO: MOCK detection (replace with YOLO later)')
    
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # DÉTECTION : à remplacer par YOLO
            detected = self.detect_object(cv_image)
            
            if detected:
                self.get_logger().info(f'🎯 {self.target_object.upper()} DETECTED! Sending STOP command...')
                
                # Publier commande STOP
                stop_msg = String()
                stop_msg.data = 'STOP'
                self.detection_pub.publish(stop_msg)
            
            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'📊 Analyzed {self.frame_count} frames')
        
        except Exception as e:
            self.get_logger().error(f'❌ Error in detection: {str(e)}')
    
    
    def detect_object(self, image):
        """
        MOCK DETECTION - À REMPLACER PAR L'ÉQUIPE IA
        
        Pour l'instant : détection aléatoire (5% de chance)
        
        L'équipe IA remplacera par YOLOv8
        
        Args:
            image: Image OpenCV (numpy array)
        
        Returns:
            bool: True si objet détecté avec confiance > threshold
        """
        # SIMULATION : détection aléatoire
        # Remplacer par: results = model.predict(image)
        detection_probability = random.random()
        
        if detection_probability < 0.001:  # 0.1% chance (was 5%)
            return True
        
        return False
    


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
