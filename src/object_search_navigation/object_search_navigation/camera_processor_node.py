import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessorNode(Node):
    def __init__(self):
        super().__init__('camera_processor_node')
        self.get_logger().info('====== Camera Processor Node started =======')
        
        self.bridge = CvBridge()
        
        self.target_width = 640
        self.target_height = 480
        self.apply_clahe = True  # Amélioration du contraste
        self.apply_denoise = False  # Réduction du bruit (coûteux en calcul)
        
        self.frame_count = 0
        self.processing_errors = 0
        
        self.camera_sub = self.create_subscription(
            Image,
            '/rgb_camera/image',
            self.image_callback,
            10
        )
        
        self.processed_pub = self.create_publisher(
            Image,
            '/processed/camera_feed',
            10
        )
    
    
    
    def image_callback(self, msg):
        """Traite et republie chaque image reçue"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') #?
            processed_image = self.preprocess_image(cv_image)
            
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            processed_msg.header = msg.header  # Garder le timestamp original
            
            self.processed_pub.publish(processed_msg)
            
            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'📊 Processed {self.frame_count} frames')
        
        except Exception as e:
            self.processing_errors += 1
            self.get_logger().error(f'❌ Error processing image: {str(e)}')
    
    ## code generée
    def preprocess_image(self, image):
        """
        Pipeline de prétraitement d'image pour Edge AI
        Adapté aux besoins de détection d'objets
        """
        height, width = image.shape[:2]
        if width != self.target_width or height != self.target_height:
            image = cv2.resize(
                image, (self.target_width, self.target_height),
                interpolation=cv2.INTER_LINEAR
            )
        
        # 2. Amélioration du contraste (CLAHE)
        if self.apply_clahe:
            # Convertir en LAB pour traiter uniquement la luminance
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            
            # Appliquer CLAHE sur le canal L
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l = clahe.apply(l)
            
            # Reconstituer l'image
            lab = cv2.merge([l, a, b])
            image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        
        # 3. Réduction du bruit (optionnel, coûteux)
        if self.apply_denoise:
            image = cv2.fastNlMeansDenoisingColored(image, None, 10, 10, 7, 21)
        
        # 4. Normalisation (optionnel, selon votre modèle IA)
        # image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
        
        return image
    
    
    ## pour debug
    def get_statistics(self):
        """Retourne les statistiques de traitement"""
        return {
            'frames_processed': self.frame_count,
            'errors': self.processing_errors,
            'success_rate': (self.frame_count / (self.frame_count + self.processing_errors)) * 100 if self.frame_count > 0 else 0
        }


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
