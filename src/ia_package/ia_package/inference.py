import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class Inference(Node):
    
    def __init__(self):
        super().__init__('inference')
        
        self.declare_parameter('target_class', 'person')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(
            String,
            '/robot_cmd',
            10
        )
        
        self.br = CvBridge()

        self.get_logger().info('Chargement du modèle YOLOv11n...')
        self.model = YOLO("yolo11n.pt") 
        self.get_logger().info('Modèle chargé !')

        self.target_object = self.get_parameter('target_class').value
        
        self.get_logger().info(f'Nœud d\'IA démarré. Classe cible: {self.target_object}')
        self.get_logger().info(f'Classes disponibles: {list(self.model.names.values())}')

    
    def image_callback(self, msg):
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Erreur de conversion: {e}')
            return

        object_detected = self.run_inference(cv_image)

        if object_detected:
            msg = String()
            msg.data = "STOP"
            self.publisher_.publish(msg)
            self.get_logger().warn('OBJET DÉTECTÉ ! Envoi de la commande STOP.')

    
    def run_inference(self, frame):

        results = self.model(frame, verbose=False, conf=0.5) 

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                current_class = self.model.names[cls_id]
                
                if current_class == self.target_object:
                    return True 
                
        return False

def main(args=None):
    rclpy.init(args=args)
    node = Inference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()