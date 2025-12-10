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
        self.declare_parameter('model_path', 'yolo11n-seg.pt')
        self.declare_parameter('conf_threshold', 0.5)
        
        self.subscription = self.create_subscription(
            Image,
            '/processed/camera_feed',
            self.image_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(
            String,
            '/detection/command',
            10
        )

        self.image_pub_ = self.create_publisher(
            Image,
            '/inference/image_processed',
            10
        )
        
        self.br = CvBridge()

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)

        self.get_logger().info(f'Chargement du modèle de segmentation: {model_path}...')
        self.model = YOLO(model_path)
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
        
        object_detected, annotated_frame = self.run_inference(cv_image)

        if annotated_frame is not None:
            processed_msg = self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_pub_.publish(processed_msg)

        if object_detected:
            msg = String()
            msg.data = "STOP"
            self.publisher_.publish(msg)
            self.get_logger().warn('OBJET DÉTECTÉ ! Envoi de la commande STOP.')

    
    def run_inference(self, frame):

        results = self.model(frame, verbose=False, conf=self.conf_threshold)
        
        detected = False
        annotated_frame = frame 

        for r in results:
            annotated_frame = r.plot() 

            if r.masks is None:
                continue

            for box in r.boxes:
                cls_id = int(box.cls[0])
                current_class = self.model.names.get(cls_id, str(cls_id))

                if current_class == self.target_object:
                    detected = True
                    break 
        
        return detected, annotated_frame

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