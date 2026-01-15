import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class Inference(Node):
    
    def __init__(self):
        super().__init__('inference')
        
        self.declare_parameter('target_class', 'dog')
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
            '/robot_cmd',
            10
        )

        self.image_pub_ = self.create_publisher(
            Image,
            '/inference/image_processed',
            10
        )
        
        self.mask_pub_ = self.create_publisher(
            Image,
            '/inference/mask',
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
        mask_msg = None

        if results[0].masks is not None and results[0].boxes is not None:
             combined_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
             
             for i, box in enumerate(results[0].boxes):
                 cls_id = int(box.cls[0])
                 mask_val = cls_id + 1
                 
                 raw_mask = results[0].masks.data[i].cpu().numpy()
                 
                 if raw_mask.shape[:2] != (frame.shape[0], frame.shape[1]):
                     raw_mask = cv2.resize(raw_mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
                 
                 combined_mask[raw_mask > 0.5] = mask_val
                 detected = True

             try:
                 mask_msg = self.br.cv2_to_imgmsg(combined_mask, encoding="mono8")
                 self.mask_pub_.publish(mask_msg)
             except Exception as e:
                 self.get_logger().error(f'Failed to publish mask: {e}')
        
        annotated_frame = results[0].plot()

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