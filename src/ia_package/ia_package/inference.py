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
        
        self.publisher_ = self.create_publisher(String, '/detection/command', 10)
        self.image_pub_ = self.create_publisher(Image, '/inference/image_processed', 10)
        self.mask_pub_ = self.create_publisher(Image, '/object/segmentation_mask', 10)
        self.class_pub_ = self.create_publisher(String, '/object/class_name', 10)

        self.subscription = self.create_subscription(Image, '/camera/image_decompressed', self.image_callback, 10)
        
        self.br = CvBridge()

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
        
        self.get_logger().info(f'Classe cible: {self.target_object}')
    
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