import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node started')
        
        # Publisher pour republier les images de la caméra
        self.camera_pub = self.create_publisher(
            Image, 
            '/search/camera_feed',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Subscriber pour recevoir les images de la caméra
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        # Subscriber pour recevoir le statut du détecteur
        self.status_sub = self.create_subscription(
            String,
            '/search/status',
            self.status_callback,
            10
        )
    
    # Callback appelée quand une image arrive de la caméra
    def camera_callback(self, msg):
        """Republier l'image sur /search/camera_feed pour le détecteur"""
        self.camera_pub.publish(msg)
    
    # Callback appelée quand le statut change (SEARCHING ou STOP)
    def status_callback(self, msg):
        if msg.data == "STOP":
            self.get_logger().info("Object found!!!, stopping")
            # Twist() crée un message avec toutes les vitesses à 0
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()