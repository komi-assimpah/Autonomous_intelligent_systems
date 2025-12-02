import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node started')
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.detection_sub = self.create_subscription(
            String,
            '/detection/command',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('✅ Listening for detection on /detection/command')
        #TODO: Integrate Nav2 for autonomous exploration, after understanding how it's currently done    
    
    
    def detection_callback(self, msg):
        if msg.data == "STOP":
            self.get_logger().info('OBJECT FOUND! Stopping mission.')
            self.emergency_stop()
    
    
    def emergency_stop(self):
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('Emergency stop executed - Mission complete!')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()