import math
import random
import rclpy
import signal

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from object_search_navigation.diffdrive import wheels_to_twist
import object_search_navigation.lidar_utils as lu


class NavigationNode(Node):
    """
    Navigation node with FSM for autonomous exploration.
    Combines obstacle avoidance with object detection.
    
    States:
    - STOP: Find clear direction
    - TURN: Rotate towards clear area
    - FORWARD: Move forward until obstacle detected
    
    Stops completely when /detection/command receives "STOP"
    """

    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('🚗 Navigation Node (FSM) started')

        # === Mission State ===
        self.target_found = False  # Set to True when object detected

        # === FSM States ===
        self.STOP = 0
        self.TURN = 1
        self.FORWARD = 2
        self.state = self.STOP

        # === Parameters ===
        self.SPEED_LINEAR = 0.2        # Forward speed (m/s)
        self.SPEED_ANGULAR_MAX = 0.5   # Max rotation speed (rad/s)
        self.Kp_ANGULAR = 1.5          # P-controller for turn
        self.ALIGN_TOLERANCE_DEG = 2.0 # Alignment tolerance
        self.WHEEL_SEPARATION = 0.16   # TurtleBot3 Burger wheel separation
        self.DISTANCE_MIN = 0.35       # Stop distance for obstacles
        self.OFFSET_MAX_DEG = 5.0      # Random offset for exploration
        self.FOV = 45.0                # Field of view for obstacle detection

        # === Internal State ===
        self.last_scan = None
        self.current_yaw = 0.0
        self.target_angle = 0.0

        # === Publishers ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === Subscribers ===
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.detection_sub = self.create_subscription(
            String, '/detection/command', self.detection_callback, 10)

        # === Control Loop Timer (20 Hz) ===
        self.timer = self.create_timer(0.05, self.control_cycle)

        # === Graceful shutdown ===
        signal.signal(signal.SIGINT, self.signal_handler)

        self.get_logger().info('✅ Listening for detection on /detection/command')
        self.get_logger().info('✅ FSM exploration active - looking for target object')

    # === Signal Handler ===
    def signal_handler(self, signum, frame):
        self.get_logger().info("Ctrl+C detected, stopping robot...")
        self.emergency_stop()
        raise SystemExit

    # === Callbacks ===
    def scan_callback(self, msg):
        self.last_scan = msg

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def detection_callback(self, msg):
        """Handle detection commands from object_detector"""
        if msg.data == "STOP":
            self.get_logger().info('🎯 OBJECT FOUND! Stopping mission.')
            self.target_found = True
            self.emergency_stop()

    # === Emergency Stop ===
    def emergency_stop(self):
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('🛑 Robot stopped - Mission complete!')

    # === Main FSM Control Loop ===
    def control_cycle(self):
        # Don't move if target found or no scan data
        if self.target_found:
            return
        
        if self.last_scan is None:
            return

        vel_l = 0.0
        vel_r = 0.0

        # ==============================
        # STATE: STOP - Find clear direction
        # ==============================
        if self.state == self.STOP:
            self.target_angle = lu.get_angle_of_clear_area_world(
                self.last_scan, self.current_yaw, self.DISTANCE_MIN, window_deg=self.FOV)

            # Add random offset for more natural exploration
            random_offset = math.radians(random.uniform(-self.OFFSET_MAX_DEG, self.OFFSET_MAX_DEG))
            self.target_angle += random_offset

            self.state = self.TURN
            self.get_logger().info(f"STOP → TURN | Target: {math.degrees(self.target_angle):.1f}°")

        # ==============================
        # STATE: TURN - Rotate towards target angle
        # ==============================
        elif self.state == self.TURN:
            angle_error = lu.normalize_angle(self.target_angle - self.current_yaw)
            angle_error_deg = math.degrees(angle_error)

            if abs(angle_error_deg) >= self.ALIGN_TOLERANCE_DEG:
                # Proportional control
                vel = self.Kp_ANGULAR * angle_error
                vel = max(min(vel, self.SPEED_ANGULAR_MAX), -self.SPEED_ANGULAR_MAX)
                vel_r = vel
                vel_l = -vel
            else:
                vel_r = 0.0
                vel_l = 0.0
                self.state = self.FORWARD
                self.get_logger().info(f"Aligned → FORWARD")

        # ==============================
        # STATE: FORWARD - Move until obstacle
        # ==============================
        elif self.state == self.FORWARD:
            # Check for obstacles ahead
            dist, _ = lu.get_min_range_at_angle(self.last_scan, 0.0, self.FOV)
            if dist is None:
                dist = lu.get_max_range(self.last_scan)

            if dist < self.DISTANCE_MIN:
                vel_l = 0.0
                vel_r = 0.0
                self.state = self.STOP
                self.get_logger().info(f"Obstacle ({dist:.2f}m) → STOP")
            else:
                vel_l = self.SPEED_LINEAR
                vel_r = self.SPEED_LINEAR

        # Publish velocity command
        twist = wheels_to_twist(vel_l, vel_r, self.WHEEL_SEPARATION)
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()