import math
import random
import rclpy
import signal
import time

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
    Smart Navigation node with FSM for autonomous exploration.
    Features:
    - Adaptive speed based on obstacle distance
    - Prefers exploring new directions
    - Smooth turning with look-ahead
    - Camera-aware exploration (prefers forward-facing directions)
    
    States:
    - STOP: Find best direction to explore
    - TURN: Rotate towards target angle
    - FORWARD: Move forward with adaptive speed
    
    Stops completely when /detection/command receives "STOP"
    """

    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('🚗 Smart Navigation Node started')

        # === Mission State ===
        self.target_found = False

        # === FSM States ===
        self.STOP = 0
        self.TURN = 1
        self.FORWARD = 2
        self.state = self.STOP

        # === Speed Parameters ===
        self.SPEED_LINEAR_MAX = 0.3    # Max forward speed (m/s)
        self.SPEED_LINEAR_MIN = 0.1   # Min speed when approaching obstacles
        self.SPEED_ANGULAR_MAX = 0.7   # Max rotation speed (rad/s)
        self.Kp_ANGULAR = 2.5          # P-controller for turn

        # === Navigation Parameters ===
        self.ALIGN_TOLERANCE_DEG = 3.0     # Alignment tolerance (slightly larger for speed)
        self.WHEEL_SEPARATION = 0.16       # TurtleBot3 Burger wheel separation
        self.DISTANCE_STOP = 0.35          # Stop distance for obstacles
        self.DISTANCE_SLOWDOWN = 0.8       # Start slowing down at this distance
        self.OFFSET_MAX_DEG = 15.0         # Random offset for exploration variety
        self.FOV = 60.0                    # Wider FOV for better awareness

        # === Exploration Memory (avoid revisiting same directions) ===
        self.recent_angles = []            # Recent directions taken
        self.max_recent_angles = 5         # Remember last N directions
        self.avoid_recent_weight = 0.3     # Penalty for recent directions

        # === Internal State ===
        self.last_scan = None
        self.current_yaw = 0.0
        self.target_angle = 0.0
        self.last_turn_time = time.time()
        self.stuck_counter = 0             # Detect if stuck

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

        self.get_logger().info('✅ Smart exploration active - searching for target')

    def signal_handler(self, signum, frame):
        self.get_logger().info("Ctrl+C detected, stopping robot...")
        self.emergency_stop()
        raise SystemExit

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
            self.get_logger().info('🎯 TARGET FOUND! Stopping mission.')
            self.target_found = True
            self.emergency_stop()

    def emergency_stop(self):
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('🛑 Robot stopped - Mission complete!')

    def find_best_direction(self):
        """
        Find the best direction to explore, considering:
        1. Maximum clear distance
        2. Avoid recently visited directions
        3. Slight preference for forward (camera view)
        """
        scan = self.last_scan
        if scan is None:
            return self.current_yaw
        
        best_angle = None
        best_score = -1
        
        # Check 12 directions around the robot (every 30 degrees)
        for angle_deg in range(-180, 180, 30):
            angle_rad = math.radians(angle_deg)
            world_angle = lu.normalize_angle(self.current_yaw + angle_rad)
            
            # Get distance at this angle
            dist, _ = lu.get_min_range_at_angle(scan, angle_rad, 45.0)
            if dist is None or dist > scan.range_max:
                dist = scan.range_max
            
            # Base score is distance (want to go where there's space)
            score = dist
            
            # Penalty for recently visited directions
            for recent in self.recent_angles:
                angle_diff = abs(lu.normalize_angle(world_angle - recent))
                if angle_diff < math.radians(45):
                    score -= self.avoid_recent_weight * dist
            
            # Bonus for forward direction (camera can see there)
            if abs(angle_deg) < 60:
                score += 0.2 * dist
            
            if score > best_score:
                best_score = score
                best_angle = world_angle
        
        # Add random variation for exploration
        random_offset = math.radians(random.uniform(-self.OFFSET_MAX_DEG, self.OFFSET_MAX_DEG))
        best_angle = lu.normalize_angle(best_angle + random_offset)
        
        # Remember this direction
        self.recent_angles.append(best_angle)
        if len(self.recent_angles) > self.max_recent_angles:
            self.recent_angles.pop(0)
        
        return best_angle

    def get_adaptive_speed(self, distance):
        """
        Calculate adaptive speed based on distance to nearest obstacle.
        Slow down as we approach obstacles for smoother navigation.
        """
        if distance <= self.DISTANCE_STOP:
            return 0.0
        elif distance >= self.DISTANCE_SLOWDOWN:
            return self.SPEED_LINEAR_MAX
        else:
            # Linear interpolation between min and max speed
            ratio = (distance - self.DISTANCE_STOP) / (self.DISTANCE_SLOWDOWN - self.DISTANCE_STOP)
            return self.SPEED_LINEAR_MIN + ratio * (self.SPEED_LINEAR_MAX - self.SPEED_LINEAR_MIN)

    def control_cycle(self):
        if self.target_found:
            return
        
        if self.last_scan is None:
            return

        vel_l = 0.0
        vel_r = 0.0

        # ==============================
        # STATE: STOP - Find best direction
        # ==============================
        if self.state == self.STOP:
            self.target_angle = self.find_best_direction()
            self.state = self.TURN
            self.last_turn_time = time.time()
            self.get_logger().info(f"🔄 Exploring → {math.degrees(self.target_angle):.0f}°")

        # ==============================
        # STATE: TURN - Rotate towards target
        # ==============================
        elif self.state == self.TURN:
            angle_error = lu.normalize_angle(self.target_angle - self.current_yaw)
            angle_error_deg = math.degrees(angle_error)

            if abs(angle_error_deg) >= self.ALIGN_TOLERANCE_DEG:
                # Proportional control with smooth acceleration
                vel = self.Kp_ANGULAR * angle_error
                vel = max(min(vel, self.SPEED_ANGULAR_MAX), -self.SPEED_ANGULAR_MAX)
                vel_r = vel
                vel_l = -vel
                
                # Detect if stuck turning too long
                if time.time() - self.last_turn_time > 5.0:
                    self.stuck_counter += 1
                    if self.stuck_counter >= 3:
                        self.get_logger().warn("⚠️ Possibly stuck, trying new direction")
                        self.state = self.STOP
                        self.stuck_counter = 0
            else:
                self.state = self.FORWARD
                self.stuck_counter = 0
                self.get_logger().info(f"✅ Aligned → Moving forward")

        # ==============================
        # STATE: FORWARD - Move with adaptive speed
        # ==============================
        elif self.state == self.FORWARD:
            # Check for obstacles ahead with wider FOV
            dist, _ = lu.get_min_range_at_angle(self.last_scan, 0.0, self.FOV)
            if dist is None:
                dist = lu.get_max_range(self.last_scan)

            if dist < self.DISTANCE_STOP:
                vel_l = 0.0
                vel_r = 0.0
                self.state = self.STOP
                self.get_logger().info(f"🛑 Obstacle at {dist:.2f}m")
            else:
                # Adaptive speed based on distance
                speed = self.get_adaptive_speed(dist)
                vel_l = speed
                vel_r = speed
                
                # Also check left and right for early warning
                dist_left, _ = lu.get_min_range_at_angle(self.last_scan, math.radians(30), 30.0)
                dist_right, _ = lu.get_min_range_at_angle(self.last_scan, math.radians(-30), 30.0)
                
                # Slight steering to avoid side obstacles
                if dist_left and dist_right:
                    if dist_left < 0.5 and dist_right > dist_left:
                        vel_l *= 1.1  # Slight right turn
                        vel_r *= 0.9
                    elif dist_right < 0.5 and dist_left > dist_right:
                        vel_l *= 0.9  # Slight left turn
                        vel_r *= 1.1

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