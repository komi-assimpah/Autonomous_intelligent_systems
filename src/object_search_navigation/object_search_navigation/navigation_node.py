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
from std_msgs.msg import String, Bool

from object_search_navigation.diffdrive import wheels_to_twist
import object_search_navigation.lidar_utils as lu


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Smart Navigation Node started')


        # === FSM States ===
        self.WAITING = -1
        self.RECALCULATE_DIRECTION = 0
        self.TURN = 1
        self.FORWARD = 2
        self.MISSION_COMPLETE = 3
        
        self.state = self.WAITING

        # === Speed Parameters ===
        self.SPEED_LINEAR_MAX = 0.2       
        self.SPEED_LINEAR_MIN = 0.15      
        self.SPEED_ANGULAR_MAX = 0.15      
        self.Kp_ANGULAR = 2.0              
        
        # === Navigation Parameters ===
        self.ALIGN_TOLERANCE_DEG = 15.0    
        self.WHEEL_SEPARATION = 0.16       
        self.MIN_RANGE = 0.29              
        self.DISTANCE_STOP = 0.35          
        self.DISTANCE_SLOWDOWN = 0.8       
        self.OFFSET_MAX_DEG = 15.0         
        self.FOV = 60.0                    

        # === Exploration Memory ===
        self.recent_angles = []
        self.max_recent_angles = 5
        self.avoid_recent_weight = 0.3

        # === Internal State ===
        self.last_scan = None
        self.current_yaw = 0.0
        self.target_angle = 0.0
        self.last_turn_time = time.time()
        self.stuck_counter = 0


        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.command_sub = self.create_subscription(String, '/navigation/command', self.command_callback, 10)

        self.timer = self.create_timer(0.05, self.control_cycle)

        signal.signal(signal.SIGINT, self.signal_handler)

        self.get_logger().info(' Smart exploration node ready - WAITING for command...')

    def signal_handler(self, signum, frame):
        self.get_logger().info("Ctrl+C detected, stopping robot...")
        self.emergency_stop()
        raise SystemExit
        
    def command_callback(self, msg):
        command = msg.data
  
        if command == "START":
            if self.state == self.WAITING:
                self.state = self.RECALCULATE_DIRECTION
                self.get_logger().info(' START command received! Beginning exploration...')
                
        elif command == "STOP":
            self.state = self.MISSION_COMPLETE
            for _ in range(5):
                self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('STOP command received - Mission complete!')

    def scan_callback(self, msg):
        self.last_scan = msg

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def emergency_stop(self):
        for _ in range(3):
            self.cmd_vel_pub.publish(Twist())

    def find_best_direction(self):
        scan = self.last_scan
        if scan is None:
            return self.current_yaw
        
        best_angle = None
        best_score = -1
        
        for angle_deg in range(-180, 180, 30):
            angle_rad = math.radians(angle_deg)
            world_angle = lu.normalize_angle(self.current_yaw + angle_rad)
            
            dist, _ = lu.get_min_range_at_angle(scan, angle_deg, 45.0, self.MIN_RANGE)
            if dist is None or dist > scan.range_max:
                dist = scan.range_max
            
            score = dist
            
            for recent in self.recent_angles:
                angle_diff = abs(lu.normalize_angle(world_angle - recent))
                if angle_diff < math.radians(45):
                    score -= self.avoid_recent_weight * dist
            
            if abs(angle_deg) < 60:
                score += 0.2 * dist
            
            if score > best_score:
                best_score = score
                best_angle = world_angle
        
        random_offset = math.radians(random.uniform(-self.OFFSET_MAX_DEG, self.OFFSET_MAX_DEG))
        best_angle = lu.normalize_angle(best_angle + random_offset)
        
        self.recent_angles.append(best_angle)
        if len(self.recent_angles) > self.max_recent_angles:
            self.recent_angles.pop(0)
        
        return best_angle

    def get_adaptive_speed(self, distance):
        if distance <= self.DISTANCE_STOP:
            return 0.0
        elif distance >= self.DISTANCE_SLOWDOWN:
            return self.SPEED_LINEAR_MAX
        else:
            ratio = (distance - self.DISTANCE_STOP) / (self.DISTANCE_SLOWDOWN - self.DISTANCE_STOP)
            return self.SPEED_LINEAR_MIN + ratio * (self.SPEED_LINEAR_MAX - self.SPEED_LINEAR_MIN)

    def control_cycle(self):
        if self.state == self.WAITING:
            return

        if self.state == self.MISSION_COMPLETE:
            return # Target found, do nothing
        
        
        if self.last_scan is None:
            return

        vel_l = 0.0
        vel_r = 0.0

        # ==============================
        # STATE: RECALCULATE_DIRECTION - Find best direction
        # ==============================
        if self.state == self.RECALCULATE_DIRECTION:
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
            
            # self.get_logger().info(f"Target: {math.degrees(self.target_angle):.1f} | Curr: {math.degrees(self.current_yaw):.1f} | Err: {angle_error_deg:.1f}")

            if abs(angle_error_deg) >= self.ALIGN_TOLERANCE_DEG:
                vel = self.Kp_ANGULAR * angle_error
                vel = max(min(vel, self.SPEED_ANGULAR_MAX), -self.SPEED_ANGULAR_MAX)
                vel_r = vel
                vel_l = -vel
                
                if time.time() - self.last_turn_time > 10.0:
                    self.stuck_counter += 1
                    if self.stuck_counter >= 3:
                        self.get_logger().warn(f"⚠️ Possibly stuck (Err: {angle_error_deg:.1f}°), trying new direction")
                        self.state = self.RECALCULATE_DIRECTION
                        self.stuck_counter = 0
            else:
                self.state = self.FORWARD
                self.stuck_counter = 0
                self.get_logger().info(f"Aligned (Err: {angle_error_deg:.1f}°) → Moving forward")

        # ==============================
        # STATE: FORWARD - Move with adaptive speed
        # ==============================
        elif self.state == self.FORWARD:
            dist, _ = lu.get_min_range_at_angle(self.last_scan, 0.0, self.FOV, self.MIN_RANGE)
            self.get_logger().info(f"Forward with dist: {dist:.2f}m")
            
            if dist is None:
                dist = lu.get_max_range(self.last_scan)

            if dist < self.DISTANCE_STOP:
                vel_l = 0.0
                vel_r = 0.0
                self.state = self.RECALCULATE_DIRECTION
                self.get_logger().info(f"Obstacle at {dist:.2f}m")
            else:
                speed = self.get_adaptive_speed(dist)
                vel_l = speed
                vel_r = speed
                
                dist_left, _ = lu.get_min_range_at_angle(self.last_scan, 30.0, 30.0, self.MIN_RANGE)
                dist_right, _ = lu.get_min_range_at_angle(self.last_scan, -30.0, 30.0, self.MIN_RANGE)
                
                if dist_left and dist_right:
                    if dist_left < 0.5 and dist_right > dist_left:
                        vel_l *= 1.1  # Slight right turn
                        vel_r *= 0.9
                    elif dist_right < 0.5 and dist_left > dist_right:
                        vel_l *= 0.9  # Slight left turn
                        vel_r *= 1.1

        twist = wheels_to_twist(vel_l, vel_r, self.WHEEL_SEPARATION)
        
        if abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.01:
             self.get_logger().info(f"CMD_VEL: Lin={twist.linear.x:.2f} Ang={twist.angular.z:.2f}")
             
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