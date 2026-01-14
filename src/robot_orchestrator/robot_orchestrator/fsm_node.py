import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from robot_orchestrator.states import RobotState

class RobotOrchestrator(Node):
    def __init__(self):
        super().__init__('robot_orchestrator')
        self.state = RobotState.INIT

        # Flags
        self.localization_ok = False
        self.goal_reached = False
        self.target_detected = False
        self.obstacle_detected = False

        # ================= Publishers =================
        self.nav_cmd_pub = self.create_publisher(String, '/navigation/command', 10)
        self.start_detection_pub = self.create_publisher(Bool, '/camera_processor/start_detection', 10)
        self.avoid_pub = self.create_publisher(Bool, '/safety_detector/avoid_obstacle', 10)
        self.pointcloud_trigger_pub = self.create_publisher(Bool, '/pointcloud_visualizer/project', 10)

        # ================= Subscribers =================
        self.create_subscription(String, '/detection/command', self.detection_callback, 10)
        self.create_subscription(Bool, '/safety_detector/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(Bool, '/navigation/goal_reached', self.goal_reached_callback, 10)
        self.create_subscription(Bool, '/amcl/localization_ok', self.localization_callback, 10)

        # FSM loop
        self.timer = self.create_timer(1.0, self.run_fsm)
        self.get_logger().info("FSM Orchestrator with Safety started")

    # ================= CALLBACKS =================
    def detection_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Target detected info received: {command}")
        self.target_detected = True
        
        # RELAY LOGIC: Forward commands to navigation
        # Orchestrator decides: if we are in detection/nav mode, forward the command
        if self.state in [RobotState.NAVIGATION, RobotState.DETECTION]:
            relay_msg = String()
            
            if command == "TARGET_FOUND":
                self.state = RobotState.OBJECT_FOUND
                relay_msg.data = "STOP"
                self.get_logger().info("Decision: Target found -> Stopping Navigation")
            else:
                relay_msg.data = command
                
            self.nav_cmd_pub.publish(relay_msg)

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data

    def goal_reached_callback(self, msg):
        self.goal_reached = msg.data

    def localization_callback(self, msg):
        self.localization_ok = msg.data

    # ================= FSM LOOP =================
    def run_fsm(self):
        self.get_logger().info(f"STATE: {self.state.name}")

        # Priorité obstacle
        if self.obstacle_detected and self.state in [RobotState.NAVIGATION, RobotState.DETECTION]:
            self.state = RobotState.AVOID_OBSTACLE

        # States
        if self.state == RobotState.INIT:
            self.handle_init()
        elif self.state == RobotState.LOCALIZATION:
            self.handle_localization()
        elif self.state == RobotState.NAVIGATION:
            self.handle_navigation()
        elif self.state == RobotState.AVOID_OBSTACLE:
            self.handle_avoid_obstacle()
        elif self.state == RobotState.OBJECT_FOUND:
            self.handle_object_found()
        elif self.state == RobotState.STOP:
            self.handle_stop()

    # ================= STATE HANDLERS =================
    def handle_init(self):
        self.get_logger().info("Initializing robot system...")
        self.state = RobotState.LOCALIZATION

    def handle_localization(self):
        self.get_logger().info("Waiting for localization from /amcl... (Simulation: Assuming OK)")
        # Simulation bypass: Assume localization is OK
        self.state = RobotState.NAVIGATION

    def handle_navigation(self):
        self.get_logger().info("Publishing START command to /navigation/command")
        msg = String()
        msg.data = "START"
        self.nav_cmd_pub.publish(msg)

        if self.goal_reached:
            self.state = RobotState.DETECTION

    def handle_detection(self):
        self.get_logger().info("Starting detection via /camera_processor + /inference")
        msg = Bool()
        msg.data = True
        self.start_detection_pub.publish(msg)

        if self.target_detected:
            self.get_logger().info("Object detected → projecting pointcloud")
            self.state = RobotState.OBJECT_FOUND
        else:
            self.get_logger().info("Object not detected → continue navigation")
            self.state = RobotState.NAVIGATION

    def handle_avoid_obstacle(self):
        self.get_logger().warn("Obstacle detected → requesting avoidance via /safety_detector")
        msg = Bool()
        msg.data = True
        self.avoid_pub.publish(msg)
        self.obstacle_detected = False
        self.state = RobotState.NAVIGATION

    def handle_object_found(self):
        msg = Bool()
        msg.data = True
        self.pointcloud_trigger_pub.publish(msg)
        self.get_logger().info("Object found → pointcloud projected, mission complete!")
        self.state = RobotState.STOP

    def handle_stop(self):
        self.get_logger().info("FSM Orchestrator stopping")
        rclpy.shutdown()


def main():
    rclpy.init()
    orchestrator = RobotOrchestrator()
    rclpy.spin(orchestrator)


if __name__ == "__main__":
    main()
