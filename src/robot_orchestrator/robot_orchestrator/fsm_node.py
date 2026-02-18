import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from robot_orchestrator.states import RobotState

class RobotOrchestrator(Node):
    def __init__(self):
        super().__init__('robot_orchestrator')
        self.state = RobotState.INIT

        self.target_detected = False
        
        self.declare_parameter('localization_wait_time', 0)  # seconds
        self.localization_wait_time = self.get_parameter('localization_wait_time').value
        self.localization_start_time = None

        self.nav_cmd_pub = self.create_publisher(String, '/navigation/command', 10)
        self.pointcloud_trigger_pub = self.create_publisher(Bool, '/pointcloud_visualizer/project', 10)
        self.create_subscription(String, '/detection/command', self.detection_callback, 10)

        self.timer = self.create_timer(1.0, self.run_fsm)
        self.get_logger().info("FSM Orchestrator started")


    # ================= CALLBACKS =================
    def detection_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Target detected info received: {command}")
        self.target_detected = True
        
        if self.state == RobotState.NAVIGATION:
            relay_msg = String()
            
            if command == "TARGET_FOUND":
                self.state = RobotState.MISSION_COMPLETE
                relay_msg.data = "STOP"
                self.get_logger().info("Decision: Target found -> Stopping Navigation")
            else:
                relay_msg.data = command
                
            self.nav_cmd_pub.publish(relay_msg)


    # ================= FSM LOOP =================
    def run_fsm(self):
        if self.state == RobotState.INIT:
            self.handle_init()
        elif self.state == RobotState.LOCALIZATION:
            self.handle_localization()
        elif self.state == RobotState.NAVIGATION:
            self.handle_navigation()
        elif self.state == RobotState.MISSION_COMPLETE:
            self.handle_object_found()
        elif self.state == RobotState.STOP:
            self.handle_stop()

    # ================= STATE HANDLERS =================
    def handle_init(self):
        self.get_logger().info("Initializing robot system...")
        self.state = RobotState.LOCALIZATION

    def handle_localization(self):
        if self.localization_start_time is None:
            self.localization_start_time = self.get_clock().now()
            self.get_logger().info(f"Waiting {self.localization_wait_time}s for AMCL to converge...")
        
        elapsed = (self.get_clock().now() - self.localization_start_time).nanoseconds / 1e9
        
        if elapsed >= self.localization_wait_time:
            self.get_logger().info("AMCL convergence time elapsed, starting navigation")
            self.state = RobotState.NAVIGATION
        else:
            remaining = self.localization_wait_time - elapsed
            self.get_logger().info(f"Waiting for AMCL... {remaining:.0f}s remaining")

    def handle_navigation(self):
        self.get_logger().info("Publishing START command to /navigation/command")
        msg = String()
        msg.data = "START"
        self.nav_cmd_pub.publish(msg)

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
