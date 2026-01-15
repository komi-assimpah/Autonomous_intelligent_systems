import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import random

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import threading
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d import axes3d
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
import json
import queue
import math

class Color:
    def __init__(self, r=0.0, g=0.0, b=0.0, a=1.0):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

    def divide_by(self, value):
        if value == 0:
            raise ValueError("Division by 0!")
        self.r /= value
        self.g /= value
        self.b /= value


class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Vector2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


class Point:
    def __init__(self):
        self.label_index = 0
        self.xyz = Vector3()
        self.color = Color(0, 0, 0, 1.0)
        self.uv = Vector2()
        # COCO 80 classes + Background (ID 0)
        self.semantic_classes = [{'name': 'Background', 'color': Color(0, 0, 0, 1.0)}] 
        
        coco_names = [
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
            "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
            "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
            "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
            "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
            "hair drier", "toothbrush"
        ]
        
        random.seed(42) 
        for name in coco_names:
            r = random.randint(50, 255)
            g = random.randint(50, 255)
            b = random.randint(50, 255)
            self.semantic_classes.append({'name': name, 'color': Color(r, g, b, 1.0)})


    def set_color(self, width, height, raw_rgb, raw_semantics, use_semantics):

        x = min(max(int(self.uv.x), 0), int(width) - 1)
        y = min(max(int(self.uv.y), 0), int(height) - 1)

        idx = int(x + (y * int(width)))

        if use_semantics and raw_semantics is not None:
             try:
                label_val = raw_semantics[int(y), int(x)]
                
                if label_val > 0 and label_val < len(self.semantic_classes):
                    self.label_index = int(label_val)
                else:
                    self.label_index = 0
             except:
                self.label_index = 0

             self.color = self.semantic_classes[self.label_index]['color']
             self.color = Color(self.color.r, self.color.g, self.color.b, self.color.a)
             self.color.divide_by(255.0)

        elif raw_rgb is not None:
            pass

    def get_label_name(self, index):
        return self.semantic_classes[index]['name']


class PointCloud:
    def __init__(self, scale):
        self.scale = scale
        self.YPR = Vector3()
        self.points = []

    def to_world_coordinates(self):
        rotation = R.from_euler(
            'xyz', [self.YPR.x, self.YPR.y, self.YPR.z], degrees=True)
        x = np.array([point.xyz.x for point in self.points])
        y = np.array([point.xyz.y for point in self.points])
        z = np.array([point.xyz.z for point in self.points])
        
        if len(x) == 0:
             return np.zeros((0, 3))

        pts = np.vstack((x, y, z)).T

        return rotation.apply(pts)

    def update_from_data(self, data, color_intrinsics, raw_rgb, raw_semantics, use_semantics):
        self.points = []
        
        if not data:
            return

        current_fx = color_intrinsics['fx'] if color_intrinsics else 554.0
        current_fy = color_intrinsics['fy'] if color_intrinsics else 554.0
        current_cx = color_intrinsics['cx'] if color_intrinsics else 320.0
        current_cy = color_intrinsics['cy'] if color_intrinsics else 240.0
        
        intrinsics_width = color_intrinsics['width'] if color_intrinsics else 640
        intrinsics_height = color_intrinsics['height'] if color_intrinsics else 480
        
        # Determine scaling factors
        scale_x = 1.0
        scale_y = 1.0
        
        if raw_semantics is not None:
             mask_h, mask_w = raw_semantics.shape[:2]
             if mask_w != intrinsics_width or mask_h != intrinsics_height:
                 scale_x = mask_w / float(intrinsics_width)
                 scale_y = mask_h / float(intrinsics_height)
                 
                 current_fx *= scale_x
                 current_fy *= scale_y
                 current_cx *= scale_x
                 current_cy *= scale_y
                 
                 intrinsics_width = mask_w
                 intrinsics_height = mask_h

        fx = current_fx
        fy = current_fy
        cx = current_cx
        cy = current_cy
        
        for p in data:
            if not math.isfinite(p[0]) or not math.isfinite(p[1]) or not math.isfinite(p[2]):
                continue

            point = Point()
            point.xyz.x = p[0]
            point.xyz.y = p[1]
            point.xyz.z = p[2]
            
            if point.xyz.x > 0.001:
                z_opt = point.xyz.x
                x_opt = -point.xyz.y
                y_opt = -point.xyz.z

                point.uv.x = (x_opt * fx / z_opt) + cx
                point.uv.y = (y_opt * fy / z_opt) + cy
                
                point.set_color(intrinsics_width, intrinsics_height, raw_rgb, raw_semantics, use_semantics)
                self.points.append(point)

    def get_label_coordinates(self):
        points_dict = {}

        for point in self.points:
            label_index = point.label_index
            if label_index not in points_dict:
                points_dict[label_index] = point

        return points_dict


class Coeffs:
    def __init__(self):
        k1 = 0.0
        k2 = 0.0
        p1 = 0.0
        p2 = 0.0
        k3 = 0.0


class Intrinsics:
    def __init__(self):
        self.width = 0
        self.height = 0
        self.ppx = 0.0
        self.ppy = 0.0
        self.fx = 0.0
        self.fy = 0.0
        self.coeffs = Coeffs()


class PlotManager:
    def __init__(self):
        self.pc = PointCloud(1.0)
        self.color_intrinsics = None
        self.raw_rgb = None
        self.raw_semantics = None
        self.use_semantics = True 
        self.firmware = "ROS2 Node"
        self.computation_time = "N/A"
        self.lock = threading.Lock()
        
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.ax_button = plt.axes([0.7, 0.01, 0.1, 0.075])
        self.button = Button(self.ax_button, 'Change view')
        self.button.on_clicked(self.toggle_color_mode)

    def toggle_color_mode(self, event):
        self.use_semantics = not self.use_semantics

    def update_data(self, points, intrinsics, mask):
        with self.lock:
            self.color_intrinsics = intrinsics
            self.raw_semantics = mask
            self.pc.update_from_data(points, self.color_intrinsics, None, self.raw_semantics, self.use_semantics)

    def update_plot(self, frame):
        try:
            with self.lock:
                 points_copy = self.pc.points[:]
                 use_semantics = self.use_semantics
                 raw_semantics = self.raw_semantics
                 
                 wc_points = self.pc.to_world_coordinates()
                 
                 labels = {}
                 if raw_semantics is not None and use_semantics:
                     labels = self.pc.get_label_coordinates()
            
            xs = [point.xyz.x for point in points_copy]
            if not xs:
                return
                
            x_wc = wc_points[:, 0]
            y_wc = wc_points[:, 1]
            z_wc = wc_points[:, 2]

            colors = [(point.color.r, point.color.g, point.color.b,
                       point.color.a) for point in points_copy]

            fig_width, fig_height = self.fig.get_size_inches() * self.fig.dpi
            size_scale = min(fig_width, fig_height) / 100.0
            sizes = [size_scale for _ in points_copy]

            self.ax.clear()
            self.ax.grid(False)
            self.fig.suptitle(
                f'Real-time Visualization', fontsize=10)

            self.ax.set_xlim(-1.0, 1.0)
            self.ax.set_ylim(-1.0, 1.0)
            self.ax.set_zlim(0, 2.0)

            if raw_semantics is None:
                self.ax.scatter(x_wc, y_wc, z_wc, s=sizes)
            else:
                self.ax.scatter(x_wc, y_wc, z_wc, s=sizes, c=colors)

            if raw_semantics is not None and use_semantics:
                for label, point in labels.items():
                    self.ax.text(point.xyz.x, point.xyz.y, point.xyz.z,
                                 point.get_label_name(label), color='red')
                                 
            self.ax.quiver(0, 0, 0, 0.1, 0, 0, color='r')
            self.ax.quiver(0, 0, 0, 0, 0.1, 0, color='g')
            self.ax.quiver(0, 0, 0, 0, 0, 0.1, color='b')
            
        except Exception as e:
            print(f"Error in update_plot: {e}")


class VisualizerROS(Node):
    def __init__(self, plot_manager):
        super().__init__('point_cloud_visualizer')
        self.plot_manager = plot_manager
        self.br = CvBridge()
        
        self.create_subscription(PointCloud2, '/depth_camera/points', self.cloud_callback, 10)
        self.create_subscription(Image, '/inference/mask', self.mask_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.target_pub = self.create_publisher(PointStamped, '/detected_object/map_pose', 10)
        self.current_pose = None # (x, y, theta)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_pose = (x, y, theta)

    def cloud_callback(self, msg):
        try:
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = list(gen)
            
            stride = 50 
            points = points[::stride]
            
            if self.plot_manager.color_intrinsics is None:
                self.get_logger().warn('Intrinsics missing, using default!', throttle_duration_sec=5.0)
                self.plot_manager.color_intrinsics = {
                    'fx': 554.0, 'fy': 554.0,
                    'cx': 320.0, 'cy': 240.0,
                    'width': 640, 'height': 480
                }

            self.plot_manager.update_data(
                points, 
                self.plot_manager.color_intrinsics, 
                self.plot_manager.raw_semantics
            )

            if self.current_pose is not None:
                robot_x, robot_y, robot_theta = self.current_pose
                
                objects = {}
                for p in self.plot_manager.pc.points:
                    if p.label_index != 0: 
                        if p.label_index not in objects:
                            objects[p.label_index] = []
                        objects[p.label_index].append(p)
                
                for label_idx, obj_points in objects.items():
                    if not obj_points: continue
                    
                    xs = [p.xyz.x for p in obj_points]
                    ys = [p.xyz.y for p in obj_points]
                    
                    avg_x = sum(xs) / len(xs) 
                    avg_y = sum(ys) / len(ys) 
                    
                    global_x = robot_x + (avg_x * math.cos(robot_theta) - avg_y * math.sin(robot_theta))
                    global_y = robot_y + (avg_x * math.sin(robot_theta) + avg_y * math.cos(robot_theta))
                    
                    pt_msg = PointStamped()
                    pt_msg.header.stamp = self.get_clock().now().to_msg()
                    pt_msg.header.frame_id = "map" 
                    pt_msg.point.x = global_x
                    pt_msg.point.y = global_y
                    pt_msg.point.z = 0.0 
                    
                    self.target_pub.publish(pt_msg)
                    
                    label_name = obj_points[0].get_label_name(label_idx)
                    self.get_logger().info(f"Target [{label_name}] at Map: ({global_x:.2f}, {global_y:.2f})", throttle_duration_sec=2.0)

        except Exception as e:
            self.get_logger().error(f'Error in cloud_callback: {e}')

    def mask_callback(self, msg):
        try:
            cv_mask = self.br.imgmsg_to_cv2(msg, "mono8")
            self.plot_manager.raw_semantics = cv_mask
            
            # Debug mask values
            # unique_vals = np.unique(cv_mask)
            # if len(unique_vals) > 0:
            #     self.get_logger().info(f'Mask unique values: {unique_vals}', throttle_duration_sec=2.0)
                
        except Exception as e:
            self.get_logger().error(f'Mask error: {e}')

    def info_callback(self, msg):
        if self.plot_manager.color_intrinsics is None:
            k = np.array(msg.k).reshape(3, 3)
            self.plot_manager.color_intrinsics = {
                'fx': k[0, 0], 'fy': k[1, 1],
                'cx': k[0, 2], 'cy': k[1, 2],
                'width': msg.width, 'height': msg.height
            }

def main(args=None):
    rclpy.init(args=args)
    
    plot_manager = PlotManager()
    node = VisualizerROS(plot_manager)
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    try:
        ani = FuncAnimation(plot_manager.fig, plot_manager.update_plot,
                            interval=100, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()