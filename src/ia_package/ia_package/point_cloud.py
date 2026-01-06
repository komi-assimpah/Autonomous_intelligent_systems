import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
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
    def __init__(self, r=0.0, g=0.0, b=0.0, a=0.0):
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
        self.color = Color()
        self.uv = Vector2()
        self.semantic_classes = [
            {'name': 'Background',   'color': Color(255, 102, 102, 1.0)},
            {'name': 'Aeroplane',    'color': Color(255, 178, 102, 1.0)},
            {'name': 'Bicycle',      'color': Color(255, 255, 102, 1.0)},
            {'name': 'Bird',         'color': Color(178, 255, 102, 1.0)},
            {'name': 'Boat',         'color': Color(102, 255, 102, 1.0)},
            {'name': 'Bottle',       'color': Color(255, 0, 0, 1.0)},
            {'name': 'Bus',          'color': Color(102, 255, 255, 1.0)},
            {'name': 'Car',          'color': Color(102, 178, 255, 1.0)},
            {'name': 'Cat',          'color': Color(102, 102, 255, 1.0)},
            {'name': 'Chair',        'color': Color(124, 252, 0, 1.0)},
            {'name': 'Cow',          'color': Color(255, 102, 255, 1.0)},
            {'name': 'Diningtable',  'color': Color(255, 102, 178, 1.0)},
            {'name': 'Dog',          'color': Color(192, 192, 192, 1.0)},
            {'name': 'Horse',        'color': Color(160, 82, 45, 1.0)},
            {'name': 'Motorbike',    'color': Color(188, 143, 143, 1.0)},
            {'name': 'Person',       'color': Color(0, 0, 255, 1.0)},
            {'name': 'Pottedplant',  'color': Color(0, 250, 154, 1.0)},
            {'name': 'Sheep',        'color': Color(255, 250, 205, 1.0)},
            {'name': 'Sofa',         'color': Color(210, 180, 140, 1.0)},
            {'name': 'Train',        'color': Color(0, 0, 0, 1.0)},
            {'name': 'Tvmonitor',    'color': Color(30, 144, 255, 1.0)},
            {'name': 'Unknown',      'color': Color(255, 255, 255, 1.0)},
        ]

    def set_color(self, color_intrinsics, raw_rgb, raw_semantics, use_semantics):

        if color_intrinsics is None:
            return

        x = min(max(int(self.uv.x), 0), int(color_intrinsics['width']) - 1)
        y = min(max(int(self.uv.y), 0), int(color_intrinsics['height']) - 1)

        idx = int(x + (y * int(color_intrinsics['width'])))

        if use_semantics and raw_semantics is not None:
             try:
                label_val = raw_semantics[int(y), int(x)]
                if label_val > 100:
                    self.label_index = 15 
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

        fx = color_intrinsics['fx'] if color_intrinsics else 1.0
        fy = color_intrinsics['fy'] if color_intrinsics else 1.0
        cx = color_intrinsics['cx'] if color_intrinsics else 0.0
        cy = color_intrinsics['cy'] if color_intrinsics else 0.0
        
        for p in data:
            point = Point()
            point.xyz.x = p[0]
            point.xyz.y = p[1]
            point.xyz.z = p[2]
            
            if point.xyz.z > 0:
                point.uv.x = (point.xyz.x * fx / point.xyz.z) + cx
                point.uv.y = (point.xyz.y * fy / point.xyz.z) + cy
            
            point.set_color(color_intrinsics, raw_rgb, raw_semantics, use_semantics)
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
        self.use_semantics = False 
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

        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_zlim(0, 3.0)

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


class VisualizerROS(Node):
    def __init__(self, plot_manager):
        super().__init__('point_cloud_visualizer')
        self.plot_manager = plot_manager
        self.br = CvBridge()
        
        self.create_subscription(PointCloud2, '/depth_camera/points', self.cloud_callback, 10)
        self.create_subscription(Image, '/inference/mask', self.mask_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)

    def cloud_callback(self, msg):
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = list(gen)
        
        stride = 50 
        points = points[::stride]
        
        self.plot_manager.update_data(
            points, 
            self.plot_manager.color_intrinsics, 
            self.plot_manager.raw_semantics
        )

    def mask_callback(self, msg):
        try:
            cv_mask = self.br.imgmsg_to_cv2(msg, "mono8")
            self.plot_manager.raw_semantics = cv_mask
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