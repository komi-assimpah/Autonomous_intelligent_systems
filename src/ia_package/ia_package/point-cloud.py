import json
import queue
import numpy as np
import math

import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from mpl_toolkits.mplot3d import axes3d
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

TOPIC_PC = 'IntelD435/point_cloud_data'
TOPIC_COLOR_INTRINSICS = 'IntelD435/color_intrinsics'
TOPIC_COLOR_FRAME = 'IntelD435/color_frame'
TOPIC_FIRMWARE = 'IntelD435/firmware'
TOPIC_COMPUTATION_TIME = 'IntelD435/pc_computation_time'
TOPIC_SEMANTIC_SEGMENTATION_MAP = 'EdgeTPU/semantic_segmentation_map'


class MQTT:
    def __init__(self, broker, port):
        self.client = mqtt.Client()
        self.subscribed_topics = []
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.broker = broker

        try:
            self.client.connect(str(broker), port, 60)
            print('Client connected to MQTT @' + str(broker) + ':' + str(port))
            self.client.loop_start()
        except:
            print('Problem while connecting to the MQTT broker @' +
                  str(broker) + ':' + str(port))
            print('Please ensure the broker is up and running.')

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            if self.subscribed_topics:
                for topic in self.subscribed_topics:
                    print('subscribed to ' + str(topic))
                    self.client.subscribe(str(topic))
        else:
            print("MQTT connection error with code=", rc)

    def on_disconnect(self, userdata, flags, rc):
        print("Disconnected from MQTT broker @" + str(self.broker))
        for t in self.subscribed_topics:
            self.client.unsubscribe(t)


class MQTTSubscriber(MQTT):
    def __init__(self, broker, port, redirect):
        super().__init__(broker, port)
        self.client.on_message = self.on_message
        self.redirect = redirect

    def Subscribe(self, topic):
        if str(topic) not in self.subscribed_topics:
            self.subscribed_topics.append(str(topic))
            self.client.subscribe(str(topic))

    def on_message(self, client, userdata, msg):
        self.redirect(msg)


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
            {'name': 'Dog',          'color': Color(209, 103, 42, 1.0)}, # Orange pour Dog
            {'name': 'Horse',        'color': Color(160, 82, 45, 1.0)},
            {'name': 'Motorbike',    'color': Color(188, 143, 143, 1.0)},
            {'name': 'Person',       'color': Color(0, 0, 255, 1.0)},  # Bleu pour Person
            {'name': 'Pottedplant',  'color': Color(0, 250, 154, 1.0)},
            {'name': 'Sheep',        'color': Color(255, 250, 205, 1.0)},
            {'name': 'Sofa',         'color': Color(210, 180, 140, 1.0)},
            {'name': 'Train',        'color': Color(0, 0, 0, 1.0)},
            {'name': 'Tvmonitor',    'color': Color(30, 144, 255, 1.0)},
            {'name': 'Unknown',      'color': Color(255, 255, 255, 1.0)},
        ]

    def set_color(self, color_intrinsics, raw_rgb, raw_semantics, use_semantics):
        # Permet de voir instantanément dans RViz2 si l'objet cherché est détecté avec la couleur associée

        if color_intrinsics is None:
            return

        x = min(max(
            self.uv.x * int(color_intrinsics['width']), 0), int(color_intrinsics['width']) - 1)
        y = min(max(
            self.uv.y * int(color_intrinsics['height']), 0), int(color_intrinsics['height']) - 1)

        if use_semantics and raw_semantics is not None:
            idx = int(x + (y * int(color_intrinsics['width'])))
            self.label_index = raw_semantics[idx]
            self.color = self.semantic_classes[self.label_index]['color']
            self.color.divide_by(255.0)
        elif raw_rgb is not None:
            idx = int((x * 3) + (y * int(color_intrinsics['width']) * 3))
            self.color.r = raw_rgb[idx]
            self.color.g = raw_rgb[idx+1]
            self.color.b = raw_rgb[idx+2]
            self.color.a = 1.0
            self.color.divide_by(255.0)

    def get_label_name(self, index):
        return self.semantic_classes[index]['name']


class PointCloud:
    def __init__(self, scale):
        self.scale = scale
        self.YPR = Vector3()
        self.points = []

    def to_world_coordinates(self):
        # Transforme les points du repère caméra vers le repère monde.

        rotation = R.from_euler(
            'xyz', [self.YPR.x, self.YPR.y, self.YPR.z], degrees=True)
        x = np.array([point.xyz.x for point in self.points])
        y = np.array([point.xyz.y for point in self.points])
        z = np.array([point.xyz.z for point in self.points])
        pts = np.vstack((x, y, z)).T

        return rotation.apply(pts)

    def update_from_data(self, data, color_intrinsics, raw_rgb, raw_semantics, use_semantics):
        # Met à jour le nuage de points à partir de données brutes.

        self.points = []

        # Extrinsics (angles are given in degrees)
        self.YPR.x = data[2]
        self.YPR.y = data[0]
        self.YPR.z = data[1]

        for i in range(3, len(data) - 3, 5):
            point = Point()
            point.xyz.x = data[i+2] * self.scale
            point.xyz.y = data[i] * self.scale
            point.xyz.z = -data[i+1] * self.scale
            point.uv = Vector2(data[i+3], data[i+4])

            point.set_color(color_intrinsics, raw_rgb,
                            raw_semantics, use_semantics)

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
        self.firmware = None
        self.computation_time = -1.0
        self.q = queue.Queue()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.ax_button = plt.axes([0.7, 0.01, 0.1, 0.075])
        self.button = Button(self.ax_button, 'Change view')
        self.button.on_clicked(self.toggle_color_mode)

    def toggle_color_mode(self, event):
        self.use_semantics = not self.use_semantics

    def on_data(self, message):
        if TOPIC_FIRMWARE in message.topic:
            self.firmware = message.payload.decode('utf-8')

        if TOPIC_COMPUTATION_TIME in message.topic:
            self.computation_time = message.payload.decode('utf-8')

        if self.q.empty():
            if TOPIC_PC in message.topic:
                msg = message.payload.decode('utf-8')
                data = list(map(float, msg.split(',')))
                if data:
                    self.q.put(data)

            if TOPIC_COLOR_INTRINSICS in message.topic:
                self.color_intrinsics = json.loads(
                    message.payload.decode('utf-8'))

            if TOPIC_COLOR_FRAME in message.topic:
                self.raw_rgb = message.payload

            if TOPIC_SEMANTIC_SEGMENTATION_MAP in message.topic:
                self.raw_semantics = message.payload

    def update_plot(self, frame):
        if not self.q.empty():
            # -----
            # Display (colored) point cloud
            # -----
            data = self.q.get()
            self.pc.update_from_data(
                data, self.color_intrinsics, self.raw_rgb, self.raw_semantics, self.use_semantics)
            xs = [point.xyz.x for point in self.pc.points]
            ys = [point.xyz.y for point in self.pc.points]
            zs = [point.xyz.z for point in self.pc.points]

            colors = [(point.color.r, point.color.g, point.color.b,
                       point.color.a) for point in self.pc.points]

            fig_width, fig_height = self.fig.get_size_inches() * self.fig.dpi
            size_scale = min(fig_width, fig_height) / 100.0
            sizes = [size_scale for _ in self.pc.points]

            self.ax.clear()
            self.ax.grid(False)
            self.fig.suptitle(
                f'Firmware - {self.firmware}\nComputation time - {self.computation_time} ms', fontsize=10)

            self.ax.set_xlim(-0.5, 0.5)
            self.ax.set_ylim(-0.5, 0.5)
            self.ax.set_zlim(-0.5, 0.5)

            wc_points = self.pc.to_world_coordinates()
            x_wc = wc_points[:, 0]
            y_wc = wc_points[:, 1]
            z_wc = wc_points[:, 2]

            if self.raw_rgb is None and self.raw_semantics is None:
                self.ax.scatter(x_wc, y_wc, z_wc, s=sizes)
            else:
                self.ax.scatter(x_wc, y_wc, z_wc, s=sizes, c=colors)

            # -----
            # DISPLAY SEMANTIC MAPPING
            # -----
            if self.raw_semantics is not None and self.use_semantics:
                labels = self.pc.get_label_coordinates()
                for label, point in labels.items():
                    self.ax.text(point.xyz.x, point.xyz.y, point.xyz.z,
                                 point.get_label_name(label), color='red')

            # -----
            # Display camera pose from IMU
            # -----
            rotation = R.from_euler(
                'xyz', [self.pc.YPR.x, self.pc.YPR.y, self.pc.YPR.z], degrees=True)
            # Get the rotation matrix
            rotation_matrix = rotation.as_matrix()
            # Define the unit vectors for the axes
            x_axis = np.array([1, 0, 0])
            y_axis = np.array([0, 1, 0])
            z_axis = np.array([0, 0, 1])
            # Rotate the unit vectors using the rotation matrix
            rotated_x_axis = rotation_matrix.dot(x_axis)
            rotated_y_axis = rotation_matrix.dot(y_axis)
            rotated_z_axis = rotation_matrix.dot(z_axis)
            # Define the origin
            origin = np.array([0, 0, 0])  # Camera origin in world coordinates.
            # Plot the arrows
            self.ax.quiver(*origin, *rotated_x_axis, color='r', length=0.3)
            self.ax.quiver(*origin, *rotated_y_axis, color='g', length=0.3)
            self.ax.quiver(*origin, *rotated_z_axis, color='b', length=0.3)
            # Add text to indicate angles at the ends of the arrows
            # self.ax.text(*rotated_x_axis, 'Roll: {:.2f}°'.format(self.pc.YPR.x), color='r')
            # self.ax.text(*rotated_y_axis, 'Pitch: {:.2f}°'.format(self.pc.YPR.y), color='g')
            # self.ax.text(*rotated_z_axis, 'Yaw: {:.2f}°'.format(self.pc.YPR.z), color='b')


if __name__ == "__main__":
    plot_manager = PlotManager()
    pc_subscriber = MQTTSubscriber('192.168.2.7', 1883, plot_manager.on_data)
    pc_subscriber.Subscribe(TOPIC_FIRMWARE)
    pc_subscriber.Subscribe(TOPIC_PC)
    pc_subscriber.Subscribe(TOPIC_COMPUTATION_TIME)
    pc_subscriber.Subscribe(TOPIC_COLOR_INTRINSICS)
    pc_subscriber.Subscribe(TOPIC_COLOR_FRAME)
    pc_subscriber.Subscribe(TOPIC_SEMANTIC_SEGMENTATION_MAP)

    ani = FuncAnimation(plot_manager.fig, plot_manager.update_plot,
                        interval=100, cache_frame_data=False)
    plt.show()
