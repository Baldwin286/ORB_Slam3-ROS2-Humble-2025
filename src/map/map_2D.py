import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib.pyplot as plt
import numpy as np

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')

        self.subscription_map = self.create_subscription(
            PointCloud2,
            '/orb_slam3/map_points',  
            self.map_callback,
            10
        )

        self.subscription_pose = self.create_subscription(
            PoseStamped,
            '/orb_slam3/pose',  
            self.pose_callback,
            10
        )

        self.x_vals = []
        self.y_vals = []
        self.pose_x_vals = []
        self.pose_y_vals = []

        self.fig, self.ax = plt.subplots()
        self.ax.set_title("ORB-SLAM3 Map 2D")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")

        plt.ion()  
        plt.show()

    def map_callback(self, msg):
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        for point in pc_data:
            self.x_vals.append(point[0])
            self.y_vals.append(point[1])

        self.plot_map()

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.pose_x_vals.append(x)
        self.pose_y_vals.append(y)

        self.plot_map()

    def plot_map(self):
        self.ax.clear()

        self.ax.scatter(self.x_vals, self.y_vals, s=1, c='r', label="Map Points")

        self.ax.plot(self.pose_x_vals, self.pose_y_vals, c='b', label="Camera Trajectory")

        self.ax.set_title("ORB-SLAM3 Map 2D")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()

        plt.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    map_visualizer = MapVisualizer()

    try:
        rclpy.spin(map_visualizer)
    except KeyboardInterrupt:
        pass

    map_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
