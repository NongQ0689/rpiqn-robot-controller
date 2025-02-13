import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose2D
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection

class OccupancyMapSubscriber(Node):
    def __init__(self):
        super().__init__('show_ocgm_node')

        # Create a QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscriptions for occupancy map and robot pose
        self.subscription_map = self.create_subscription(
            Float64MultiArray,
            'occupancy_map',
            self.occupancy_map_callback,
            qos_profile)

        self.subscription_pose = self.create_subscription(
            Pose2D,
            'robot_pose',
            self.robot_pose_callback,
            qos_profile)
        
        self.subscription_path = self.create_subscription(
            Float64MultiArray,
            'path',
            self.path_callback,
            qos_profile)

        # Create a figure and axis for plotting
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        plt.ion()
        plt.show()

        # Set up the grid
        self.grid_size = 0.05
        self.xlim = [-2, 6]
        self.ylim = [-2, 2]
        self.grid_shape = (int((self.ylim[1] - self.ylim[0]) / self.grid_size), 
                               int((self.xlim[1] - self.xlim[0]) / self.grid_size))
        self.occupancy_map = np.zeros(self.grid_shape)

        self.create_grid()

        # Initialize patch collection for efficient updating
        self.patches = [patches.Rectangle((self.xlim[0] + j * self.grid_size, self.ylim[0] + i * self.grid_size), 
                                          self.grid_size, self.grid_size) 
                        for i in range(self.grid_shape[0]) 
                        for j in range(self.grid_shape[1])]
        self.patch_collection = PatchCollection(self.patches, cmap=plt.cm.binary, edgecolors='none')
        self.ax.add_collection(self.patch_collection)

        # Robot position and direction
        self.robot_point, = self.ax.plot([], [], 'bo', markersize=10)
        self.robot_arrow = self.ax.arrow(0, 0, 0, 0, head_width=0.1, head_length=0.1, fc='r', ec='r')

        self.node_points, = self.ax.plot([], [], 'bo', markersize=3,alpha=0.5)
        self.path_line, = self.ax.plot([], [], 'g-', lw=2,alpha=0.5)

    def create_grid(self):
        # Create a grid (only once)
        for x in np.arange(self.xlim[0], self.xlim[1] + self.grid_size, self.grid_size):
            self.ax.axvline(x, color='lightgray', lw=0.5)
        for y in np.arange(self.ylim[0], self.ylim[1] + self.grid_size, self.grid_size):
            self.ax.axhline(y, color='lightgray', lw=0.5)
        
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        self.ax.set_aspect('equal')
        self.ax.grid(True)

    def occupancy_map_callback(self, msg):
        self.occupancy_map = np.array(msg.data).reshape(self.grid_shape)
        self.patch_collection.set_array(self.occupancy_map.ravel())

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def robot_pose_callback(self, msg):
        robot_x = msg.x
        robot_y = msg.y
        orientation_angle = msg.theta

        self.robot_point.set_data([robot_x], [robot_y])

        arrow_length = 0.2
        arrow_dx = arrow_length * np.cos(orientation_angle)
        arrow_dy = arrow_length * np.sin(orientation_angle)
        self.robot_arrow.remove()
        self.robot_arrow = self.ax.arrow(robot_x, robot_y, arrow_dx, arrow_dy, 
                                         head_width=0.1, head_length=0.1, fc='r', ec='r')

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    def path_callback(self, msg):
        path_data = np.array(msg.data)
        num_points = path_data.size // 2
        path = path_data.reshape((num_points, 2)).tolist()

        x_coords, y_coords = zip(*path)

        self.node_points.set_data(x_coords, y_coords)
        self.path_line.set_data(x_coords, y_coords)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
