import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose2D
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math


class OccupancyMap(Node):
    def __init__(self):
        super().__init__('ocgm_node')
        
        # Create a QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            qos_profile)
        
        self.subscription_odom = self.create_subscription(
            Pose2D,
            'myodom',
            self.odom_callback,
            qos_profile)
        
        
        # Create publishers for occupancy map and robot pose
        self.map_publisher = self.create_publisher(Float64MultiArray, 'occupancy_map', qos_profile)
        self.map_for_path_planing_publisher = self.create_publisher(Float64MultiArray, 'occupancy_map_for_path_planing', qos_profile)
        self.pose_publisher = self.create_publisher(Pose2D, 'robot_pose', qos_profile)

        # Initialize robot position and orientation
        self.robot_x, self.robot_y, self.orientation_angle = 0.0, 0.0, 0.0

        self.init_error = False
        self.error_x, self.error_y, self.error_orientation_angle = 0.0, 0.0, 0.0
        
        # Set up the grid
        self.grid_size = 0.05
        self.xlim = [-2, 6]
        self.ylim = [-2, 2]
        self.grid_shape = (
            int((self.ylim[1] - self.ylim[0]) / self.grid_size), 
            int((self.xlim[1] - self.xlim[0]) / self.grid_size)
        )
        
        self.occupancy_map = np.zeros(self.grid_shape)
        self.expanded_map = np.zeros(self.grid_shape)


    def odom_callback(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.orientation_angle = msg.theta
        #print(self.robot_x,self.robot_y,self.orientation_angle)
        
        # Publish the robot pose
        pose_msg = Pose2D()
        pose_msg.x = self.robot_x
        pose_msg.y = self.robot_y
        pose_msg.theta = self.orientation_angle
    
        self.pose_publisher.publish(msg)

        print(f"Publish | robot_pose ({self.robot_x:.3f}, {self.robot_y:.3f}, {self.orientation_angle:.3f})")
    

    def expand_obstacles(self, occupancy_map, expansion_cells):
        expanded_map = np.zeros_like(occupancy_map)
        obstacle_mask = (occupancy_map == 1)
        
        for i in range(occupancy_map.shape[0]):
            for j in range(occupancy_map.shape[1]):
                if obstacle_mask[i, j]:
                    expanded_map[i, j] = 1  # Mark the obstacle cell itself
                    
                    # Expand to adjacent cells
                    for di in range(-expansion_cells, expansion_cells + 1):
                        for dj in range(-expansion_cells, expansion_cells + 1):
                            if 0 <= i + di < occupancy_map.shape[0] and 0 <= j + dj < occupancy_map.shape[1]:
                                expanded_map[i + di, j + dj] = 2  # Use a different value to represent expanded cells
        return expanded_map

    def laser_scan_callback(self, msg):
        # Extract LaserScan data
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        distances = np.array(msg.ranges)

        # Filter out invalid measurements
        valid = np.isfinite(distances) & (distances > 0)
        angles = angles[valid]
        distances = distances[valid]

        if len(distances) > 0:
            # Calculate x and y coordinates of the LiDAR points
            lidar_x = self.robot_x + distances * np.cos(angles + self.orientation_angle)
            lidar_y = self.robot_y + distances * np.sin(angles + self.orientation_angle)

            # Update occupancy map
            self.occupancy_map.fill(0)
            grid_x = ((lidar_x - self.xlim[0]) / self.grid_size).astype(int)
            grid_y = ((lidar_y - self.ylim[0]) / self.grid_size).astype(int)
            
            valid_indices = (0 <= grid_x) & (grid_x < self.grid_shape[1]) & (0 <= grid_y) & (grid_y < self.grid_shape[0])
            self.occupancy_map[grid_y[valid_indices], grid_x[valid_indices]] = 1

                ###### แถวนี้แหละะะะะะ bug save map  

        # occupancy_map_for_path_planing
        self.expanded_map_for_path_planing = self.expand_obstacles(self.occupancy_map, expansion_cells=2)
        map_msg = Float64MultiArray()
        map_msg.data = self.expanded_map_for_path_planing.ravel().tolist()
        self.map_for_path_planing_publisher.publish(map_msg)

        # occupancy_map
        self.expanded_map = self.expand_obstacles(self.occupancy_map, expansion_cells=1)
        map_msg = Float64MultiArray()
        map_msg.data = self.expanded_map.ravel().tolist()
        self.map_publisher.publish(map_msg)

        print(f"Publish | occupancy_map")


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
