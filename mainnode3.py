import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Pose2D
import numpy as np


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        
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
        
        self.subscription_reached_target = self.create_subscription(
            Bool,
            'reached_target',
            self.reached_target_callback,
            qos_profile)
        
        self.subscription_path = self.create_subscription(
            Float64MultiArray,
            'path',
            self.path_callback,
            qos_profile)

        # Publisher for target pose
        self.target_pose_publisher = self.create_publisher(Pose2D, 'target_pose', qos_profile)

        self.path_request_publisher = self.create_publisher(Bool, 'path_request', qos_profile)

        self.sent_path_request = False

        # Grid parameters
        self.grid_size = 0.05
        self.xlim = [-2, 6]
        self.ylim = [-2, 2]
        self.grid_shape = (int((self.ylim[1] - self.ylim[0]) / self.grid_size), 
                           int((self.xlim[1] - self.xlim[0]) / self.grid_size))
        self.occupancy_map = np.zeros(self.grid_shape)

        self.robot_pose = None
        self.path = []
        self.current_target_index = 0

    def occupancy_map_callback(self, msg):
        self.occupancy_map = np.array(msg.data).reshape(self.grid_shape)

    def robot_pose_callback(self, msg):
        self.robot_pose = (msg.x, msg.y, msg.theta)

        if self.path == []:
            self.path_request()
        else:
            self.check_path()
            self.sent_path_request = False
    
    def path_request(self):
        if not self.sent_path_request :
            msg = Bool()
            msg.data = True
            self.path_request_publisher.publish(msg)
            
            self.get_logger().info("send path_request")
            self.sent_path_request = True

    def path_callback(self, msg):
        path_data = np.array(msg.data)
        num_points = path_data.size // 2
        self.path = path_data.reshape((num_points, 2)).tolist()
        self.get_logger().info(f"Path received \n{self.path}\n")
        self.current_target_index = 1


    def check_path(self):
        if self.current_target_index <= (len(self.path)) :

            for i in range(self.current_target_index , min(self.current_target_index + int(len(self.path)/2)  , len(self.path)), 1):
                #current_target = self.path[self.current_target_index]

                current_target = self.path[self.current_target_index]
                t0 = self.path[i-1]
                t1 = self.path[i]

                if self.is_collision_free(t0, t1):
                    # Publish the next target pose
                    target_pose_msg = Pose2D()
                    target_pose_msg.x, target_pose_msg.y = current_target
                    self.target_pose_publisher.publish(target_pose_msg)
                else:
                    # If there's an obstacle, publish the current pose and stop
                    current_pose_msg = Pose2D()
                    current_pose_msg.x, current_pose_msg.y = self.robot_pose[:2]
                    self.target_pose_publisher.publish(current_pose_msg) # stop
                    self.path = []

                    # Recalculate the path
                    self.get_logger().info("Obstacle detected, requesting path.")
                    break



    def reached_target_callback(self, msg):
        self.target_reached = msg.data

        if self.target_reached :
            self.current_target_index += 1
            if self.current_target_index < (len(self.path)) :
                self.get_logger().info(f"Target reached, moving to the next target. {self.current_target_index} / {(len(self.path))}")
            elif len(self.path) != 0 :
                    self.get_logger().info(f"Finish!!!!!!!! {self.current_target_index} / {(len(self.path))}")  


    def is_collision_free(self, from_node, to_node):
        x0, y0 = self.world_to_grid(from_node)
        x1, y1 = self.world_to_grid(to_node)
        
        if not (0 <= x1 < self.grid_shape[1] and 0 <= y1 < self.grid_shape[0]):
            return False  # Out of bounds

        # Bresenham's line algorithm to check for collisions along the path
        points = self.bresenham(x0, y0, x1, y1)
        for point in points:
            if self.occupancy_map[point[1], point[0]] > 0:  # Obstacle
                return False
        return True

    def world_to_grid(self, node):
        x = int((node[0] - self.xlim[0]) / self.grid_size)
        y = int((node[1] - self.ylim[0]) / self.grid_size)
        return x, y

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

    def get_coords(self):
        return (self.x, self.y)

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()