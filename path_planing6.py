import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray , Bool
from geometry_msgs.msg import Pose2D
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import time

class path_planing(Node):
    def __init__(self):
        super().__init__('path_planing_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription_map = self.create_subscription(
            Float64MultiArray,
            'occupancy_map_for_path_planing',
            self.occupancy_map_callback,
            qos_profile)

        self.subscription_pose = self.create_subscription(
            Pose2D,
            'robot_pose',
            self.robot_pose_callback,
            qos_profile)
        
        self.subscription_request = self.create_subscription(
            Bool,
            'path_request',
            self.path_request_callback,
            qos_profile
        )

        self.path_publisher = self.create_publisher(
            Float64MultiArray,
            'path',
            qos_profile
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0

        self.grid_size = 0.05
        self.xlim = [-2, 6]
        self.ylim = [-2, 2]
        self.grid_shape = (int((self.ylim[1] - self.ylim[0]) / self.grid_size), 
                               int((self.xlim[1] - self.xlim[0]) / self.grid_size))
        self.occupancy_map = np.zeros(self.grid_shape)

        self.map_received = True
        self.pose_received = True
        self.find_path = True
        self.distance_node = 4

        print('Start path planing')
        gx = float(input("Goal x : "))
        gy = float(input("Goal Y : "))
        self.goal = [gx, gy]
        print('Run Mainnode to start...')



    def path_request_callback(self, msg):
        if msg.data:
            self.map_received = False
            self.pose_received = False
            self.find_path = False

    def occupancy_map_callback(self, msg):
        if not self.map_received:
            self.occupancy_map = np.array(msg.data).reshape(self.grid_shape)
            self.map_received = True
            print('map_received')

    def robot_pose_callback(self, msg):
        if not self.pose_received:
            self.current_x = msg.x
            self.current_y = msg.y
            self.current_orientation = msg.theta
            self.pose_received = True
            print('pose_received')

        if self.pose_received and self.map_received and (not self.find_path):
            self.find_path = True
            print('Finding path...')
            path , world_path = self.calculate_path([self.current_x, self.current_y], self.goal)

            if world_path is not None:
                self.publish_path(world_path)
                print(f"Path found:{len(world_path)}point {world_path}")
            else:
                print("No path found.")

            
    def calculate_path(self, robot_position, goal):
        robot_grid = self.position_to_grid(robot_position)
        goal_grid = self.position_to_grid(goal)

        binary_map = np.where(self.occupancy_map > 0, 1, 0)

        print(f"calculate_path from {robot_grid} to {goal_grid}")
        robot_grid = (robot_grid[1],robot_grid[0])
        path = self.a_star(robot_grid, goal_grid, binary_map)

        if path is None:
            return None , None

        flip_path = [(point[1], point[0]) for point in path]

        smoothpath = self.smoothpath(flip_path)########################################

        world_path = []
        for point in smoothpath:
            coor_world = self.grid_to_world(point)
            coor_world[0] = round(coor_world[0],3)
            coor_world[1] = round(coor_world[1],3)
            world_path.append(coor_world)

        world_path.append(self.goal)

        return smoothpath , world_path
    
    ########################
    def smoothpath(self,path):

        if len(path) <= 2:
            return path

        def get_direction(p1, p2):
            """Calculate direction vector between two points"""
            dx = (p2[0] - p1[0]) // 4  # หาร 4 เพื่อให้ได้ unit vector
            dy = (p2[1] - p1[1]) // 4
            return (dx, dy)

        final_path = [path[0]]  # เก็บจุดแรก
        current_direction = None

        for i in range(1, len(path)):
            # จุดแรกให้เก็บ direction ไว้ก่อน
            if i == 1:
                current_direction = get_direction(path[0], path[1])
                continue
                
            new_direction = get_direction(path[i-1], path[i])
            
            # ถ้าทิศทางเปลี่ยน เก็บจุดก่อนหน้า
            if new_direction != current_direction:
                final_path.append(path[i-1])
                current_direction = new_direction

        final_path.append(path[-1])  # เก็บจุดสุดท้าย

        return final_path

    def position_to_grid(self, position):
        x, y = position
        grid_x = int((x - self.xlim[0]) / self.grid_size)
        grid_y = int((y - self.ylim[0]) / self.grid_size)
        return (grid_x, grid_y)

    def a_star(self, start, goal, grid):
        rows, cols = grid.shape
        open_set = set()
        open_set.add(start)
        came_from = {}
        
        g_score = np.full(grid.shape, np.inf)
        g_score[start] = 0
        
        f_score = np.full(grid.shape, np.inf)
        f_score[start] = self.heuristic(start, goal)

        while open_set:
            current = min(open_set, key=lambda pos: f_score[pos])


            step = self.distance_node
            directions = [(-step, -step), (-step, 0), (-step, step),
                          (0, -step),         (0, step),
                          (step, -step), (step, 0), (step, step)]
            
            dis = self.distance((current[1],current[0]), goal)
            if dis <= step:
                return self.retrace_path(came_from, current)

            open_set.remove(current)

            x, y = current

            for dx, dy in directions:
                neighbor = (x + dx, y + dy)
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if self.is_collision_free(current, neighbor):
                        
                        #tentative_g_score = g_score[current] + 1  # Use 1 for uniform cost
                        tentative_g_score = g_score[current] + self.distance(current, neighbor)
                        #print(current, neighbor ,self.distance(current, neighbor))
                        if tentative_g_score < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                            open_set.add(neighbor)

        return None  # No path found
    
    def is_collision_free(self, node1, node2):
        x0, y0 = node1[0], node1[1]
        x1, y1 = node2[0], node2[1]
        for x, y in self.bresenham(x0, y0, x1, y1):
            if self.occupancy_map[x, y] > 0:  # Occupied cell
                return False
        return True

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
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points

    def retrace_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def heuristic(self, node1, node2):
        dx = abs(node1[0] - node2[0])
        dy = abs(node1[1] - node2[1])
        return 0.01 * math.sqrt(dx * dx + dy * dy)
    
        #return self.distance(node1, node2)
        #return 0.2*abs(node1[0] - node2[0]) #+ abs(node1[1] - node2[1]) # Manhattan distance

        #return abs(node1[0] - node2[0]) + 1.8*abs(node1[1] - node2[1])  

    def distance(self, node1, node2):
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)
    
    def grid_to_world(self, grid_coord):
        grid_x, grid_y = grid_coord
        x = grid_x * self.grid_size + self.xlim[0]
        y = grid_y * self.grid_size + self.ylim[0]
        return [x, y]

    def publish_path(self, path):
        msg = Float64MultiArray()
        msg.data = [coord for point in path for coord in point]
        self.path_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = path_planing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
