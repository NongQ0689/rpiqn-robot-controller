import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool
import numpy as np
from math import atan2, sqrt, pi

class Bug0Node(Node):
    def __init__(self):
        super().__init__('bug0_node')

        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.reached_target_publisher = self.create_publisher(Bool, 'reached_target', 10)
        self.sent = False

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription_pose = self.create_subscription(
            Pose2D,
            'robot_pose',
            self.robot_pose_callback,
            qos_profile)
        
        self.subscription_target_pose = self.create_subscription(
            Pose2D,
            'target_pose',
            self.target_pose_callback,
            qos_profile)

        # Initial values
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0

        self.target_pose = Pose2D()

        self.linear_speed = 0.2
        self.angular_speed = 1

        self.linear_speed_min = 0.1
        self.angular_speed_min = 0.3

        # Current velocities and acceleration control
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.acceleration_rate = 0.005  # m/s per iteration

    def target_pose_callback(self, msg):
        # Update target pose when received
        self.target_pose = msg
        self.get_logger().info(f'Received target pose: {msg.x}, {msg.y}, {msg.theta}')

    def robot_pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_orientation = msg.theta
        
        # Compute distance and angle to the target
        dx = self.target_pose.x - self.current_x
        dy = self.target_pose.y - self.current_y
        distance = sqrt(dx**2 + dy**2)
        angle_to_target = atan2(dy, dx)
        angle_diff = angle_to_target - self.current_orientation
        angle_diff = (angle_diff + pi) % (2 * pi) - pi  # Normalize to [-pi, pi]

        # Define tolerances
        angle_tolerance = 0.08  # radians
        angle_tolerance_max = 0.15  # radians
        distance_tolerance = 0.05  # meters

        # Create a Twist message
        msg = Twist()
        msg_reached_target = Bool()

        # Publish the message
        if distance > distance_tolerance:
            if abs(angle_diff) > angle_tolerance_max:
                # Rotate towards the target
                direction = angle_diff / abs(angle_diff)
                w = self.angular_speed * abs(angle_diff)
                w = min(w, self.angular_speed)
                w = max(w, self.angular_speed_min)

                msg.linear.x = 0.0
                msg.angular.z = w * direction

            elif abs(angle_diff) > angle_tolerance:
                direction = angle_diff / abs(angle_diff)
                w = self.angular_speed * abs(angle_diff)
                w = min(w, self.angular_speed)
                w = max(w, self.angular_speed_min)

                # Move forward towards the target with curvature control
                v = self.linear_speed * distance * 2 
                v = min(v, self.linear_speed)
                v = max(v, self.linear_speed_min)

                # Apply acceleration control
                if self.current_linear_speed < v:
                    self.current_linear_speed += self.acceleration_rate
                else:
                    self.current_linear_speed = v

                msg.linear.x = self.current_linear_speed
                msg.angular.z = w * direction
            
            else:
                # Move forward towards the target
                v = self.linear_speed * distance * 2
                v = min(v, self.linear_speed)
                v = max(v, self.linear_speed_min)

                # Apply acceleration control
                if self.current_linear_speed < v:
                    self.current_linear_speed += self.acceleration_rate
                else:
                    self.current_linear_speed = v

                msg.linear.x = self.current_linear_speed
                msg.angular.z = 0.0

            msg_reached_target.data = False
            self.reached_target_publisher.publish(msg_reached_target)
            self.sent = False

        else:
            if not self.sent:
                msg_reached_target.data = True
                self.reached_target_publisher.publish(msg_reached_target)
                self.sent = True

            print('end')
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        print(f'Go to {self.target_pose.x} , {self.target_pose.y} ')
        print(f'Now   {self.current_x:.3f} , {self.current_y:.3f} , {self.current_orientation:.3f} ')
        print(f'Diff  {dx:.3f} , {dy:.3f} , {angle_diff:.3f}')
        print(f'msg   {msg.linear.x:.3f} , {msg.angular.z:.3f}\n')

        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Bug0Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
