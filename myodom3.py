import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class MyOdom(Node):
    def __init__(self):
        super().__init__('myodom_node')
        
        # Create a QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscription for odometry and velocity
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile
        )
        
        self.subscription_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vel_callback,
            qos_profile
        )

        self.odom_pub = self.create_publisher(Pose2D, 'myodom', qos_profile)

        # Initialize position and orientation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.orientation_angle = 0.0
        
        self.last_time = self.get_clock().now()
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0

        # Create a timer that calls the update method every 0.1 seconds
        self.timer = self.create_timer(0.02, self.update_position)

        self.init_error = False
        self.orientation_angle_error = 0

    def update_position(self):
        # Calculate dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds

        # Update position using last known velocities
        self.robot_x += self.last_linear_velocity * dt * np.cos(self.orientation_angle)
        self.robot_y += self.last_linear_velocity * dt * np.sin(self.orientation_angle)

        # Publish updated position
        pose2d = Pose2D()
        pose2d.x = self.robot_x
        pose2d.y = self.robot_y
        pose2d.theta = self.orientation_angle

        self.odom_pub.publish(pose2d)
        self.get_logger().info(f"Publish | myodom ({self.robot_x:.3f}, {self.robot_y:.3f}, {self.orientation_angle:.3f})")
        self.last_time = current_time

    def vel_callback(self, msg):
        # Save the last linear and angular velocity received
        self.last_linear_velocity = msg.linear.x
        self.last_angular_velocity = msg.angular.z

    def odom_callback(self, msg):

        # Update orientation from odometry if needed
        msg_orientation = msg.pose.pose.orientation
        self.orientation_angle = euler_from_quaternion(
            msg_orientation.x, 
            msg_orientation.y, 
            msg_orientation.z, 
            msg_orientation.w
        )[2]

        if not self.init_error :
            self.orientation_angle_error =  self.orientation_angle
            self.init_error = True

        else :
            self.orientation_angle -= self.orientation_angle_error



def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    myodom_node = MyOdom()
    rclpy.spin(myodom_node)
    myodom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
