import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Class Definition
class puzzlebot_sim(Node):
    def __init__(self):
        super().__init__('dc_motor')

        # Parameters
        self.r = 0.05  # Wheel radius (meters)
        self.whell_base = 0.19  # Distance between wheels (meters)
        self.sample_time = 0.1  # Timer period (seconds)

        # Set the messages
        self.wr_msg = Float32()
        self.wl_msg = Float32()

        # Set variables to be used
        self.thetark = 0.0  # Initial orientation (radians)
        self.xrk = 0.0  # Initial x position (meters)
        self.yrk = 0.0  # Initial y position (meters)
        self.input_v = 0.0  # Linear velocity (m/s)
        self.input_w = 0.0  # Angular velocity (rad/s) - FIXED

        # Declare publishers, subscribers, and timers
        self.input_sub = self.create_subscription(Twist, 'cmd_vel', self.input_callback, 10)
        self.Wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.Wl_pub = self.create_publisher(Float32, 'wl', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_sim', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # Node Started
        self.get_logger().info('Dynamical System Node Started 🚀')

    # Timer Callback
    def timer_cb(self):
        # Calculate linear velocities of the wheels
        v_r = self.input_v + (self.input_w * self.whell_base / 2)
        v_l = self.input_v - (self.input_w * self.whell_base / 2)

        # Calculate angular velocities of the wheels
        wr = v_r / self.r
        wl = v_l / self.r

        # Publish the angular velocities
        self.wr_msg.data = wr
        self.wl_msg.data = wl

        # Calculate the new pose of the robot
        self.thetark += self.r * (wr - wl) / self.whell_base * self.sample_time
        self.xrk += self.r * (wr + wl) / 2 * self.sample_time * np.cos(self.thetark)
        self.yrk += self.r * (wr + wl) / 2 * self.sample_time * np.sin(self.thetark)

        # Create and publish the pose message
        self.pose_msg = PoseStamped()
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg.header.frame_id = 'base_link'
        self.pose_msg.pose.position.x = self.xrk
        self.pose_msg.pose.position.y = self.yrk
        self.pose_msg.pose.position.z = 0.0
        self.pose_msg.pose.orientation.x = 0.0
        self.pose_msg.pose.orientation.y = 0.0
        self.pose_msg.pose.orientation.z = np.sin(self.thetark / 2)
        self.pose_msg.pose.orientation.w = np.cos(self.thetark / 2)

        # Publish the pose and wheel angular velocities
        self.pose_pub.publish(self.pose_msg)
        self.Wr_pub.publish(self.wr_msg)
        self.Wl_pub.publish(self.wl_msg)

        # Create and publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'base_link'  # Parent frame
        odom_msg.child_frame_id = 'odom'  # Child frame

        # Set position and orientation
        odom_msg.pose.pose.position.x = self.xrk
        odom_msg.pose.pose.position.y = self.yrk
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(self.thetark / 2)
        odom_msg.pose.pose.orientation.w = np.cos(self.thetark / 2)

        # Set pose covariance
        odom_msg.pose.covariance = [
            0.00253953567, 0.00020759152, 0.0, 0.0, 0.0, 0.00796400351,
            0.00020759152, 0.00292401754, 0.0, 0.0, 0.0, 0.01098941784,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.00796400351,0.01098941784, 0.0, 0.0, 0.0, 0.12178119415  # Covariance for yaw
        ]

        # Publish odometry
        self.odom_pub.publish(odom_msg)

    # Subscriber Callback
    def input_callback(self, input_sgn):
        # Extract linear and angular velocities from cmd_vel
        self.input_v = input_sgn.linear.x
        self.input_w = input_sgn.angular.z


# Main
def main(args=None):
    rclpy.init(args=args)

    node = puzzlebot_sim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# Execute Node
if __name__ == '__main__':
    main()