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

        self.flag = 0

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

        # Extract yaw from quaternion
        yaw = 2 * np.arctan2(odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)

        # State vector (x, y, yaw)
        state_vector = np.array([self.xrk, self.yrk, yaw])

        # Initial covariance matrix (3x3)
        cov_matrix = np.array([
            [0.0, 0.0, 0.0],  # Initial uncertainty in x
            [0.0, 0.0, 0.0],  # Initial uncertainty in y
            [0.0, 0.0, 0.0]   # Initial uncertainty in yaw
        ])

        # Process noise covariance matrix (Q)
        q = np.array([
            [0.0253953567, 0.000020759152, 0.000796400351],
            [0.000020759152, 0.292401754, 0.001098941784],
            [0.000796400351, 0.001098941784, 0.012178119415]
        ])

        # Update the state vector based on the motion model miu
        state_vector[0] += self.sample_time * self.input_v * np.cos(state_vector[2])  # x
        state_vector[1] += self.sample_time * self.input_v * np.sin(state_vector[2])  # y
        state_vector[2] += self.sample_time * self.input_w                            # yaw

        # Jacobian matrix (H) for the motion model
        h = np.array([
            [1, 0, -self.sample_time * self.input_v * np.sin(state_vector[2])],
            [0, 1,  self.sample_time * self.input_v * np.cos(state_vector[2])],
            [0, 0,  1]
        ])

        # Transpose of the Jacobian matrix (H.T)
        h_t = h.T

        # Propagate the covariance matrix
        if self.flag == 0:
            new_cov_matrix = h @ cov_matrix @ h_t + q
        else:
            # Update the covariance matrix with the new state
            new_cov_matrix = h @ new_cov_matrix @ h_t + q

        # Update the odometry message
        odom_msg.pose.pose.position.x = state_vector[0]
        odom_msg.pose.pose.position.y = state_vector[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(state_vector[2] / 2)
        odom_msg.pose.pose.orientation.w = np.cos(state_vector[2] / 2)

        # Flatten the covariance matrix for the odometry message
        odom_msg.pose.covariance = [
                    new_cov_matrix[0, 0], new_cov_matrix[0, 1], 0.0, 0.0, 0.0, new_cov_matrix[0, 2],
                    new_cov_matrix[1, 0], new_cov_matrix[1, 1], 0.0, 0.0, 0.0, new_cov_matrix[1, 2],
                    0.0,              0.0,              0.0, 0.0, 0.0, 0.0,
                    0.0,              0.0,              0.0, 0.0, 0.0, 0.0,
                    0.0,              0.0,              0.0, 0.0, 0.0, 0.0,
                    new_cov_matrix[2, 0], new_cov_matrix[2, 1], 0.0, 0.0, 0.0, new_cov_matrix[2, 2]
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