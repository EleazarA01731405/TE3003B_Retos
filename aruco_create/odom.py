import rclpy
from rclpy.node import Node
import rclpy.qos
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import math
import numpy as np


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Robot parameters
        self.wheel_radius = 0.05  # Wheel radius in meters
        self.wheel_base = 0.19   # Distance between wheels in meters
        self.sample_time = 0.1   # Time step in seconds

        # Robot state
        self.x = 0.20  # X position in meters
        self.y = 0.15  # Y position in meters
        self.yaw = 0.0  # Orientation in radians

        # Encoder velocities
        self.vel_enc_l = 0.0  # Left wheel velocity (rad/s)
        self.vel_enc_r = 0.0  # Right wheel velocity (rad/s)

        # Subscribers
        self.sub_enc_l = self.create_subscription(Float32, 'VelocityEncL', self.callback_enc_l, rclpy.qos.qos_profile_sensor_data)
        self.sub_enc_r = self.create_subscription(Float32, 'VelocityEncR', self.callback_enc_r, rclpy.qos.qos_profile_sensor_data)
        self.sub_aruco_pose = self.create_subscription(
            PoseStamped, '/aruco_pose', self.aruco_pose_callback, 10
        )

        # Publisher
        self.pub_odometry = self.create_publisher(PoseStamped, '/odom', 10)
        self.pub_atan2 = self.create_publisher(Float32, '/atan2', 10)

        # Timer
        self.timer = self.create_timer(self.sample_time, self.update_odometry)

        self.get_logger().info('Odometry Node Started 🚀')

    def callback_enc_l(self, msg):
        self.vel_enc_l = msg.data

    def callback_enc_r(self, msg):
        self.vel_enc_r = msg.data

    def aruco_pose_callback(self, msg):
        # Update robot's pose with ArUco estimate
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        # Recover yaw from quaternion
        #qz = msg.pose.orientation.z
        #qw = msg.pose.orientation.w
        #self.yaw = 2 * math.atan2(qz, qw)
        self.get_logger().info(
            f"Odometry reset from ArUco: x={self.x:.2f}, y={self.y:.2f}"
        )

    def update_odometry(self):
        # Error correction factor (as a percentage)
        linear_correction_factor = 1.10  # Adjust this value based on your measurements (e.g., 1.10 for 10% error)
        angular_correction_factor = 1.17  # Adjust this value based on your measurements (e.g., 1.10 for 10% error)

        # Calculate linear and angular velocities
        v_l = self.vel_enc_l * self.wheel_radius  # Linear velocity of left wheel
        v_r = self.vel_enc_r * self.wheel_radius  # Linear velocity of right wheel

        v = (v_r + v_l) / 2  # Linear velocity of the robot
        w = (v_r - v_l) / self.wheel_base  # Angular velocity of the robot

        # Apply correction factor
        v *= linear_correction_factor
        w *= angular_correction_factor

        # Update robot pose
        delta_x = v * math.cos(self.yaw) * self.sample_time
        delta_y = v * math.sin(self.yaw) * self.sample_time
        delta_yaw = w * self.sample_time

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # Normalize yaw to the range [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Publish atan2 value
        atan2_msg = Float32()
        atan2_msg.data = self.yaw
        self.pub_atan2.publish(atan2_msg)

        # Publish odometry
        odometry_msg = PoseStamped()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = 'odom'
        odometry_msg.pose.position.x = self.x
        odometry_msg.pose.position.y = self.y
        odometry_msg.pose.position.z = 0.0
        odometry_msg.pose.orientation.x = 0.0
        odometry_msg.pose.orientation.y = 0.0
        odometry_msg.pose.orientation.z = math.sin(self.yaw / 2)
        odometry_msg.pose.orientation.w = math.cos(self.yaw / 2)

        # Obtain yaw orientation from quaternion
        qz = odometry_msg.pose.orientation.z
        qw = odometry_msg.pose.orientation.w
        th = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

        self.pub_odometry.publish(odometry_msg)
        #self.get_logger().info(f'X: {odometry_msg.pose.position.x}, Y: {odometry_msg.pose.position.y} Yaw: {th}')


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()