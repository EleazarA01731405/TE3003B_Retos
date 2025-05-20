import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import math


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Robot parameters
        self.wheel_radius = 0.05  # Wheel radius in meters
        self.wheel_base = 0.19   # Distance between wheels in meters
        self.sample_time = 0.1   # Time step in seconds

        # Robot state
        self.x = 1.45  # X position in meters
        self.y = -3.5 # Y position in meters
        self.yaw = 1.57  # Orientation in radians

        # Encoder velocities
        self.vel_enc_l = 0.0  # Left wheel velocity (rad/s)
        self.vel_enc_r = 0.0  # Right wheel velocity (rad/s)

        # Subscribers
        self.sub_enc_l = self.create_subscription(Float32, '/VelocityEncL', self.callback_enc_l, 10)
        self.sub_enc_r = self.create_subscription(Float32, '/VelocityEncR', self.callback_enc_r, 10)

        # Publisher
        self.pub_odometry = self.create_publisher(PoseStamped, '/odometryMeters', 10)

        # Timer
        self.timer = self.create_timer(self.sample_time, self.update_odometry)

        self.get_logger().info('Odometry Node Started 🚀')

    def callback_enc_l(self, msg):
        self.vel_enc_l = msg.data

    def callback_enc_r(self, msg):
        self.vel_enc_r = msg.data

    def update_odometry(self):
        # Calculate linear and angular velocities
        v_l = self.vel_enc_l * self.wheel_radius  # Linear velocity of left wheel
        v_r = self.vel_enc_r * self.wheel_radius  # Linear velocity of right wheel

        v = (v_r + v_l) / 2  # Linear velocity of the robot
        w = (v_r - v_l) / self.wheel_base  # Angular velocity of the robot

        # Update robot pose
        delta_x = v * math.cos(self.yaw) * self.sample_time
        delta_y = v * math.sin(self.yaw) * self.sample_time
        delta_yaw = w * self.sample_time

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # Normalize yaw to the range [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

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

        self.pub_odometry.publish(odometry_msg)


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