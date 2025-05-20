import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class PuzzleBotPublisher(Node):

    def __init__(self):
        super().__init__('puzzlebot')

        # Bot Initial Pose
        self.intial_pos_x = 1.0
        self.intial_pos_y = 1.0
        self.intial_pos_z = 0.0
        self.intial_pos_yaw = np.pi / 2
        self.intial_pos_pitch = 0.0
        self.intial_pos_roll = 0.0

        # Robot parameters
        self.wheel_radius = 0.05  # Radius of the wheels (meters)
        self.wheel_base = 0.19    # Distance between the wheels (meters)

        # Linear and angular velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Define Transformations
        self.define_TF()

        # Initialise Message to be published
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ["wheel_r_joint", "wheel_l_joint", "caster_joint"]
        self.ctrlJoints.position = [0.0, 0.0, 0.0]
        self.ctrlJoints.velocity = [0.0] * 3
        self.ctrlJoints.effort = [0.0] * 3

        # Create Transform Broadcasters
        self.tf_br_base = TransformBroadcaster(self)
        self.tf_br_base_footprint = TransformBroadcaster(self)

        # Publisher for joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriber to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Create a Timer for updating robot state
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def cmd_vel_cb(self, msg):
        """Callback for /cmd_vel topic."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def timer_cb(self):
        """Timer callback to update robot state."""
        time = self.get_clock().now().nanoseconds / 1e9

        # Calculate wheel velocities
        v_r = (2 * self.linear_velocity + self.angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        v_l = (2 * self.linear_velocity - self.angular_velocity * self.wheel_base) / (2 * self.wheel_radius)

        # Update wheel joint positions
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.position[0] += v_r * 0.01  # Increment based on timer period
        self.ctrlJoints.position[1] += v_l * 0.01
        self.ctrlJoints.position[2] = 0.0

        # Update robot position and orientation
        delta_x = self.linear_velocity * np.cos(self.intial_pos_yaw) * 0.01
        delta_y = self.linear_velocity * np.sin(self.intial_pos_yaw) * 0.01
        delta_yaw = self.angular_velocity * 0.01

        self.intial_pos_x += delta_x
        self.intial_pos_y += delta_y
        self.intial_pos_yaw += delta_yaw

        # Update transforms
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.transform.translation.x = self.intial_pos_x
        self.base_link_tf.transform.translation.y = self.intial_pos_y
        self.base_link_tf.transform.translation.z = self.intial_pos_z
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]

        self.tf_br_base.sendTransform(self.base_link_tf)

        # Publish joint states
        self.publisher.publish(self.ctrlJoints)

    def define_TF(self):
        """Define initial transforms."""
        self.base_footprint_tf = TransformStamped()
        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.header.frame_id = 'odom'
        self.base_footprint_tf.child_frame_id = 'base_footprint'
        self.base_footprint_tf.transform.translation.x = self.intial_pos_x
        self.base_footprint_tf.transform.translation.y = self.intial_pos_y
        self.base_footprint_tf.transform.translation.z = 0.0
        q_foot = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)
        self.base_footprint_tf.transform.rotation.x = q_foot[1]
        self.base_footprint_tf.transform.rotation.y = q_foot[2]
        self.base_footprint_tf.transform.rotation.z = q_foot[3]
        self.base_footprint_tf.transform.rotation.w = q_foot[0]

        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.header.frame_id = 'odom'
        self.base_link_tf.child_frame_id = 'base_link'
        self.base_link_tf.transform.translation.x = self.intial_pos_x
        self.base_link_tf.transform.translation.y = self.intial_pos_y
        self.base_link_tf.transform.translation.z = self.intial_pos_z
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]


def main(args=None):
    rclpy.init(args=args)

    node = PuzzleBotPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()