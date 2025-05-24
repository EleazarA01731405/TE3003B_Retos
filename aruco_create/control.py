import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


class ControlNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Subscriber to pose_sim
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/odom',
            self.pose_callback,
            10
        )

        # Publisher to cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Parameters
        self.current_pose = None
        self.current_target_index = 0

        # Predefined waypoints: Start and goals
        self.points_to_go = [
            (1.67, 0.70),    # P1
            (1.49, 2.19),   # P2
            (0.25, 1.88),    # P3
            (0.16, 0.25)      # P4
        ]

        # PID gains for linear velocity
        self.kp_linear = 0.2
        self.ki_linear = 0.001
        self.kd_linear = 0.01

        # PID gains for angular velocity
        self.kp_angular = 0.9
        self.ki_angular = 0.005
        self.kd_angular = 1.0

        # PID state variables
        self.linear_error_sum = 0.0
        self.prev_linear_error = 0.0
        self.angular_error_sum = 0.0
        self.prev_angular_error = 0.0

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Controller Node Started 🚀 with predefined waypoints')

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return

        # Extract current position and orientation
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        theta = 2 * np.arctan2(
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        )

        # If all waypoints reached, stop
        if self.current_target_index >= len(self.points_to_go):
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('All waypoints reached. Stopping robot.')
            return

        x_target, y_target = self.points_to_go[self.current_target_index]

        # Calculate errors
        x_error = x_target - x
        y_error = y_target - y
        theta_target = np.arctan2(y_error, x_error)
        theta_error = theta_target - theta
        theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))
        distance_error = np.sqrt(x_error**2 + y_error**2)

        # Thresholds
        angle_threshold = 0.025  # radians (~4.5 degrees)
        distance_threshold = 0.05  # meters

        twist = Twist()

        # Phase 1: Rotate in place to face the target
        if abs(theta_error) > angle_threshold:
            twist.linear.x = 0.0
            # Simple P controller for angular velocity
            twist.angular.z = np.clip(self.kp_angular * theta_error, -0.3, 0.3)
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(
                f"Rotating: theta_error={theta_error:.2f}, angular.z={twist.angular.z:.2f}"
                f"Current: x={x:.2f}, y={y:.2f}, theta={theta:.2f}"
            )
            return

        # Phase 2: Move forward to the target
        if distance_error > distance_threshold:
            twist.linear.x = np.clip(self.kp_linear * distance_error, 0.1, 0.15)
            twist.angular.z = 0.0  # No angular correction while moving forward
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(
                f"Moving: dist_error={distance_error:.2f}, linear.x={twist.linear.x:.2f}"
                f"Current: x={x:.2f}, y={y:.2f}, theta={theta:.2f}"
            )
            return

        # If close to target, stop and go to next waypoint
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.current_target_index += 1
        self.get_logger().info(f"Reached waypoint {self.current_target_index}/{len(self.points_to_go)}")

        # Log control information
        self.get_logger().info(
            f"Target: x={x_target:.2f}, y={y_target:.2f} | "
            f"Current: x={x:.2f}, y={y:.2f}, theta={theta:.2f} | "
            f"Errors: dist={distance_error:.2f}, theta={theta_error:.2f} | "
            f"Cmd: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()