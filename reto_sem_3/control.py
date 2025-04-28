import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

class ControlNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Declare parameters for namespace and number of points
        self.declare_parameter('number_of_points', 4)  # Default is 4 points (square)

        # Get namespace and number of points
        self.namespace = self.get_namespace().rstrip('/')
        self.number_of_points = self.get_parameter('number_of_points').value

        # Subscriber to pose_sim (with namespace)
        self.pose_subscriber = self.create_subscription(
            PoseStamped,  # Message type
            f'{self.namespace}/pose_sim',  # Topic name with namespace
            self.pose_callback,  # Callback function
            10  # QoS (queue size)
        )

        # Publisher to cmd_vel (with namespace)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,  # Message type
            f'{self.namespace}/cmd_vel',  # Topic name with namespace
            10  # QoS (queue size)
        )

        # Parameters
        self.current_pose = None  # To store the latest pose
        self.polygon_trajectory = self.calculate_polygon_points(self.number_of_points)  # Generate polygon points
        self.current_target_index = 0  # Index of the current target waypoint

        # PID gains for linear velocity
        self.kp_linear = 0.2  # Proportional gain
        self.ki_linear = 0.001  # Integral gain
        self.kd_linear = 0.01  # Derivative gain

        # PID gains for angular velocity
        self.kp_angular = 0.9  # Proportional gain
        self.ki_angular = 0.005  # Integral gain
        self.kd_angular = 1.0  # Derivative gain

        # PID state variables
        self.linear_error_sum = 0.0  # Integral term for linear velocity
        self.prev_linear_error = 0.0  # Previous error for linear velocity
        self.angular_error_sum = 0.0  # Integral term for angular velocity
        self.prev_angular_error = 0.0  # Previous error for angular velocity

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Node Started
        self.get_logger().info(f'Controller Node Started 🚀 for {self.namespace} with {self.number_of_points} points')

    def calculate_polygon_points(self, num_points):
        """Calculate points on a unit circle for a regular polygon."""
        points = [(1.0, 0.0)]  # Start at (1, 0) on the border of the unit circle
        for i in range(1, num_points):  # Start from the second point
            angle = 2 * np.pi * i / num_points  # Angle for each point
            x = np.cos(angle)  # X-coordinate
            y = np.sin(angle)  # Y-coordinate
            points.append((x, y))
        points.append(points[0])  # Close the polygon by returning to the starting point
        self.get_logger().info(f"Generated polygon with {num_points} points: {points}")
        return points

    def pose_callback(self, msg):
        """Callback function to handle PoseStamped messages."""
        self.current_pose = msg

    def control_loop(self):
        """Control loop to calculate and publish velocity commands."""
        if self.current_pose is None:
            # Wait until the first pose is received
            return

        # Extract current position and orientation
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        theta = 2 * np.arctan2(
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        )

        # Get the current target waypoint
        if self.current_target_index >= len(self.polygon_trajectory):
            # Stop the robot if all waypoints are reached
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Polygon trajectory completed!')
            return

        x_target, y_target = self.polygon_trajectory[self.current_target_index]

        # Calculate position and orientation errors
        x_error = x_target - x
        y_error = y_target - y
        theta_target = np.arctan2(y_error, x_error)  # Desired heading
        theta_error = theta_target - theta

        # Normalize theta_error to the range [-pi, pi]
        theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))

        # Calculate distance error
        distance_error = np.sqrt(x_error**2 + y_error**2)

        # PID control for linear velocity
        self.linear_error_sum += distance_error  # Integral term
        linear_error_diff = distance_error - self.prev_linear_error  # Derivative term
        linear_velocity = (
            self.kp_linear * distance_error +
            self.ki_linear * self.linear_error_sum +
            self.kd_linear * linear_error_diff
        )
        self.prev_linear_error = distance_error  # Update previous error

        # PID control for angular velocity
        self.angular_error_sum += theta_error  # Integral term
        angular_error_diff = theta_error - self.prev_angular_error  # Derivative term
        angular_velocity = (
            self.kp_angular * theta_error +
            self.ki_angular * self.angular_error_sum +
            self.kd_angular * angular_error_diff
        )
        self.prev_angular_error = theta_error  # Update previous error

        # Stop linear velocity if the robot is close to the target
        if distance_error < 0.09:  # 0.09m tolerance
            linear_velocity = 0.0
            angular_velocity = 0.0
            self.current_target_index += 1  # Move to the next waypoint
            self.get_logger().info(f"Reached waypoint {self.current_target_index}/{len(self.polygon_trajectory)}")
            return

        # Publish velocity commands
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)

        # Log control information
        self.get_logger().info(
            f"Target: x={x_target:.2f}, y={y_target:.2f} | "
            f"Current: x={x:.2f}, y={y:.2f}, theta={theta:.2f} | "
            f"Errors: dist={distance_error:.2f}, theta={theta_error:.2f} | "
            f"Cmd: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    # Initialize the node
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