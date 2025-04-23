import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from math import cos, sin, pi


class ControlNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Declare a parameter for the number of points
        self.declare_parameter('number_of_points', 6)  # Default is 4 points (square)

        # Subscriber to pose_sim
        self.pose_subscriber = self.create_subscription(
            PoseStamped,  # Message type
            'pose_sim',   # Topic name
            self.pose_callback,  # Callback function
            10  # QoS (queue size)
        )

        # Publisher to cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist,  # Message type
            'cmd_vel',  # Topic name
            10  # QoS (queue size)
        )

        # Parameters
        self.current_pose = None  # To store the latest pose
        self.number_of_points = self.get_parameter('number_of_points').value  # Get the parameter value
        self.polygon_trajectory = self.calculate_polygon_points(self.number_of_points)  # Generate polygon points
        self.current_target_index = 0  # Index of the current target waypoint

        # Control gains
        self.kp_linear = 0.2  # Proportional gain for linear velocity
        self.kp_angular = 0.65  # Proportional gain for angular velocity

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Node Started
        self.get_logger().info(f'Controller Node Started 🚀 with {self.number_of_points} points')

    def calculate_polygon_points(self, num_points):
        """Calculate points on a unit circle for a regular polygon."""
        points = []
        for i in range(num_points):
            angle = 2 * pi * i / num_points  # Angle for each point
            x = cos(angle)  # X-coordinate
            y = sin(angle)  # Y-coordinate
            points.append((x, y))
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

        # Calculate control signals
        distance_error = np.sqrt(x_error**2 + y_error**2)
        linear_velocity = self.kp_linear * distance_error
        angular_velocity = self.kp_angular * theta_error

        # Stop linear velocity if the robot is close to the target
        if distance_error < 0.05:  # 5 cm tolerance
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