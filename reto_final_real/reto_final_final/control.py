import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


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

        # State subcriber for control changes
        self.state_subscriber = self.create_subscription(
            String,
            '/state',
            self.state_callback,
            10
        )

        # State publisher for control changes
        self.state_publisher = self.create_publisher(
            String,
            '/state',
            10
        )

        # Subcriber to lidar for obstacle detection
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Parameters
        self.current_pose = None
        self.current_target_index = 0
        self.state = "ARUCO"  # Initial state
        self.min_dis_f = float('inf')

        # Predefined waypoints: Start and goals
        self.points_to_go = [
            (1.67, 0.70),    # P1
            (1.49, 2.19),   # P2
            (0.25, 1.88),    # P3
            (0.16, 0.25)      # P4
        ]

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Controller Node Started 🚀 with predefined waypoints')

    # Pose callback to update current pose
    def pose_callback(self, msg):
        self.current_pose = msg

    # State callback to update current state
    def state_callback(self, msg):
        self.state = msg.data

    # Lidar callback to update minimum distances
    def lidar_callback(self, msg):
        front = msg.ranges[:30] + msg.ranges[-30:]
        val_readings_f = [x for x in front if 0.15 <= x <= 12.0 and not np.isnan(x)]
        self.min_dis_f = np.min(val_readings_f) if val_readings_f else float('inf')

    # Stop the robot
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
    
    # Control loop to manage robot movement based on state and pose
    def control_loop(self):

        if self.state == "ARUCO":

            self.get_logger().info(f"Front detection: {self.min_dis_f:6f}")
            if self.current_pose is None:
                return

            if self.min_dis_f <  0.35:
                self.stop()
                obstacle_found = "WALL FOLLOWER"
                self.state_publisher.publish(String(data=obstacle_found))
                self.get_logger().info(f"Obstacle detected, switching to [{obstacle_found}] state")
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
            angle_threshold = 0.05  # radians (~4.5 degrees)
            distance_threshold = 0.05  # meters

            twist = Twist()

            # Phase 1: Rotate in place to face the target
            if abs(theta_error) > angle_threshold:
                twist.linear.x = 0.0
                twist.angular.z = 0.2 if theta_error > 0 else -0.2  # Fixed angular speed
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info(
                    f"Rotating: theta_error={theta_error:.2f}, angular.z={twist.angular.z:.2f}"
                    f"Current: x={x:.2f}, y={y:.2f}, theta={theta:.2f}"
                )
                return

            # Phase 2: Move forward to the target
            if distance_error > distance_threshold:
                twist.linear.x = 0.1  # Fixed linear speed
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