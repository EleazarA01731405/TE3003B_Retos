import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import math


class Bug0Algorithm(Node):
    def __init__(self):
        super().__init__('bug0_algorithm')
        self.lidar_subscriber = self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_threshold = 0.25  # Distance threshold to detect obstacles (meters)
        self.state = 'MOVE_FORWARD'
        self.min_distance_f = 0
        self.min_distance_l = 0
        self.min_distance_r = 0
        self.timer = self.create_timer(0.1, self.loop_callback)

        # Subscriber to pose_sim
        self.pose_subscriber = self.create_subscription(
            PoseStamped,  # Message type
            'pose_sim',   # Topic name
            self.pose_callback,  # Callback function
            10  # QoS (queue size)
        )

        # Parameters
        self.current_pose = None
        self.goal = [1.45,1.2]

        # PID parameters
        self.kp_linear = 0.2
        self.ki_linear = 0.001
        self.kd_linear = 0.01

        # PID angular parameters
        self.kp_angular = 0.9
        self.ki_angular = 0.005
        self.kd_angular = 1.0

    def pose_callback(self, msg):
        """Callback function to handle PoseStamped messages."""
        self.current_pose = msg

    def lidar_callback(self, msg):
        # Extract the first 15 and last 15 values from the ranges array
        front1 = msg.ranges[:15]
        front2 = msg.ranges[-15:]
        left = msg.ranges[16:100]
        right = msg.ranges[-16:-100]

        # Combine the selected ranges
        selected_ranges = front1 + front2

        # Filter out invalid values (e.g., inf or NaN)
        valid_ranges_f = [r for r in selected_ranges if r > 0.0 and r < float('inf')]
        valid_ranges_l = [r for r in left if r > 0.0 and r < float('inf')]
        valid_ranges_r = [r for r in right if r > 0.0 and r < float('inf')]

        # Check if any valid range is below the obstacle threshold
        if valid_ranges_f:
            self.min_distance_f = min(valid_ranges_f)
        else:
            self.min_distance_f = float('inf')  # No valid readings

        if valid_ranges_l:
            self.min_distance_l = min(valid_ranges_l)
        else:
            self.min_distance_l = float('inf')  # No valid readings

        if valid_ranges_r:
            self.min_distance_r = min(valid_ranges_r)
        else:
            self.min_distance_r = float('inf')  # No valid readings

    def loop_callback(self):
        # Check if there is a direct path to the goal
        if self.min_distance_f > self.obstacle_threshold:
            # If an obstacle is detected, follow the wall
            self.move_towards_goal()
        else:
            # If no obstacle, move towards the goal
            self.wall_follower()

    def wall_follower(self):
        # Bug 0 logic for wall following
        if self.min_distance_f < self.obstacle_threshold and self.min_distance_r < self.obstacle_threshold:
            self.turn_l()
        elif self.min_distance_f < self.obstacle_threshold and self.min_distance_r > self.obstacle_threshold:
            self.turn_l()
        elif self.min_distance_f > self.obstacle_threshold and self.min_distance_r < self.obstacle_threshold:
            self.move_forward()
        #elif self.min_distance_f > self.obstacle_threshold and self.min_distance_r > self.obstacle_threshold:
        #    self.turn_r()

    def move_towards_goal(self):
        
        if self.current_pose is None:
            # Wait for the first pose update
            return
        
        # Extract the current pose
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y

        theta = 2 * math.atan2(
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
            )
        
        """Move towards the self.goal using PID control."""
        # Extract the goal position
        x_target, y_target = self.goal

        # Calculate position and orientation errors
        x_error = x_target - x
        y_error = y_target - y
        theta_target = math.atan2(y_error, x_error)  # Desired heading
        theta_error = theta_target - theta

        # Normalize theta_error to the range [-pi, pi]
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

        # Calculate distance error
        distance_error = math.sqrt(x_error**2 + y_error**2)

        # Stop linear velocity if the robot is close to the goal
        if distance_error < 0.1:  # 0.1m tolerance
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Goal reached!')
            return

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

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.2  # Move forward at 0.2 m/s
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def turn_r(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5  # Turn at 0.5 rad/s
        self.cmd_vel_publisher.publish(twist)

    def turn_l(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Turn at -0.5 rad/s
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    bug0_node = Bug0Algorithm()
    rclpy.spin(bug0_node)
    bug0_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
