import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Declare parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Subscriber to lidar topic for obstacle detection
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )

        # Publisher to cmd_vel for robot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Global variables
        self.min_threshold = 0.20  # Minimum distance to detect obstacles
        self.max_threshold = 0.30  # Increase from 0.60 to 0.80
        self.right_max_threshold = 17 #Max treshold for right sensing
        self.back_max_threshold = 0.30  # Threshold for back sensor
        self.min_dis_f = float('inf')  # Front distance
        self.min_dis_r = float('inf')  # Right distance
        self.min_dis_b = float('inf')  # Back distance
        self.obstacle_found = False  # Obstacle detection flag

        # Timer callback
        self.timer = self.create_timer(0.1, self.loop_callback)

    def lidar_callback(self, msg):
        # Extract ranges for front, right, and back
        front = msg.ranges[:60] + msg.ranges[-60:]
        right = msg.ranges[750:870]
        back = msg.ranges[540:660]

        # Filter out invalid values (inf, NaN, below 15 cm, or above 12 m)
        val_readings_f = [x for x in front if 0.15 <= x <= 12.0 and not np.isnan(x)]
        val_readings_r = [x for x in right if 0.15 <= x <= 12.0 and not np.isnan(x)]
        val_readings_b = [x for x in back if 0.15 <= x <= 12.0 and not np.isnan(x)]

        # Update minimum distances
        self.min_dis_f = np.min(val_readings_f) if val_readings_f else float('inf')
        self.min_dis_r = np.min(val_readings_r) if val_readings_r else float('inf')
        self.min_dis_b = np.min(val_readings_b) if val_readings_b else float('inf')

    def loop_callback(self):
        # Log the current distances
        self.get_logger().info(
            f"Obstacle Found: {self.obstacle_found} | Front: {self.min_dis_f:.2f} | Right: {self.min_dis_r:.2f} | Back: {self.min_dis_b:.2f}"
        )

        # Check if there is an obstacle in front (first contact)
        if not self.obstacle_found and (0.15 <= self.min_dis_f <= self.max_threshold):
            self.turn_l()
            self.obstacle_found = True
            self.get_logger().info('Obstacle detected FRB[True | False | False], turning left')

        # Check if there is an obstacle on the right side, but not in front (following object)
        elif self.obstacle_found and (self.min_dis_f > self.max_threshold) and (self.min_dis_r < self.right_max_threshold):
            self.move_forward()
            self.get_logger().info('Obstacle detected FRB[False | False | True], moving forward')

        # Check if there is an obstacle on the right side and in front (following object)
        elif self.obstacle_found and (self.min_dis_f < self.max_threshold) and (self.min_dis_r < self.right_max_threshold):
            self.turn_l()
            self.get_logger().info('Obstacle detected FRB[True | True | False], turning left')

        # Check for corners of object found (following object)
        elif self.obstacle_found and (self.min_dis_f > self.max_threshold) and (self.min_dis_r > self.right_max_threshold) and (self.min_dis_b < self.back_max_threshold):
            self.turn_r()
            self.get_logger().info('Obstacle corner detected FRB[False | False | True], turning right')

        # Object clear (moving forward)
        elif self.obstacle_found and (self.min_dis_f > self.max_threshold) and (self.min_dis_r > self.right_max_threshold) and (self.min_dis_b > self.back_max_threshold):
            self.move_forward()
            self.obstacle_found = False
            self.get_logger().info('Obstacle cleared, moving forward')

        # Default behavior: Stop or turn if no valid readings
        else:
            if self.min_dis_f == float('inf') or self.min_dis_f > 12.0:
                self.get_logger().warn('No valid front readings, stopping robot')
                self.stop()
            else:
                self.move_forward()
                self.get_logger().info('Moving forward')

    # Move the robot forward
    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.15  # Move forward at 0.15 m/s
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    # Turn the robot to the right
    def turn_r(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.3  # Turn at -0.3 rad/s
        self.cmd_vel_publisher.publish(twist)

    # Turn the robot to the left
    def turn_l(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3  # Turn at 0.3 rad/s
        self.cmd_vel_publisher.publish(twist)

    # Stop the robot
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    # Move the robot backward
    def move_backward(self):
        twist = Twist()
        twist.linear.x = -0.1
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


