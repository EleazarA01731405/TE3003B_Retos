import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np


class WallFollower(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('wall_follower')

        # Declare parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Subscriber to lidar topic for obstacle detection
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
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

        # Publisher to cmd_vel for robot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Global variables

        # 
        self.front_max = .30
        self.right_max = .15
        self.back_max = .30

        # Initialize minimum distances of lidar readings
        self.min_dis_f = float('inf')  # Front distance
        self.min_dis_r = float('inf')  # Right distance
        self.min_dis_b = float('inf')  # Back distance
        self.obstacle_found = False  # Obstacle detection flag

        self.state = ""  # Initial state

        # Timer callback
        self.timer = self.create_timer(0.1, self.loop_callback)

    # State callback to update current state
    def state_callback(self, msg):
        self.state = msg.data

    def lidar_callback(self, msg):
        # Extract ranges for front, right, and back
        front = msg.ranges[:60] + msg.ranges[-60:]
        right = msg.ranges[700:870]
        back = msg.ranges[540:600]

        # Filter out invalid values (inf, NaN, below 15 cm, or above 12 m)
        val_readings_f = [x for x in front if 0.15 <= x <= 12.0 and not np.isnan(x)]
        val_readings_r = [x for x in right if 0.15 <= x <= 12.0 and not np.isnan(x)]
        val_readings_b = [x for x in back if 0.15 <= x <= 12.0 and not np.isnan(x)]

        # Update minimum distances
        self.min_dis_f = np.min(val_readings_f) if val_readings_f else float('inf')
        self.min_dis_r = np.min(val_readings_r) if val_readings_r else float('inf')
        self.min_dis_b = np.min(val_readings_b) if val_readings_b else float('inf')

    def loop_callback(self):

        if self.state == "WALL FOLLOWER":
            self.get_logger().info(f"   Front: {self.min_dis_f:6f}   |   Right: {self.min_dis_r:6f}   |   Back: {self.min_dis_b:6f}")
            # Check if there is an obstacle in front (first contact)

            # New Logic
            if self.min_dis_f < self.front_max:
                self.turn_l()
                self.get_logger().info("Pared enfrente, I")
                return
            elif self.min_dis_f > self.front_max and .50 > self.min_dis_r > self.right_max:
                self.move_forward()
                self.get_logger().info("Pared derecha, F")
                return
            elif self.min_dis_f > .50 and self.min_dis_r > .50:
                self.stop()
                obstacle_cleared = "ARUCO"
                self.state_publisher.publish(String(data=obstacle_cleared))
                self.get_logger().info(f"Obstacle detected, switching to [{obstacle_cleared}] state 1")
                #self.get_logger().info("PAred librada")
                return
            else:
                self.move_forward()

    # Move the robot forward
    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.1  # Move forward at 0.15 m/s
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    # Turn the robot to the right
    def turn_r(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.2  # Turn at -0.3 rad/s
        self.cmd_vel_publisher.publish(twist)

    # Turn the robot to the left
    def turn_l(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.2  # Turn at 0.3 rad/s
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


