import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import math

class WallFollower(Node):
    def __init__(self):

        super().__init__('wall_follower')

        # Verificar si el parámetro ya está declarado
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Subscriber to lidar topic for obstacle detection
        self.lidar_subscriber = self.create_subscription(
            LaserScan,              # Message type  
            '/scan',                # Topic name
            self.lidar_callback,    # Callback function
            10                      # QoS (queue size)
        )
        
        # Publiser to cmd_vel for robot movement and reaction
        self.cmd_vel_publisher = self.create_publisher(
            Twist,                  # Message type
            '/cmd_vel',             # Topic name
            10                      # QoS (queue size)
        )

        # Globarl variables

        # Distance threshold to detect obstacles (meters) [min]
        self.min_threshold = 0.15
        # Distance threshold to detect obstacles (meters) [max]
        self.max_threshold = 0.25
        # Distance threshold aid to detect obstacle's corners (meters) [max]
        self.back_max_threshold = 0.50

        # Lidar data reading variables ranges
        self.min_dis_f = 0
        self.min_dis_r = 0
        self.min_dis_b = 0

        # Timer callback
        self.timer = self.create_timer(0.1, self.loop_callback)

        # Obstacle found control variable
        self.obstacle_found = False

    def lidar_callback(self, msg):
        # Extract the first 20 and last 20 values from the front reading ranges array
        front1 = msg.ranges[:20]
        front2 = msg.ranges[-20:]

        # Combine first and last 20th values
        front = front1 + front2

        # Extract the left and right values from the ranges array
        right = msg.ranges[250:290]
        #left = msg.ranges[70:110]

        # Extract the back values from the ranges array
        back = msg.ranges[180:220]
        # Filter out inf values and NaN values
        val_readings_f = [x for x in front if x != float('inf') and not math.isnan(x)]
        val_readings_l = [x for x in back if x != float('inf') and not math.isnan(x)]
        val_readings_r = [x for x in right if x != float('inf') and not math.isnan(x)]
        
    def loop_callback(self):
        
        # Check if there is and obstacle in front (fisrt contact)
        if (self.obstacle_found == False) and (self.min_threshold < self.min_dis_f < self.max_threshold):
            self.turn_l()
            self.obstacle_found = True
            self.get_logger().info('Obstacle detected FRB[  True  |  False  |  False  ], turning left')
        
        # Check if there is an obstacle on the right side, but not  on the front (Following object)
        elif (self.obstacle_found == True) and (self.min.dis_f > self.max_threshold) and  (self.min_threshold < self.min_dis_r < self.max_threshold):
            self.move_forward()
            self.get_logger().info('Obstacle detected FRB[  False  |  False  |  True  ], moving forward')
        
        # Check if there is an obstacle on the right side and on the front (Following object)
        elif (self.obstacle_found == True) and (self.min_threshold < self.min_dis_f < self.max_threshold) and  (self.min_dis_r < self.max_threshold):
            self.turn_l()
            self.get_logger().info('Obstacle detected FRB[  True  |  True  |  False  ], turning left')
        
        # Check for corners of object found (Following object)
        elif (self.obstacle_found == True) and (self.min_dis_f > self.max_threshold) and  (self.min_dis_r > self.max_threshold) and (self.min_threshold < self.min_dis_b < self.back_max_threshold):
            self.turn_r()
            self.get_logger().info('Obstacle corner detected FRB[  False  |  False  |  True  ], turning right')

        # Object clear (Moving foward)
        else :
            self.move_forward()
            self.obstacle_found = False
            self.get_logger().info('Obstacle cleared, moving forward')


    # Move the robot forward        
    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.15  # Move forward at 0.2 m/s
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    # Turn the robot to the right
    def turn_r(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = -0.15  # Turn at 0.5 rad/s
        self.cmd_vel_publisher.publish(twist)

    # Turn the robot to the left
    def turn_l(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.15  # Turn at -0.5 rad/s
        self.cmd_vel_publisher.publish(twist)
    

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
