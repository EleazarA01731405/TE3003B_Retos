import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import cv2
import math


class Bug0Algorithm(Node):
    def __init__(self):
        super().__init__('bug0_algorithm')

        # Verificar si el parámetro ya está declarado
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_subscriber = self.create_subscription(PoseStamped, '/odometryMeters', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_threshold = 0.35  # Distance threshold to detect obstacles (meters)
        self.state = 'MOVE_FORWARD'
        self.min_distance_f = 0
        self.min_distance_l = 0
        self.min_distance_r = 0
        self.initial_position = None
        self.initial_position_set = False
        self.pos_x = None
        self.pos_y = None
        self.m = None
        self.b = None
        self.timer = self.create_timer(0.1, self.loop_callback)

        # Subscriber to pose_sim
        self.pose_subscriber = self.create_subscription(
            PoseStamped,  # Message type
            'pose_sim',   # Topic name
            self.pose_callback,  # Callback function
            10  # QoS (queue size)
        )

        # Camera subscriber
        self.camera_subscriber = self.create_subscription(
            Image,  # Message type
            '/camera',  # Topic name (adjust based on your simulation setup)
            self.camera_callback,  # Callback function
            10  # QoS (queue size)
        )

        # Parameters
        self.current_pose = None
        self.goal = [1.45, 1.2]

        # PID parameters
        self.kp_linear = 0.2
        self.ki_linear = 0.001
        self.kd_linear = 0.01

        # PID angular parameters
        self.kp_angular = 0.9
        self.ki_angular = 0.005
        self.kd_angular = 1.0

        # CvBridge for image conversion
        self.bridge = CvBridge()

    def pose_callback(self, msg):
        """Callback function to handle PoseStamped messages."""
        self.current_pose = msg

    def camera_callback(self, msg):
        """Callback function to process camera images."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image in a window
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)  # Required for OpenCV to process the image
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")

    def lidar_callback(self, msg):
        # Extract the first 15 and last 15 values from the ranges array
        front1 = msg.ranges[:10]
        front2 = msg.ranges[-10:]
        left = msg.ranges[20:80]
        right = msg.ranges[-20:-80]

        # Combine the selected ranges
        selected_ranges = front1 + front2

        # Filter out invalid values (e.g., inf or NaN)
        valid_ranges_f = [r for r in selected_ranges if r > 0.0 and r < float('inf')]
        valid_ranges_l = [r for r in left if r > 0.0 and r < float('inf')]
        valid_ranges_r = [r for r in right if r > 0.0 and r < float('inf')]

        # Check if any valid range is below the obstacle threshold
        if valid_ranges_f:
            self.min_distance_f = min(valid_ranges_f)
            #self.get_logger().info(f"pf:{self.min_distance_f}.")
        else:
            self.min_distance_f = 0  # No valid readings
            #self.get_logger().info("pf:inf.")

        if valid_ranges_l:
            self.min_distance_l = min(valid_ranges_l)
        else:
            self.min_distance_l = 0  # No valid readings

        if valid_ranges_r:
            self.min_distance_r = min(valid_ranges_r)
        else:
            self.min_distance_r = 0  # No valid readings

    def odom_callback(self, msg):
        self.pos_x = msg.pose.position.x
        self.pos_y = msg.pose.position.y
        orientation_z = msg.pose.orientation.z
        orientation_w = msg.pose.orientation.w
        
        self.yaw = math.atan2(2 * (orientation_z * orientation_w), 1 - 2 * (orientation_z ** 2 + orientation_w ** 2))

        # Set the initial position if not already set
        if not self.initial_position_set:
            self.initial_position = (self.pos_x, self.pos_y)
            self.initial_position_set = True
            self.get_logger().info(f"Initial position set: x={self.pos_x}, y={self.pos_y}")
            self.draw_line_to_goal()

    def draw_line_to_goal(self):
        if self.initial_position is None:
            self.get_logger().info("No hay posición inicial.")
            return

        x_start, y_start = self.initial_position
        x_goal, y_goal = self.goal

        # Manejar el caso de una línea vertical
        if x_goal == x_start:
            self.m = float('inf')  # Pendiente infinita
            self.b = None  # No hay intercepto en este caso
            #self.get_logger().info("Línea vertical creada.")
        else:
            # Calcular pendiente (m) e intercepto (b)
            self.m = (y_goal - y_start) / (x_goal - x_start)
            self.b = y_start - self.m * x_start
            #self.get_logger().info(f"Recta creada: y = {self.m:.2f}x + {self.b:.2f}")

    def is_near_line(self):
        if self.m is None or (self.m != float('inf') and self.b is None):
            self.get_logger().warn("La línea no está definida correctamente.")
            return False

        if self.m == float('inf'):
            # Línea vertical: calcular distancia horizontal
            distance = abs(self.pos_x - self.initial_position[0])
        else:
            # Calcular distancia perpendicular a la línea
            distance = abs(self.m * self.pos_x - self.pos_y + self.b) / math.sqrt(self.m**2 + 1)

        # Define un umbral para considerar que el robot está cerca de la línea
        threshold = 0.05

        if distance < threshold:
            #self.get_logger().info("Robot is near the line to the goal.")
            return True
        else:
            return False
        
    def is_near_goal(self):
        # Verificar si la posición actual está definida
        if self.pos_x is None or self.pos_y is None:
            self.get_logger().warn("La posición actual no está definida.")
            return False
        
        # Calculate the distance to the goal
        distance_to_goal = math.sqrt((self.pos_x - self.goal[0])**2 + (self.pos_y - self.goal[1])**2)
        
        # Define a threshold for "closeness" to the goal (e.g., 0.1 meters)
        threshold = 0.05
        
        if distance_to_goal < threshold:
            self.get_logger().info("llego.")
            return True
        else:
            return False

    def loop_callback(self):
        if self.initial_position is None:
            self.get_logger().info("Esperando posición inicial...")
            return

        self.draw_line_to_goal()
        if self.is_near_goal():
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
        else:
            if self.is_near_line() and (self.min_distance_f >= self.obstacle_threshold and self.min_distance_r >= self.obstacle_threshold):
                self.get_logger().info("mover hacia la meta.")
                self.move_towards_goal()
            elif self.is_near_line() and (self.min_distance_f <= self.obstacle_threshold or self.min_distance_r <= self.obstacle_threshold):
                self.get_logger().info("wall follower")
                self.wall_follower()
            elif self.is_near_line() == False and (self.min_distance_f <= self.obstacle_threshold or self.min_distance_r <= self.obstacle_threshold):
                self.get_logger().info("wall follower")
                self.wall_follower()

    def wall_follower(self):
        # Bug 0 logic for wall following
        if self.min_distance_f <= self.obstacle_threshold and self.min_distance_r <= self.obstacle_threshold:
            self.get_logger().info("pared f y r.")
            self.turn_l()
        elif self.min_distance_f <= self.obstacle_threshold and self.min_distance_r >= self.obstacle_threshold:
            self.get_logger().info("pared f.")
            self.turn_l()
        elif self.min_distance_f >= self.obstacle_threshold and self.min_distance_r <= self.obstacle_threshold:
            self.get_logger().info("pared r.")
            self.move_forward()
        """else :
            # Si no hay pared, cambiar al estado de moverse hacia la meta
            self.get_logger().info("No hay pared, moviéndose hacia la meta.")
            self.move_towards_goal()"""

    def move_towards_goal(self):
        if self.initial_position is None:
            return  # No hacer nada si no hay posición inicial

        x_start, y_start = self.initial_position
        x_goal, y_goal = self.goal
        x_current, y_current = self.pos_x, self.pos_y

        if self.m == float('inf'):
            # Línea vertical: controlar desviación horizontal
            deviation = x_current - x_start
        else:
            # Calcular desviación perpendicular
            deviation = (self.m * x_current - y_current + self.b) / math.sqrt(self.m**2 + 1)

        # Control proporcional
        kp_angular = 2.0
        angular_velocity = -kp_angular * deviation
        linear_velocity = 0.2

        # Publicar comando de velocidad
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)
        

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
    cv2.destroyAllWindows()  # Close OpenCV windows when the node shuts down


if __name__ == '__main__':
    main()
