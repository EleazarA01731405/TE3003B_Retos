import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
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

        # Parameters
        self.current_pose = None
        self.goal = [-1.5, -1.5]

        # PID parameters
        self.kp_linear = 0.2
        self.ki_linear = 0.001
        self.kd_linear = 0.01

        # PID angular parameters
        self.kp_angular = 0.9
        self.ki_angular = 0.005
        self.kd_angular = 1.0

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
            self.min_distance_f = float('inf') # No valid readings
            #self.get_logger().info("pf:inf.")

        if valid_ranges_l:
            self.min_distance_l = min(valid_ranges_l)
        else:
            self.min_distance_l = float('inf')  # No valid readings

        if valid_ranges_r:
            self.min_distance_r = min(valid_ranges_r)
        else:
            self.min_distance_r = float('inf')  # No valid readings

    def odom_callback(self, msg):
        self.current_pose = msg
        self.pos_x = msg.pose.position.x
        self.pos_y = msg.pose.position.y
        orientation_z = msg.pose.orientation.z
        orientation_w = msg.pose.orientation.w
        
        self.yaw = math.atan2(2 * (orientation_z * orientation_w), 1 - 2 * (orientation_z ** 2 + orientation_w ** 2))
        
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
            return True
        else:
            return False

    def loop_callback(self):
        #"""
        # Verificar si se alcanzó la meta
        if self.is_near_goal():
            self.state = 'STOP'
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("Meta alcanzada. Deteniendo el robot.")
            return

        #"""
        # Cambiar de estado según las condiciones
        if self.state == 'MOVE_TOWARDS_GOAL':
            if self.min_distance_f < self.obstacle_threshold:
                self.state = 'FOLLOW_WALL'
                self.get_logger().info("Obstáculo detectado. Cambiando a seguimiento de pared.")
            else:
                self.move_towards_goal()

        elif self.state == 'FOLLOW_WALL':
            if self.min_distance_f >= self.obstacle_threshold:
                self.state = 'MOVE_TOWARDS_GOAL'
                self.get_logger().info("Línea despejada. Cambiando a moverse hacia la meta.")
            else:
                self.wall_follower()

        elif self.state == 'STOP':
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)

        else:
            # Estado inicial por defecto
            self.state = 'MOVE_TOWARDS_GOAL'
        #"""

    def wall_follower(self):
        # Bug 0 logic for wall following
        desired_distance = 0.2  # Distancia deseada de la pared (en metros)
        kp = 1.0  # Ganancia proporcional para el control de distancia

        if self.min_distance_f <= self.obstacle_threshold:
            # Si hay un obstáculo al frente, girar a la derecha
            self.get_logger().info("Obstáculo al frente. Girando a la derecha.")
            self.turn_r()
        elif self.min_distance_l > 0:  # Asegurarse de que hay una lectura válida a la izquierda
            # Calcular el error de distancia con respecto a la pared
            error = self.min_distance_l - desired_distance

            # Ajustar la velocidad angular proporcionalmente al error
            angular_velocity = kp * error

            # Mantener una velocidad lineal constante mientras sigue la pared
            twist = Twist()
            twist.linear.x = 0.2  # Velocidad hacia adelante
            twist.angular.z = angular_velocity
            self.cmd_vel_publisher.publish(twist)

            self.get_logger().info(f"Siguiendo pared por la izquierda. Error: {error:.2f}, Velocidad angular: {angular_velocity:.2f}")
        """else:
        # Si no hay pared detectada, avanzar hacia adelante
        self.get_logger().info("No se detecta pared. Avanzando hacia adelante.")
        self.move_forward()"""

    def move_towards_goal(self):
        if self.current_pose is None:
            self.get_logger().warn("La posición actual no está disponible.")
            return

        # Extraer la posición actual y la meta
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        goal_x, goal_y = self.goal

        # Calcular el error de posición
        error_x = goal_x - current_x
        error_y = goal_y - current_y

        # Calcular la distancia al objetivo
        distance_to_goal = math.sqrt(error_x**2 + error_y**2)

        # Calcular el ángulo deseado hacia el objetivo
        desired_yaw = math.atan2(error_y, error_x)

        # Calcular el error angular
        angular_error = desired_yaw - self.yaw

        # Normalizar el error angular entre -pi y pi
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        # Control proporcional para las velocidades
        linear_velocity = self.kp_linear * distance_to_goal
        angular_velocity = self.kp_angular * angular_error

        # Limitar las velocidades máximas
        max_linear_velocity = 0.5  # m/s
        max_angular_velocity = 1.0  # rad/s
        linear_velocity = min(linear_velocity, max_linear_velocity)
        angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)

        # Publicar comando de velocidad
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info(f"Moviéndose hacia la meta: v_lineal={linear_velocity:.2f}, v_angular={angular_velocity:.2f}")
            

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
