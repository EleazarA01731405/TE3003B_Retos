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
        self.lidar_subscriber = self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)
        # Publiser to cmd_vel for robot movement and reaction
        self_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Globarl variables

        # Distance threshold to detect obstacles (meters) [min]
        self.min_threshold = 0.10
        # Distance threshold to detect obstacles (meters) [max]
        self.max_threshold = 0.15

        # Lidar data reading variables ranges
        self.min_dis_f = 0
        self.min_dis_l = 0
        self.min_dis_r = 0

        # Timer callback
        self.timer = self.create_timer(0.1, self.loop_callback)

    def lidar_callback(self, msg):
        # Extract the first 20 and last 20 values from the front reading ranges array
        front1 = msg.ranges[:20]
        front2 = msg.ranges[-20:]

        # Combine first and last 20th values
        front = front1 + front2

        # Extract the left and right values from the ranges array
        left = msg.ranges[70:110]
        right = msg.ranges[250:290]

        # Filter out inf values and NaN values
        val_readings_f = [x for x in front if x != float('inf') and not math.isnan(x)]
        val_readings_l = [x for x in left if x != float('inf') and not math.isnan(x)]
        val_readings_r = [x for x in right if x != float('inf') and not math.isnan(x)]
        
        
        
        
        #Notas para read me file

        # Se debe de conectar el LiDar despues de que haya boteado la jetson

        # Velocidades lineales y angulares no deben de pasar de 0.2 en cmd_vel

        # Se de be de lanzar el agente de micro_ros con el puerto especificado ttyUSB1
        #ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1