import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np

class DegreesCalibration(Node):

    def __init__(self):
        super().__init__('degrees_calibration')

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

        # Timer callback
        #self.timer = self.create_timer(0.1, self.loop_callback)
        
    def lidar_callback(self, msg):
        # Front reading ranges
        front = msg.ranges[0]
        left = msg.ranges[90*3]
        back = msg.ranges[180*3]
        right = msg.ranges[270*3]

        # Valid values
        val_front = front if front != float('inf') else 0
        val_left = left if left != float('inf') else 0
        val_back = back if back != float('inf') else 0
        val_right = right if right != float('inf') else 0

        self.get_logger().info(
            f'[ 0 ]: {val_front:.6f}, [ 90]: {val_left:.6f}, [180]: {val_back:.6f}, [270]: {val_right:.6f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = DegreesCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()