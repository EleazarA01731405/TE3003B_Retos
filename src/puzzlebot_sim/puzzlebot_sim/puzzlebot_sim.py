import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from math import *

#Class Definition
class puzzlebot_sim(Node):
    def __init__(self):
        super().__init__('dc_motor')

        # Parameters
        self.r = 0.05
        self.whell_base = 0.19
        self.sample_time = 0.1

        #Set the messages
        self.wr_msg = Float32()
        self.wl_msg = Float32()

        #Set variables to be used
        self.thetark = 0.0
        self.xrk = 0.0
        self.yrk = 0.0
    
        #Declare publishers, subscribers and timers
        self.input_v = self.create_subscription(Twist, 'cmd_vel', self.input_callback,10)
        self.Wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.Wl_pub = self.create_publisher(Float32, 'wl', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_sim', 10)
        self.timer = self.create_timer(self.sample_time, self.timer_cb) 

        #Node Started
        self.get_logger().info('Dynamical System Node Started \U0001F680')   
        
    #Timer Callback
    def timer_cb(self):         
        # Calculate linear velocities of the wheels
        v_r = self.input_v + (self.input_w * self.whell_base / 2)
        v_l = self.input_v - (self.input_w * self.whell_base / 2)

        # Calculate angular velocities of the wheels
        wr = v_r / self.r
        wl = v_l / self.r

        # Publish the angular velocities
        self.wr_msg.data = wr
        self.wl_msg.data = wl

        # Calculate the new pose of the robot
        self.thetark += self.r * (wr-wl) / self.whell_base * self.sample_time
        self.xrk += self.r * (wr-wl) / self.whell_base * self.sample_time * np.cos(self.thetark)
        self.yrk += self.r * (wr-wl) / self.whell_base * self.sample_time * np.sin(self.thetark)
        
        self.pose_msg = PoseStamped()
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg.header.frame_id = 'base_link'
        self.pose_msg.pose.position.x = self.xrk
        self.pose_msg.pose.position.y = self.yrk
        self.pose_msg.pose.position.z = 0.0
        self.pose_msg.pose.orientation.x = 0.0
        self.pose_msg.pose.orientation.y = 0.0
        self.pose_msg.pose.orientation.z = np.sin(self.thetark/2)
        self.pose_msg.pose.orientation.w = np.cos(self.thetark/2)

        # Publish the pose
        self.pose_pub.publish(self.pose_msg)
        
        #Publish the result
        self.Wr_pub.publish(self.wr_msg)
        self.Wl_pub.publish(self.wl_msg)

    #Subscriber Callback
    def input_callback(self, input_sgn):
        # Extract linear and angular velocities from cmd_vel
        self.input_v = input_sgn.linear.x
        self.input_w = input_sgn.angular.z

#Main
def main(args=None):
    rclpy.init(args=args)

    node = puzzlebot_sim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()