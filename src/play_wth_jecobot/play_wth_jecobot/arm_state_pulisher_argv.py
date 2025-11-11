#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys

theta_1 = float(sys.argv[1])
theta_2 = float(sys.argv[2])
theta_3 = float(sys.argv[3])
theta_4 = float(sys.argv[4])
theta_5 = float(sys.argv[5])
theta_6 = float(sys.argv[6])

class FixedJointPublisher(Node):
    def __init__(self):
        super().__init__('fixed_joint_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_state)  # 10 Hz
        
        self.theta_1 = float(sys.argv[1])
        self.theta_2 = float(sys.argv[2])
        self.theta_3 = float(sys.argv[3])
        self.theta_4 = float(sys.argv[4])
        self.theta_5 = float(sys.argv[5])
        self.theta_6 = float(sys.argv[6])

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['1_Joint', '2_Joint', '3_Joint', '4_Joint', '5_Joint', '6_Joint']  # replace with real names
        msg.position = [self.theta_1, self.theta_2, self.theta_3, self.theta_4, self.theta_5, self.theta_6]          # your desired angles
        self.pub.publish(msg)

def main():
	rclpy.init()
	node = FixedJointPublisher()
	rclpy.spin(node)
	
if __name__ == "__main__":
	main()
