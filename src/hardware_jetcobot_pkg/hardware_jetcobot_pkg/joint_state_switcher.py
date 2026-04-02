#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool
import time


class JointStateSwitcher(Node):
    def __init__(self):
        super().__init__('joint_state_switcher')
        
        # Subscriber setup
        self.fake_joint_states_sub = self.create_subscription(
            JointState,
            'joint_states_raw',
            self.fake_joint_states_callback,
            1
        )
        
        self.real_joint_states_sub = self.create_subscription(
            JointState,
            'real_joint_states',
            self.real_joint_states_callback,
            1
        )
        
        self.move_action_status_sub = self.create_subscription(
            GoalStatusArray,
            '/arm_controller/follow_joint_trajectory/_action/status',
            self.move_action_status_callback,
            1
        )
        
        # Publisher setup
        self.joint_states_pub = self.create_publisher(
            JointState,
            'joint_states',
            1
        )
        
        self.get_angles_cmd_pub = self.create_publisher(
            Bool,
            'get_angles_cmd',
            1
        )
        
        # State variables
        self.latest_fake_joint_states = None
        self.latest_real_joint_states = None
        self.current_status = None
        self.using_real_states = True
        
        # 40Hz timer (runs every 0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info('Joint State Switcher node initialized')
    
    def fake_joint_states_callback(self, msg):
        # self.get_logger().info(f"Received fake joint states! one value is {msg.position[0]} for {msg.name[0]}")
        """Callback for fake_joint_states topic"""
        self.latest_fake_joint_states = msg
    
    def real_joint_states_callback(self, msg):
        # self.get_logger().info(f"Received real joint states! one value is {msg.position[0]} for {msg.name[0]}")
        """Callback for real_joint_states topic"""
        # self.get_logger().warn(f'Received real joint states! one value is {msg.position[0]} for {msg.name[0]}')
        self.latest_real_joint_states = msg
    
    def timer_callback(self):
        """Publish joint_states at 40Hz"""
        if self.using_real_states and self.latest_real_joint_states is not None:
            # In real states mode – update timestamp
            updated_msg = JointState()
            updated_msg.header = self.latest_real_joint_states.header
            updated_msg.header.stamp = self.get_clock().now().to_msg()
            updated_msg.name = self.latest_real_joint_states.name
            updated_msg.position = self.latest_real_joint_states.position
            updated_msg.velocity = self.latest_real_joint_states.velocity
            updated_msg.effort = self.latest_real_joint_states.effort
            
            self.joint_states_pub.publish(updated_msg)
            # self.get_logger().info(f"Published joint_states from real_joint_states: {self.latest_real_joint_states.position}")
            
        elif self.latest_fake_joint_states is not None:
            # In fake states mode (status = 2)
            self.joint_states_pub.publish(self.latest_fake_joint_states)
            # self.get_logger().info(f"Published joint_states from fake_joint_states: {self.latest_fake_joint_states.position}")
    
    def move_action_status_callback(self, msg):
        """Callback for move_action status topic"""
        if not msg.status_list:
            return
        
        # Update current status (use most recent status)
        self.current_status = msg.status_list[-1].status
        self.get_logger().info(f"Current follow_joint_trajectory status: {self.current_status}")
        
        if self.current_status == 2:
            # Detect state: when = 2 (EXECUTING)
            self.latest_real_joint_states = None  # Reset real joint states
            self.using_real_states = False  # Switch to fake_joint_states mode
        
        else:
            # When status not 2 (EXECUTING)
            # Publish any value to get_angles_cmd topic
            cmd_msg = Bool()
            self.get_angles_cmd_pub.publish(cmd_msg)
            # Switch to real_joint_states mode
            self.using_real_states = True


def main(args=None):
    rclpy.init(args=args)
    
    joint_state_switcher = JointStateSwitcher()
    
    try:
        rclpy.spin(joint_state_switcher)
    except KeyboardInterrupt:
        joint_state_switcher.get_logger().info('Shutting down Joint State Switcher')
    finally:
        joint_state_switcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
