#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander.moveit_py import MoveItPy
from moveit_commander.robot_state import RobotState
from geometry_msgs.msg import Pose, PoseStamped
import time
import random


class SetTargetPosition(Node):
    def __init__(self):
        super().__init__('set_target_position')
        self.get_logger().info('Initializing RandomMoveIt2Control.')
        
        # MoveIt interfaces will be initialized in initialize()
        self.moveit = None
        self.arm_group = None
        
    def initialize(self):
        """Initialize MoveIt interfaces after node creation"""
        # Initialize MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm_group = self.moveit.get_planning_component("arm")
        
        # Set planning parameters
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_max_acceleration_scaling_factor(0.1)
        
        # Move to named target "up"
        self.get_logger().info('Moving to "up" position...')
        self.arm_group.set_goal_state(configuration_name="up")
        
        plan_result = self.arm_group.plan()
        
        if plan_result:
            self.get_logger().info('Init arm succeeded.')
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
        else:
            self.get_logger().error('Init arm failed!')
            return
        
        # Get current pose
        robot_model = self.moveit.get_robot_model()
        robot_state = self.moveit.get_planning_scene_monitor().get_current_state()
        
        # Log the "up" position pose
        pose_up = robot_state.get_pose("link5")  # Replace with your end-effector link name
        self.get_logger().info(
            f'pose_up: position.x={pose_up.position.x:.6f}, '
            f'position.y={pose_up.position.y:.6f}, '
            f'position.z={pose_up.position.z:.6f}, '
            f'orientation.x={pose_up.orientation.x:.6f}, '
            f'orientation.y={pose_up.orientation.y:.6f}, '
            f'orientation.z={pose_up.orientation.z:.6f}, '
            f'orientation.w={pose_up.orientation.w:.6f}'
        )
        
        # Set target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"  # Adjust frame_id as needed
        
        target_pose.pose.position.x = 0.13695764541625977
        target_pose.pose.position.y = -0.005238924641162157
        target_pose.pose.position.z = 0.3486359715461731
        target_pose.pose.orientation.x = 3.1805826438358054e-05
        target_pose.pose.orientation.y = -4.311262455303222e-05
        target_pose.pose.orientation.z = -3.6135115806246176e-05
        target_pose.pose.orientation.w = 1.0
        
        # Set pose target
        self.get_logger().info('Planning to target position...')
        self.arm_group.set_goal_state(pose_stamped_msg=target_pose, pose_link="link5")
        
        plan_result = self.arm_group.plan()
        
        if plan_result:
            self.get_logger().info('Planning succeeded, moving the arm.')
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            
            # Sleep for 5 seconds
            time.sleep(5)
            
            # Get current pose after movement
            robot_state = self.moveit.get_planning_scene_monitor().get_current_state()
            pose_target = robot_state.get_pose("link5")
            
            self.get_logger().info(
                f'pose_target: position.x={pose_target.position.x:.6f}, '
                f'position.y={pose_target.position.y:.6f}, '
                f'position.z={pose_target.position.z:.6f}, '
                f'orientation.x={pose_target.orientation.x:.6f}, '
                f'orientation.y={pose_target.orientation.y:.6f}, '
                f'orientation.z={pose_target.orientation.z:.6f}, '
                f'orientation.w={pose_target.orientation.w:.6f}'
            )
        else:
            self.get_logger().error('Planning failed!')


def main(args=None):
    rclpy.init(args=args)
    
    node = SetTargetPosition()
    
    # Delayed initialization
    node.initialize()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()