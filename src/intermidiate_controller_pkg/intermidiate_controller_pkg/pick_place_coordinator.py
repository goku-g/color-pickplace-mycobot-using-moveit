#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from enum import Enum
import time


class RobotState(Enum):
    """State machine states for pick-and-place operation"""
    IDLE = 0
    GO_TO_START = 1
    WAIT_FOR_OBJECT = 2
    BOUNDARY_CHECK = 3
    GO_TO_PRE_GRASP = 4
    WAIT_FOR_ARM_1 = 5
    OPEN_GRIPPER_1 = 6
    WAIT_FOR_OPEN_1 = 7
    GO_TO_GRASP = 8
    WAIT_FOR_ARM_2 = 9
    CLOSE_GRIPPER = 10
    WAIT_FOR_GRASP_1 = 11
    GO_TO_FINAL = 12
    WAIT_FOR_ARM_3 = 13
    OPEN_GRIPPER_2 = 14
    WAIT_FOR_OPEN_2 = 15


class PickPlaceCoordinator(Node):
    def __init__(self):
        super().__init__('pick_place_coordinator')
        
        # Publishers
        self.target_pose_pub = self.create_publisher(
            Float64MultiArray, 
            '/target_pose_xyz', 
            10
        )
        
        self.gripper_state_pub = self.create_publisher(
            Bool,
            '/gripper_action_command',
            100
        )
        
        # Subscribers
        self.object_pose_sub = self.create_subscription(
            Float64MultiArray,
            '/detected_object_pose',
            self.object_pose_callback,
            10
        )
        
        self.arm_status_sub = self.create_subscription(
            Bool,
            '/arm_execution_status',
            self.arm_status_callback,
            10
        )
        
        self.gripper_status_sub = self.create_subscription(
            Bool,
            '/gripper_action_status',
            self.gripper_completion_callback,
            10
        )
        
        # State machine
        self.current_state = RobotState.IDLE
        self.arm_execution_complete = False
        self.detected_pose = None
        
        self.gripper_action_complete = True  # Start as False
        
        # Timeout tracking
        self.state_entry_time = self.get_clock().now()
        self.STATE_TIMEOUT = 30.0  # 60 seconds timeout
        self.STATE_TRANSITION_TIME = 5.0  # 5 seconds between states
        self.TIMEOUT_EXEMPT_STATES = [RobotState.WAIT_FOR_OBJECT, RobotState.IDLE]
        
        # Configuration parameters
        self.START_POSE = [-0.01, -0.06, 0.41, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
        self.FINAL_POSE = [0.13, 0.0, 0.34, 0.0, 0.0, 0.0]
        self.PRE_GRASP_OFFSET_Z = 0.03  # 3cm above object
        
        # Boundary limits [x_min, x_max, y_min, y_max, z_min, z_max]
        self.BOUNDARY = [0.2, 0.3, -0.2, 0.2, 0.05, 0.3]
        
        # Timer for state machine execution
        self.state_timer = self.create_timer(0.1, self.state_machine_update)
        
        self.get_logger().info('Pick-Place Coordinator Node Initialized')
        self.get_logger().info(f'Current State: {self.current_state.name}')
    
    
    def object_pose_callback(self, msg):
        """Callback for detected object pose"""
        if len(msg.data) == 6:
            # Only accept new poses when waiting for object
            if self.current_state == RobotState.WAIT_FOR_OBJECT:
                self.detected_pose = list(msg.data)
                self.get_logger().info(f'Received object pose: {self.detected_pose}')
        else:
            self.get_logger().warn(f'Invalid pose data length: {len(msg.data)}')
    
    
    def arm_status_callback(self, msg):
        """Callback for arm execution status"""
        self.arm_execution_complete = msg.data
        if msg.data:
            self.get_logger().info('Arm execution completed successfully')
    
    
    def gripper_completion_callback(self, msg):
        """Callback for gripper action status"""
        self.gripper_action_complete = msg.data
        if msg.data:
            self.get_logger().info('Gripper action completed successfully')
    
    
    def publish_target_pose(self, pose):
        """Publish target pose to /target_pose_xyz"""
        msg = Float64MultiArray()
        msg.data = pose
        self.target_pose_pub.publish(msg)
        self.get_logger().info(f'Published target pose: {pose}')
        self.arm_execution_complete = False  # Reset flag
    
    
    def is_within_boundary(self, pose):
        """Check if pose is within safe working boundaries"""
        x, y, z = pose[0], pose[1], pose[2]
        
        within = (self.BOUNDARY[0] <= x <= self.BOUNDARY[1] and
                  self.BOUNDARY[2] <= y <= self.BOUNDARY[3] and
                  self.BOUNDARY[4] <= z <= self.BOUNDARY[5])
        
        if not within:
            self.get_logger().warn(f'Pose {pose[:3]} is outside boundary limits')
        
        return within
    
    
    def open_gripper(self):
        """Open gripper command"""
        self.get_logger().info('Commanding gripper to open...')
        self.gripper_state_pub.publish(Bool(data=False))
        # self.gripper_action_complete = False  # Reset flag
    
    
    def close_gripper(self):
        """Close gripper command"""
        self.get_logger().info('Commanding gripper to close...')
        self.gripper_state_pub.publish(Bool(data=True))
        # self.gripper_action_complete = False  # Reset flag
    
    
    def reset_state_machine(self):
        """Reset state machine to IDLE"""
        self.get_logger().error('Resetting state machine to IDLE...')
        self.detected_pose = None
        self.arm_execution_complete = False
        self.gripper_action_complete = True
        self.transition_to(RobotState.IDLE)
    
    
    def state_machine_update(self):
        """Main state machine logic"""
        
        # Check for timeout (except in WAIT_FOR_OBJECT and IDLE states)
        if self.current_state not in self.TIMEOUT_EXEMPT_STATES:
            time_in_state = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
            
            if time_in_state > self.STATE_TIMEOUT:
                self.get_logger().error(
                    f'TIMEOUT! Stuck in state {self.current_state.name} for {time_in_state:.1f}s.'
                )
                self.reset_state_machine()
                return
        
        if self.current_state == RobotState.IDLE:
            self.get_logger().info('Starting pick-place cycle...')
            self.transition_to(RobotState.GO_TO_START)
        
        elif self.current_state == RobotState.GO_TO_START:
            self.close_gripper()
            time.sleep(1.5)  # brief pause after gripper action to grasp
            self.publish_target_pose(self.START_POSE)
            self.transition_to(RobotState.WAIT_FOR_OBJECT)
        
        elif self.current_state == RobotState.WAIT_FOR_OBJECT:
            if self.detected_pose is not None:
                self.get_logger().info('Object detected, checking boundaries...')
                self.transition_to(RobotState.BOUNDARY_CHECK)
        
        elif self.current_state == RobotState.BOUNDARY_CHECK:
            if self.is_within_boundary(self.detected_pose):
                self.get_logger().info('Object within boundaries, proceeding to grasp')
                self.transition_to(RobotState.GO_TO_PRE_GRASP)
            else:
                self.get_logger().warn('Object outside boundaries, waiting for new detection')
                self.detected_pose = None
                self.transition_to(RobotState.WAIT_FOR_OBJECT)
        
        elif self.current_state == RobotState.GO_TO_PRE_GRASP:
            pre_grasp_pose = self.detected_pose.copy()
            pre_grasp_pose[2] += self.PRE_GRASP_OFFSET_Z  # Move 3cm above
            self.publish_target_pose(pre_grasp_pose)
            self.transition_to(RobotState.WAIT_FOR_ARM_1)
        
        elif self.current_state == RobotState.WAIT_FOR_ARM_1:
            if self.arm_execution_complete:
                time.sleep(2)  # brief pause before opening gripper
                self.transition_to(RobotState.OPEN_GRIPPER_1)
        
        elif self.current_state == RobotState.OPEN_GRIPPER_1:
            self.open_gripper()
            self.transition_to(RobotState.WAIT_FOR_OPEN_1)
        
        elif self.current_state == RobotState.WAIT_FOR_OPEN_1:
            if self.gripper_action_complete:
                time.sleep(1.5)  # brief pause after gripper action to grasp
                self.transition_to(RobotState.GO_TO_GRASP)
        
        elif self.current_state == RobotState.GO_TO_GRASP:
            self.publish_target_pose(self.detected_pose)
            self.transition_to(RobotState.WAIT_FOR_ARM_2)
        
        elif self.current_state == RobotState.WAIT_FOR_ARM_2:
            if self.arm_execution_complete:
                time.sleep(2)  # brief pause before opening gripper
                self.transition_to(RobotState.CLOSE_GRIPPER)
        
        elif self.current_state == RobotState.CLOSE_GRIPPER:
            self.close_gripper()
            self.transition_to(RobotState.WAIT_FOR_GRASP_1)
        
        elif self.current_state == RobotState.WAIT_FOR_GRASP_1:
            if self.gripper_action_complete:
                time.sleep(1.5)  # brief pause after gripper action to grasp
                self.transition_to(RobotState.GO_TO_FINAL)
        
        elif self.current_state == RobotState.GO_TO_FINAL:
            self.publish_target_pose(self.FINAL_POSE)
            self.transition_to(RobotState.WAIT_FOR_ARM_3)
        
        elif self.current_state == RobotState.WAIT_FOR_ARM_3:
            if self.arm_execution_complete:
                time.sleep(2)  # brief pause before opening gripper
                self.transition_to(RobotState.OPEN_GRIPPER_2)
        
        elif self.current_state == RobotState.OPEN_GRIPPER_2:
            self.open_gripper()
            self.transition_to(RobotState.WAIT_FOR_OPEN_2)
        
        elif self.current_state == RobotState.WAIT_FOR_OPEN_2:
            if self.gripper_action_complete:
                time.sleep(1.5)  # brief pause after gripper action to grasp
                self.get_logger().info('Pick-place cycle complete! Restarting...')
                self.detected_pose = None  # Reset for next cycle
                self.transition_to(RobotState.GO_TO_START)
    
    
    def transition_to(self, new_state):
        """Transition to new state"""
        self.get_logger().info(f'State: {self.current_state.name} -> {new_state.name}')
        self.current_state = new_state
        self.state_entry_time = self.get_clock().now()  # Reset timeout timer


def main():
    rclpy.init()
    node = PickPlaceCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()