#!/usr/bin/env python3

from ros2_control_py import HardwareInterface
from pymycobot.mycobot import MyCobot
import math


class MyCobotHardware(HardwareInterface):    
    def on_init(self, hardware_info):
        """Initialize hardware - called once at startup"""
        # Get parameters from URDF
        port = hardware_info.hardware_parameters.get("serial_port", "/dev/ttyUSB0")
        baud = int(hardware_info.hardware_parameters.get("baud_rate", "1000000"))
        
        # Connect to robot
        self.robot = MyCobot(port, baud)
        
        # Get joint names from URDF
        self.joint_names = [joint.name for joint in hardware_info.joints]
        self.num_joints = len(self.joint_names)
        
        # Initialize arrays
        self.positions = [0.0] * self.num_joints
        self.velocities = [0.0] * self.num_joints
        self.commands = [0.0] * self.num_joints
        
        print(f"MyCobot initialized: {self.num_joints} joints on {port}")
        return True
    
    def on_configure(self, previous_state):
        """Configure hardware - open connection"""
        # Read initial position
        angles = self.robot.get_angles()
        if angles:
            self.positions = [math.radians(a) for a in angles]
            self.commands = self.positions.copy()
        return True
    
    def on_activate(self, previous_state):
        """Activate hardware - ready to move"""
        print("MyCobot activated and ready")
        return True
    
    def on_deactivate(self, previous_state):
        """Deactivate hardware"""
        return True
    
    def on_cleanup(self, previous_state):
        """Cleanup - close connection"""
        return True
    
    def read(self, time, period):
        """Read joint states from robot - called every control cycle"""
        try:
            # Get current angles from robot (in degrees)
            angles = self.robot.get_angles()
            
            if angles and len(angles) == self.num_joints:
                # Convert to radians
                new_positions = [math.radians(a) for a in angles]
                
                # Calculate velocity (simple derivative)
                dt = period.nanoseconds / 1e9
                if dt > 0:
                    for i in range(self.num_joints):
                        self.velocities[i] = (new_positions[i] - self.positions[i]) / dt
                
                self.positions = new_positions
                return True
            
            return False
            
        except Exception as e:
            print(f"Read error: {e}")
            return False
    
    def write(self, time, period):
        """Write commands to robot - called every control cycle"""
        try:
            # Convert commands from radians to degrees
            angles = [math.degrees(cmd) for cmd in self.commands]
            
            # Send to robot (speed 50 = medium speed)
            self.robot.send_angles(angles, 50)
            return True
            
        except Exception as e:
            print(f"Write error: {e}")
            return False
    
    def export_state_interfaces(self):
        """Export what states controllers can read"""
        interfaces = []
        for i, name in enumerate(self.joint_names):
            interfaces.append({
                'name': f'{name}/position',
                'value': lambda i=i: self.positions[i]
            })
            interfaces.append({
                'name': f'{name}/velocity', 
                'value': lambda i=i: self.velocities[i]
            })
        return interfaces
    
    def export_command_interfaces(self):
        """Export what commands controllers can write"""
        interfaces = []
        for i, name in enumerate(self.joint_names):
            interfaces.append({
                'name': f'{name}/position',
                'value': lambda i=i: self.commands[i],
                'setter': lambda val, i=i: self.commands.__setitem__(i, val)
            })
        return interfaces


# Plugin entry point for ros2_control
def create_hardware_interface():
    return MyCobotHardware()