#!/usr/bin/env python3
import tkinter as tk
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot

import math

# --------------------
# Robot connection
# --------------------
mc = MyCobot('/dev/ttyUSB0', 1000000)

joint_limits = [
    (-168, 168),  # Joint 1
    (-135, 135),  # Joint 2
    (-150, 150),  # Joint 3
    (-145, 145),  # Joint 4
    (-165, 165),  # Joint 5
    (-180, 180)   # Joint 6
]

const_vel = 50.0  # Default velocity

# --------------------
# ROS2 Node
# --------------------
class JointPublisher(Node):
    def __init__(self):
        super().__init__('gui_joint_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_state)  # 20 Hz
        self.angles = mc.get_angles()
        self.vel = [const_vel] * 6

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"{i+1}_Joint" for i in range(6)]  # update with real names if needed
        msg.position = [a * math.pi / 180 for a in self.angles]  # convert deg → rad
        msg.velocity = self.vel
        self.pub.publish(msg)

# --------------------
# Tkinter GUI
# --------------------
def start_gui(node: JointPublisher):
    root = tk.Tk()
    root.title("JetCobot Joint Control")

    sliders = []
    current_angles = mc.get_angles()
    if not current_angles or len(current_angles) != 6:
        current_angles = [0, 0, 0, 0, 0, 0]

    def update_angle(i, val):
        node.angles[i] = float(val)
        send_to_robot()

    def send_to_robot():
        mc.send_angles(node.angles, int(const_vel))
        print("Sent to robot:", node.angles)

    # Create sliders
    for i, (low, high) in enumerate(joint_limits):
        frame = tk.Frame(root)
        frame.pack(fill="x")
        label = tk.Label(frame, text=f"Joint {i+1}")
        label.pack(side="left")
        slider = tk.Scale(frame, from_=low, to=high,
                          orient="horizontal", resolution=1,
                          length=400, command=lambda val, i=i: update_angle(i, val))
        slider.set(current_angles[i])
        slider.pack(side="left")
        sliders.append(slider)

    # Button to move robot
    send_btn = tk.Button(root, text="Move Arm", command=send_to_robot)
    send_btn.pack(pady=10)

    root.mainloop()

# --------------------
# Main
# --------------------
def main():
    rclpy.init()
    node = JointPublisher()

    # GUI in a separate thread so ROS can spin
    gui_thread = threading.Thread(target=start_gui, args=(node,), daemon=True)
    gui_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
