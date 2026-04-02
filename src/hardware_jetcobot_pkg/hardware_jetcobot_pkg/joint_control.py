import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatusArray
import time
import math
from pymycobot.mycobot import MyCobot
    
class Joint_controller(Node):
    def __init__(self):
        super().__init__("control_slider")
        self.state_sub = self.create_subscription(
            JointState,
            # "joint_commands",
            "joint_states",
            self.listener_callback,
            1
        )
        self.sub_get_angles_cmd = self.create_subscription(
            Bool,
            "get_angles_cmd",
            self.get_radians_cmd_callback,
            1
        )

        self.move_action_status_sub = self.create_subscription(
            GoalStatusArray,
            '/arm_controller/follow_joint_trajectory/_action/status',
            self.move_action_status_callback,
            10
        )
        
        self.move_action_status_sub = self.create_subscription(
            Bool,
            '/gripper_action_command',
            self.handler_gripper_callback,
            100
        )

        self.pub = self.create_publisher(JointState, "real_joint_states", 1)
        self.gripper_msg = False
        
        # Track move_action status
        self.move_action_status = None
        self.should_stop_movement = True
        
        # Timer to automatically execute get_radians_cmd
        self.get_radians_timer = self.create_timer(1.0, self.auto_get_radians_callback)

        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        time.sleep(0.4)  # wait for connection to establish

    def listener_callback(self, msg):
        # self.get_logger().info(f"Callback called! Received joint commands: {msg.position}")
        
        # Stop sending angles when follow_joint_trajectory status is 4 (SUCCEEDED)
        if self.should_stop_movement:
            # self.get_logger().info("Movement paused due to follow_joint_trajectory status")
            return
            
        # MyCobot joint order: 1, 4, 2, 3, 5, 6
        # ROS joint order: 1, 2, 3, 4, 5, 6
        data_list = [0] * 6
        index_map = [0, 2, 3, 1, 4, 5]
        i = 0
        for _, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list[index_map[i]] = radians_to_angles
            i += 1

        # self.get_logger().info(f"Sending angles to MyCobot: {data_list}")
        self.mc.send_angles(data_list, 50)

    def move_action_status_callback(self, msg):
        """Callback for move_action status topic"""
        if not msg.status_list:
            return
        
        # Check the most recent status
        self.move_action_status = msg.status_list[-1].status
        self.get_logger().info(f"Current follow_joint_trajectory status: {self.move_action_status}")
        
        # If status is 2 (EXECUTING), allow movement
        if self.move_action_status == 2:
            # Resume movement for other statuses
            if self.should_stop_movement:
                self.get_logger().warn(f"follow_joint_trajectory status changed to {self.move_action_status} - resuming send_angles")
                self.should_stop_movement = False
        else:
             # If status is not 2 (EXECUTING), stop movement     
             if not self.should_stop_movement:
                self.get_logger().info("follow_joint_trajectory SUCCEEDED (status 4) - stopping send_angles")
                self.should_stop_movement = True

    def auto_get_radians_callback(self):
        """Automatically execute get_radians when move_action status is not 2 (EXECUTING)"""
        # Run only when move_action_status is None or not 2 (EXECUTING)
        if self.move_action_status is None or self.move_action_status != 2:
            self.get_radians_cmd_callback(None)

    def get_radians_cmd_callback(self,_):
        joint_state = JointState()
        
        # Retry up to 5 times
        attempt = 0
        for attempt in range(5):
            # time.sleep(0.2)
            angles = self.mc.get_angles()
            # Check if angles exist, are a list, and contain all 6 values
            if angles is not None and isinstance(angles, list):
                # self.get_logger().info(f"Real robot angles: {angles}")
                # Convert angles to radians
                index_map = [0, 2, 3, 1, 4, 5]
                joint_state.position = [math.radians(angles[i]) for i in index_map]
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = [
                    "1_Joint",
                    "3_Joint",
                    "4_Joint",
                    "2_Joint",
                    "5_Joint",
                    "6_Joint",
                ]
                joint_state.velocity = [0.0] * len(joint_state.name)
                joint_state.effort = [0.0] * len(joint_state.name)
                self.pub.publish(joint_state)
                
                # self.get_logger().info(f"Published real_joint_states: {joint_state.position}")
                break
            else:
                self.get_logger().warn(f"Failed to get 6 joint angles, attempt {attempt + 1}/5. Got: {angles}")
                if attempt == 4:  # Last attempt
                    self.get_logger().error("Failed to get valid joint angles after 5 attempts")
    
    def handler_gripper_callback(self, msg):
        """Callback for gripper command"""
        self.gripper_msg = msg.data
        self.get_logger().info(f"Executing Gripper Action... Action: {int(self.gripper_msg)}")
        self.mc.set_gripper_state(int(self.gripper_msg), 2)
               
def main(args=None):
    rclpy.init(args=args)
    joint_controller = Joint_controller()

    try:
        rclpy.spin(joint_controller)
    except KeyboardInterrupt:
        joint_controller.get_logger().info('Shutting down Joint State Switcher')
    finally:
        joint_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
