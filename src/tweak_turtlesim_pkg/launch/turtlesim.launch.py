from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define the path to the turtlesim node executable
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    return LaunchDescription([
        turtlesim_node
    ])
    
if __name__ == '__main__':
    generate_launch_description()