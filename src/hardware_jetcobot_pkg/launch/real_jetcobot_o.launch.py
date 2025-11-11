from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch
import os

def generate_launch_description():
    
    robot_description_content = Command(['xacro ', PathJoinSubstitution([
        FindPackageShare('moveit_jetcobot_pkg'),
        'config',
        'jetcobot.urdf.xacro'
    ])])
    
    robot_description = {'robot_description': robot_description_content}
    
    controller_config = PathJoinSubstitution([
                            FindPackageShare('moveit_jetcobot_pkg'),
                            'config',
                            'ros2_controllers.yaml'
                        ])
    
    # Include robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_jetcobot_pkg'),
                'launch',
                'rsp.launch.py'
            ])
        ])
    )
    
    # Include static TF
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_jetcobot_pkg'),
                'launch',
                'static_virtual_joint_tfs.launch.py'
            ])
        ])
    )
    
    # Start ros2_control node with real hardware
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,  # Your URDF with ros2_control
            controller_config
        ],
        output='screen'
    )
    
    # Spawn controllers
    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_jetcobot_pkg'),
                'launch',
                'spawn_controllers.launch.py'
            ])
        ])
    )
    
    # Start MoveIt
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_jetcobot_pkg'),
                'launch',
                'move_group.launch.py'
            ])
        ])
    )
    
    # Start RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_jetcobot_pkg'),
                'launch',
                'moveit_rviz.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        rsp,
        static_tf,
        ros2_control_node,
        spawn_controllers,
        move_group,
        rviz
    ])