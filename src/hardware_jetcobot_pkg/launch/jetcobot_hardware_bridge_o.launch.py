from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch
import os

def generate_launch_description():
    # Load robot description
    robot_description_content = Command(['xacro ', PathJoinSubstitution([
        FindPackageShare('moveit_jetcobot_pkg'),
        'config',
        'jetcobot.urdf.xacro'
    ])])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Load controller configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('moveit_jetcobot_pkg'),
        'config',
        'ros2_controllers.yaml'
    ])
    
    # Controller manager node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )
    
    # Arm controller spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager']
    )
    
    # MoveIt node
    moveit_config = (
        MoveItConfigsBuilder("jetcobot", package_name="moveit_jetcobot_pkg")
        .robot_description(file_path="config/jetcobot.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()]
    )
    
    # RViz
    rviz_config = PathJoinSubstitution([
        FindPackageShare('moveit_jetcobot_pkg'),
        'config',
        'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[moveit_config.robot_description,
                   moveit_config.robot_description_semantic,
                   moveit_config.robot_description_kinematics]
    )
    
    return LaunchDescription([
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node
    ])